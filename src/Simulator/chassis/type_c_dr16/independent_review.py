"""Read-only numerical acceptance of the frozen DR16 simulation artifacts.

No simulator/report module is imported or executed. This reviewer checks the
recorded inputs and transitions, not the controller's PI implementation. A PASS
is software evidence only; executor/UI/receipt acceptance belongs to the caller.
"""
import argparse
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


class Checks:
    def __init__(self):
        self.count = 0
        self.failures = 0
        self.issues = []

    def require(self, condition, message):
        self.count += 1
        if not condition:
            self.failures += 1
            if len(self.issues) < 100:
                self.issues.append(message)


def review(task_path, output, smoke=False, preflight_identity=None):
    c = Checks()
    task = read_json(task_path) if task_path else {}
    output = Path(output).resolve()
    summary = read_json(output / 'summary.json')
    runner = read_json(output / 'runner_summary.json')
    build = read_json(output / 'build_manifest.json')
    identity = preflight_identity or (('CODEX-PREBUILT-SMOKE', 'entry-smoke-20261105') if smoke else (task['job_id'], task['nonce']))
    for name, obj in [('summary', summary), ('runner', runner), ('build', build)]:
        c.require((obj['job_id'], obj['nonce']) == identity, name + ' identity')
        c.require(obj['software_only'] is True, name + ' software-only scope')
    c.require(build['runner_exit'] == 0 and build['inputs_unchanged'] is True, 'runner exit/input unchanged')
    c.require(Path(build['output_dir']).resolve() == output, 'build output directory')
    if not smoke and not preflight_identity:
        c.require(output == Path(task['output_dir']).resolve() / 'run', 'task output directory')
    c.require(summary['build'] == build, 'embedded build manifest matches artifact')
    c.require(summary['scenarios'] == runner['scenarios'], 'summary/runner scenarios match')
    c.require(summary['status'] == runner['status'] == 'PASS', 'reported status')
    c.require(not summary['csv_recomputation']['issues'], 'reported CSV issues empty')
    c.require(summary['left_vertical_policy'] == runner['left_vertical_policy'] == 'ignore', 'unused left V policy')
    c.require(summary['model'] == runner['model'] and summary['limits'] == runner['limits'], 'model/limits consistency')
    c.require(runner['model'] == {'kind': 'assumed_first_order_rotor', 'gain_rpm_per_raw': 0.8,
              'tau_ms': 180.0, 'dt_ms': 1, 'can_command_hold_ms': 5,
              'rc_frame_ms': 14, 'motor_feedback_ms': [1, 1, 8, 1, 1]}, 'required simulation parameters')
    c.require(runner['limits'] == {'max_current_raw': 1500, 'max_target_rotor_rpm': 300, 'active_state': 2}, 'required limits')
    started = datetime.fromisoformat(build['started_at'])
    finished = datetime.fromisoformat(build['finished_at'])
    generated = datetime.fromisoformat(summary['generated_at_utc'])
    c.require(started.tzinfo is not None and started <= finished <= generated, 'automatic timestamps ordered')
    hashes = {}
    hash_groups = ({'preflight_build_inputs': {x['path']: x['sha256'] for x in build['inputs']}}
                   if preflight_identity else {k: task['input'][k] for k in ('source_sha256', 'working_copy_sha256')})
    for group, entries in hash_groups.items():
        for path, expected in entries.items():
            actual = sha(path)
            hashes[path] = actual
            c.require(actual == expected, group + ': ' + path)
    frozen = hash_groups['preflight_build_inputs'] if preflight_identity else task['input']['working_copy_sha256']
    for entry in build['inputs']:
        c.require(entry['path'] in frozen and entry['sha256'] == frozen.get(entry['path']), 'build input hash: ' + entry['path'])
    input_dir = Path(build['source_dir']).resolve()
    if preflight_identity:
        c.require(build['runner_sha256'] == sha(output / 'model_runner.exe'), 'preflight executable identity')
    else:
        prebuilt = read_json(input_dir / 'prebuilt_manifest.json')
        c.require({Path(x['path']).resolve() for x in build['inputs']} ==
                  {Path(p).resolve() for p in frozen if Path(p).name != 'prebuilt_manifest.json'}, 'complete build input list')
        c.require(build['runner_sha256'] == prebuilt['runner_sha256'] == sha(input_dir / 'model_runner.exe'), 'frozen executable identity')
        for rel, expected in prebuilt['input_sha256'].items():
            c.require(sha(input_dir / rel) == expected, 'prebuilt source binding: ' + rel)
    for key, filename in [('raw_csv', 'raw.csv'), ('runner_summary', 'runner_summary.json'), ('report_html', 'report.html')]:
        c.require(summary['artifacts'][key] == filename, 'artifact name ' + key)
        c.require(summary['artifacts'][key + '_sha256'] == sha(output / filename), 'artifact hash ' + filename)
    html = (output / 'report.html').read_text(encoding='utf-8')
    c.require(all(x in html for x in identity), 'HTML identity')

    scenarios = {}
    rows_total = 0
    with (output / 'raw.csv').open(newline='', encoding='utf-8-sig') as f:
        for raw in csv.DictReader(f):
            sid = raw['scenario']
            r = {k: (v if k in ('job_id', 'nonce', 'scenario', 'phase') else
                     float(v) if k.startswith('model_') else int(v)) for k, v in raw.items()}
            t = r['sim_ms']
            if sid not in scenarios:
                scenarios[sid] = {'rows': 0, 'active_rows': 0, 'peak_current_raw': 0,
                    'first_fault_ms': None, 'phases': {}, 'transitions': [], 'clear_events': [], 'feedback_clear_events': [],
                    'feedback_counts': [0] * 4, 'feedback_ms': [None] * 4,
                    'last_rc_ms': None, 'last_switch': None, 'neutral': False,
                    'off_start': None, 'last_off': None, 'qualification': None,
                    'queued_zero': False,
                    'previous': None, 'saturated_rows': 0, 'peak_model_rpm': 0.0,
                    'error_squared': 0.0, 'motor203_loss_last_feedback_ms': None}
            s = scenarios[sid]
            prev = s['previous']
            ctx = f'{sid}@{t}: '
            c.require((r['job_id'], r['nonce']) == identity, ctx + 'CSV identity')
            c.require(t == s['rows'], ctx + 'consecutive 1 ms samples')
            c.require(r['state'] in (0, 1, 2, 3), ctx + 'valid state')
            currents = [r[f'command_{i:x}_raw'] for i in range(0x201, 0x209)]
            targets = [r[f'target_{i:x}_millirpm'] for i in range(0x201, 0x205)]
            c.require(max(map(abs, currents)) <= 1500, ctx + 'current limit')
            c.require(all(x == 0 for x in currents[4:]), ctx + 'non-drive outputs zero')
            c.require(r['state'] == 2 or all(x == 0 for x in currents), ctx + 'non-ACTIVE outputs zero')
            c.require(max(map(abs, targets)) <= 300000, ctx + 'target speed limit')
            c.require((r['fault'] != 0) == (r['state'] == 3), ctx + 'fault/state coherence')
            if r['rc_packet_sent']:
                c.require(r['rc_enabled'] == 1, ctx + 'RC packet requires enabled stream')
                s['last_rc_ms'] = t
                s['last_switch'] = r['switch_left']
                s['neutral'] = all(r[k] == 0 for k in ('command_right_milli', 'command_forward_milli', 'command_cw_milli'))
            age = t - s['last_rc_ms'] if s['last_rc_ms'] is not None else 0xffffffff
            c.require(r['rc_age_ms'] == age, ctx + 'RC timestamp reconstruction')
            fresh = True
            still = True
            for j, mid in enumerate(range(0x201, 0x205)):
                count = r[f'feedback_{mid:x}_count']
                received = bool(r['feedback_mask'] & (1 << j)) and (j != 2 or t % 8 == 0)
                received &= not (j == 2 and r.get('corrupt_203', 0))
                c.require(count == s['feedback_counts'][j] + int(received), ctx + f'{mid:x} feedback cadence/count')
                if received:
                    s['feedback_ms'][j] = t
                s['feedback_counts'][j] = count
                fresh &= count >= 8 and s['feedback_ms'][j] is not None and t - s['feedback_ms'][j] <= 30
                still &= abs(r[f'feedback_{mid:x}_rpm']) <= 5
                model = r[f'model_{mid:x}_rpm']
                c.require(math.isfinite(model), ctx + 'finite model speed')
                s['peak_model_rpm'] = max(s['peak_model_rpm'], abs(model))
                s['error_squared'] += (targets[j] / 1000.0 - model) ** 2
            rc_good = bool(r['rc_valid']) and age < 50 and r['rc_consecutive'] >= 5
            safe_down = rc_good and fresh and still and s['neutral'] and s['last_switch'] == 2
            if r['state'] == 2:
                c.require(rc_good and fresh and s['last_switch'] == 1 and r['can_healthy'] == 1 and r['policy_fault_flags'] == 0, ctx + 'ACTIVE authorization')
            clear = r['controller_clear_count'] - (prev['controller_clear_count'] if prev else 0)
            feedback_clear = r.get('feedback_clear_count', 0) - (prev.get('feedback_clear_count', 0) if prev else 0)
            c.require(clear in (0, 1), ctx + 'controller clear count monotonic')
            c.require(feedback_clear in (0, 1), ctx + 'feedback clear count monotonic')
            if feedback_clear:
                c.require(prev is not None and prev['state'] == 3 and prev['fault'] == 3 and safe_down
                          and r['state'] == 0 and r['can_healthy'] == 1 and r['policy_fault_flags'] == 0
                          and r['feedback_temperature_c'] < 70 and r['feedback_clear_ms'] == t,
                          ctx + 'feedback clear requires private safe DOWN and healthy link')
                s['feedback_clear_events'].append({'at_ms': t})
                s['off_start'] = None
            if r['policy_enabled']:
                # Independently count successful zero-command transmissions from
                # queue observations and RQCP|TXOK, rather than trusting the
                # cumulative counter used in the recovery summary.
                tx_ok = (r['policy_mailbox_status'] & 3) == 3
                zero_delta = 2 if tx_ok and s['queued_zero'] else 0
                prior_zero_count = prev['policy_zero_tx_count'] if prev else 0
                prior_zero_time = prev['policy_last_zero_tx_ms'] if prev else 0
                c.require(r['policy_zero_tx_count'] == prior_zero_count + zero_delta,
                          ctx + 'zero TX counter reconstructed from successful queued pair')
                c.require(r['policy_last_zero_tx_ms'] == (t if zero_delta else prior_zero_time),
                          ctx + 'zero TX timestamp reconstruction')
                if tx_ok:
                    c.require(prev is not None and prev['policy_pending'] == 1,
                              ctx + 'TXOK completes a prior pending request')
                if r['policy_pending'] and r['policy_pending_age_ms'] == 0:
                    s['queued_zero'] = all(x == 0 for x in currents[:4])
                if r['phase'] == 'stuff_pending_retry':
                    c.require(r['policy_esr'] == 0x01000010 and r['policy_mailbox_status'] == 8
                              and r['policy_pending'] == 1 and r['policy_pending_age_ms'] == 0
                              and r['policy_fault_flags'] == 0, ctx + 'actual transient stuff/retry observations')
                if r['phase'] == 'retry_success_1ms':
                    c.require(r['policy_esr'] == 0 and tx_ok and r['policy_pending'] == 0,
                              ctx + 'actual retry success observation')
                if r['phase'] == 'pending_below_deadline':
                    c.require(r['policy_pending'] == 1 and r['policy_pending_age_ms'] ==
                              t - s['phases'].get('pending_below_deadline', {'first_ms': t})['first_ms'],
                              ctx + 'pending age reconstructed from initial request')
                if r['phase'] == 'pending_reaches_3ms':
                    c.require(prev is not None and prev['policy_pending_age_ms'] == 2
                              and r['policy_pending'] == 0 and r['policy_fault_flags'] & 128,
                              ctx + 'pending request aborts at deadline')
                safe_request = bool(prev and prev['state'] == 3 and prev['fault'] in (1, 3) and safe_down
                                    and r.get('feedback_temperature_c', 25) < 70)
                c.require(bool(r['policy_safe_off']) == safe_request, ctx + 'independent safe-OFF gate')
                flags = prev['policy_fault_flags'] if clear else r['policy_fault_flags']
                eligible = bool(flags and not flags & ~0xc8 and safe_request and r['policy_esr'] == 0
                    and r['policy_zero_tx_count'] > 0 and t - r['policy_last_zero_tx_ms'] <= 20)
                if not eligible:
                    s['qualification'] = None
                elif s['qualification'] is None or s['qualification'][2] != r['policy_error_generation']:
                    s['qualification'] = (t, r['policy_zero_tx_count'], r['policy_error_generation'])
                if clear:
                    q = s['qualification']
                    elapsed = t - q[0] if q else -1
                    delta = r['policy_zero_tx_count'] - q[1] if q else -1
                    c.require(q is not None and elapsed >= 1000 and delta >= 20, ctx + 'independent clean dwell/zero TX minimum')
                    c.require(r['policy_pending'] == 0 and r['policy_fault_flags'] == 0, ctx + 'recovery release drained and flags cleared')
                    c.require(r['policy_clear_count'] == r['controller_clear_count'] and r['state'] == 0, ctx + 'policy/controller clear to DISABLED only')
                    c.require(elapsed == r['policy_clear_elapsed_ms'] and delta == r['policy_clear_zero_tx_delta'], ctx + 'reported recovery metrics')
                    s['clear_events'].append({'at_ms': t, 'clean_start_ms': q[0] if q else None,
                        'clean_elapsed_ms': elapsed, 'zero_tx_delta': delta})
                    s['qualification'] = None
                    s['off_start'] = None
                c.require(bool(r['policy_qualifying']) == (s['qualification'] is not None), ctx + 'recomputed policy qualification')
                if s['qualification']:
                    c.require(r['policy_qualifying_ms'] == t - s['qualification'][0], ctx + 'qualification clock')
            if r['state'] not in (2, 3) and safe_down:
                if s['off_start'] is None:
                    s['off_start'] = t
                s['last_off'] = t
            elif r['state'] == 3 or r['state'] == 0 or r['invalid_injected']:
                s['off_start'] = None
            if r['state'] == 1:
                c.require(s['off_start'] is not None and t - s['off_start'] >= 500 and safe_down or
                          prev is not None and prev['state'] == 1 and rc_good and fresh and still and s['neutral'] and s['last_switch'] == 3
                          and s['last_off'] is not None and t - s['last_off'] <= 2000, ctx + 'READY needs OFF-neutral dwell or valid MID transit')
            if prev and prev['state'] != r['state']:
                s['transitions'].append({'at_ms': t, 'from': prev['state'], 'to': r['state'], 'phase': r['phase']})
                if r['state'] == 2:
                    c.require(prev['state'] == 1 and s['neutral'] and still and s['last_switch'] == 1, ctx + 'explicit neutral UP from READY')
                if prev['state'] == 3:
                    c.require(clear + feedback_clear == 1 and r['state'] == 0, ctx + 'fault only leaves through qualified clear')
            if r['invalid_injected']:
                c.require(r['state'] == 0 and all(x == 0 for x in currents), ctx + 'invalid then valid same tick stays disarmed')
            if r['phase'] == 'id203_missing' and s['motor203_loss_last_feedback_ms'] is None:
                s['motor203_loss_last_feedback_ms'] = s['feedback_ms'][2]
            if r['state'] == 3 and s['first_fault_ms'] is None:
                s['first_fault_ms'] = t
            p = s['phases'].setdefault(r['phase'], {'first_ms': t, 'last_ms': t, 'rows': 0, 'states': set(), 'max_current': 0})
            p['rows'] += 1
            p['last_ms'] = t
            p['states'].add(r['state'])
            p['max_current'] = max(p['max_current'], max(map(abs, currents)))
            s['rows'] += 1
            s['active_rows'] += int(r['state'] == 2)
            s['saturated_rows'] += int(1500 in map(abs, currents))
            s['peak_current_raw'] = max(s['peak_current_raw'], max(map(abs, currents)))
            s['previous'] = r
            rows_total += 1

    expected_ids = task.get('scenario_ids', [x['id'] for x in runner['scenarios']])
    c.require(list(scenarios) == expected_ids, 'complete ordered scenario list')
    expected_rows = next((x['expected'] for x in task.get('acceptance', []) if x['metric'] == 'raw_rows'),
                         sum(x['rows'] for x in runner['scenarios']))
    c.require(rows_total == expected_rows == summary['csv_recomputation']['rows'], 'raw sample total')
    if preflight_identity:
        c.require(len(scenarios) == 14, '14 feedback-recovery regression scenarios')
    metrics = {}
    for reported in runner['scenarios']:
        sid = reported['id']
        s = scenarios[sid]
        last = s['previous']
        m = {k: s[k] for k in ('rows', 'active_rows', 'peak_current_raw', 'first_fault_ms', 'saturated_rows')}
        m.update(final_state=last['state'], final_fault=last['fault'], arm_count=last['arm_count'],
                 policy_clears=last['policy_clear_count'], controller_clears=last['controller_clear_count'])
        if 'feedback_clears' in reported:
            m['feedback_clears'] = last['feedback_clear_count']
        for k, value in m.items():
            c.require(reported[k] == value, sid + ': runner metric ' + k)
        for k, value in summary['csv_recomputation']['scenarios'][sid].items():
            c.require(value == (last['sim_ms'] if k == 'last_ms' else m[k]), sid + ': summary metric ' + k)
        c.require(abs(reported['peak_model_rpm'] - s['peak_model_rpm']) < 0.000002, sid + ': peak model speed')
        rmse = math.sqrt(s['error_squared'] / (s['rows'] * 4))
        c.require(abs(reported['model_tracking_rmse_rpm'] - rmse) < 0.000002, sid + ': model tracking RMSE')
        c.require(reported['passed'] and reported['failures'] == 0, sid + ': runner assertions passed')
        m['transitions'] = s['transitions']
        m['clear_events'] = s['clear_events']
        m['feedback_clear_events'] = s['feedback_clear_events']
        m['phases'] = {k: {**v, 'states': sorted(v['states'])} for k, v in s['phases'].items()}
        metrics[sid] = m
    def phase(sid, name, states):
        p = scenarios[sid]['phases'][name]
        c.require(p['states'] == set(states), sid + ': phase states ' + name)
        return p
    sid = 'continuous_switching_20min'
    c.require(sum(phase(sid, p, [2])['rows'] for p in ('forward', 'reverse', 'right', 'left', 'clockwise', 'counterclockwise')) == 1200000,
              '20 minutes simulated uninterrupted motion switching at 1 ms steps')
    for sid, up, drive in [('rc_loss_and_explicit_rearm', 'reconnected_up', 'drive_after_rearm'),
                           ('invalid_then_valid_same_tick', 'valid_up_recovery', 'drive_after_rearm')]:
        phase(sid, up, [0])
        phase(sid, drive, [2])
        c.require(metrics[sid]['arm_count'] == 2, sid + ': two explicit arms')
    s = scenarios['motor_203_feedback_loss']
    c.require(s['first_fault_ms'] - s['motor203_loss_last_feedback_ms'] == 31, 'motor feedback fault immediately after 30 ms allowance')
    for sid, fault in [('motor_203_feedback_loss', 3), ('can_unhealthy', 1), ('jam_and_current_limit', 8)]:
        c.require(metrics[sid]['final_state'] == 3 and metrics[sid]['final_fault'] == fault and metrics[sid]['arm_count'] == 1, sid + ': fault stays latched')
    c.require(metrics['can_unhealthy']['first_fault_ms'] == scenarios['can_unhealthy']['phases']['can_error']['first_ms'], 'CAN unhealthy immediate stop')
    c.require(metrics['jam_and_current_limit']['peak_current_raw'] == 1500 and metrics['jam_and_current_limit']['saturated_rows'] > 0, 'jam exercises current saturation')
    c.require(metrics['unused_left_vertical_bias']['arm_count'] == 1 and metrics['unused_left_vertical_bias']['peak_current_raw'] == 0, 'unused left V neither blocks arm nor drives')
    sid = 'can_transient_stuff_retry'
    a = phase(sid, 'stuff_pending_retry', [2])
    b = phase(sid, 'retry_success_1ms', [2])
    phase(sid, 'drive_after_retry', [2])
    c.require(b['first_ms'] - a['first_ms'] == 1 and metrics[sid]['first_fault_ms'] is None, '1 ms transient retry no interruption')
    sid = 'can_pending_timeout_explicit_recovery'
    p = phase(sid, 'pending_below_deadline', [2])
    phase(sid, 'pending_reaches_3ms', [3])
    c.require(metrics[sid]['first_fault_ms'] - p['first_ms'] == 3, 'exact 3 ms pending timeout')
    phase(sid, 'link_clean_but_up', [3])
    phase(sid, 'fresh_down_dwell', [0])
    phase(sid, 'down_dwell_complete', [1])
    phase(sid, 'drive_after_can_recovery', [2])
    events = metrics[sid]['clear_events']
    c.require(len(events) == 1 and metrics[sid]['arm_count'] == 2, 'one CAN clear and explicit second arm')
    if events:
        ready = next(x['at_ms'] for x in metrics[sid]['transitions'] if x['to'] == 1 and x['at_ms'] > events[0]['at_ms'])
        c.require(ready - events[0]['at_ms'] == 500, 'fresh 500 ms DOWN dwell after clear')
        metrics[sid]['rearm_ready_delay_ms'] = ready - events[0]['at_ms']
        metrics[sid]['pending_fault_latency_ms'] = metrics[sid]['first_fault_ms'] - p['first_ms']
    for sid, p in [('can_busoff_still_set_rejects_recovery', 'safe_down_but_busoff'),
                   ('can_nonrecoverable_flag_rejects_clear', 'clean_zeros_but_nonrecoverable')]:
        phase(sid, p, [3])
        c.require(metrics[sid]['policy_clears'] == metrics[sid]['controller_clears'] == 0, sid + ': recovery denied')
    if 'feedback_reconnect_requires_safe_down' in metrics:
        sid = 'feedback_reconnect_requires_safe_down'
        for p in ('fresh_feedback_but_up', 'fresh_feedback_but_mid', 'down_non_neutral',
                  'down_wheels_moving', 'moving_wait_rc_stale', 'down_rc_stale'):
            phase(sid, p, [3])
        phase(sid, 'feedback_fresh_down_dwell', [0])
        phase(sid, 'feedback_down_dwell_complete', [1])
        phase(sid, 'drive_after_feedback_recovery', [2])
        events = metrics[sid]['feedback_clear_events']
        c.require(len(events) == 1 and metrics[sid]['arm_count'] == 2, 'one feedback clear and explicit second arm')
        if events:
            ready = next(x['at_ms'] for x in metrics[sid]['transitions'] if x['to'] == 1 and x['at_ms'] > events[0]['at_ms'])
            c.require(ready - events[0]['at_ms'] == 500, 'fresh 500 ms DOWN after feedback clear')
            metrics[sid]['rearm_ready_delay_ms'] = ready - events[0]['at_ms']
        sid = 'feedback_first_then_can_recovery'
        phase(sid, 'feedback_can_recovered_but_up', [3])
        phase(sid, 'combined_fresh_down_dwell', [0])
        phase(sid, 'combined_down_dwell_complete', [1])
        phase(sid, 'drive_after_combined_recovery', [2])
        c.require(metrics[sid]['feedback_clears'] == 0 and len(metrics[sid]['clear_events']) == 1
                  and metrics[sid]['arm_count'] == 2, 'combined recovery must use production CAN policy')
        events = metrics[sid]['clear_events']
        if events:
            ready = next(x['at_ms'] for x in metrics[sid]['transitions'] if x['to'] == 1 and x['at_ms'] > events[0]['at_ms'])
            c.require(ready - events[0]['at_ms'] == 500, 'fresh 500 ms DOWN after combined clear')
            metrics[sid]['rearm_ready_delay_ms'] = ready - events[0]['at_ms']
        sid = 'isolated_bad_motor_frame_is_discarded'
        phase(sid, 'isolated_invalid_payload', [2])
        phase(sid, 'good_frames_after_bad_payload', [2])
        c.require(metrics[sid]['first_fault_ms'] is None and metrics[sid]['arm_count'] == 1,
                  'bad motor payload discarded without interruption')
    return {'schema_version': 1, 'reviewed_at_utc': datetime.now(timezone.utc).isoformat(),
        'reviewer': 'Codex independent CSV/file readback', 'job_id': identity[0], 'nonce': identity[1],
        'review_kind': 'Codex_preflight_not_WorkBuddy_execution' if preflight_identity else
                       'smoke_validator_selfcheck' if smoke else 'numerical_software_acceptance_only',
        'status': 'PASS' if c.failures == 0 else 'FAIL', 'accepted': False,
        'executor_receipt_and_UI_review': 'pending_root_review', 'software_only': True,
        'task_sha256': sha(task_path) if task_path else None, 'review_script_sha256': sha(__file__),
        'raw_sha256': sha(output / 'raw.csv'), 'raw_rows': rows_total,
        'checked_assertions': c.count, 'failed_assertions': c.failures, 'issues': c.issues,
        'checked_input_sha256': hashes, 'scenarios': metrics,
        'limitations': ['Assumed first-order rotor model; no physical direction or load validation.',
            'Injected CAN observations; no real bxCAN retry timing or BSP validation.',
            'CSV records post-tick state; private-state safe-OFF reconstructed from recorded RC/feedback.',
            'Temperature and forced-speed inputs are recorded; thermal/STOP upgrade coverage is outside this simulation suite.',
            'Twenty minutes is simulated time, not physical endurance or wall-clock test duration.',
            'Numerical acceptance does not identify who executed the command or accept screenshots/receipts.']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task', type=Path)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--report', type=Path)
    p.add_argument('--smoke', action='store_true', help='Only the fixed Codex prebuilt smoke identity; never WorkBuddy acceptance')
    p.add_argument('--preflight', action='store_true', help='Codex-only compiled preflight; requires explicit identity and no task')
    p.add_argument('--job-id')
    p.add_argument('--nonce')
    a = p.parse_args()
    if a.preflight:
        if a.task or a.smoke or not a.job_id or not a.nonce:
            p.error('Preflight requires --job-id/--nonce and excludes --task/--smoke')
    elif not a.task:
        p.error('Formal review requires --task')
    result = review(a.task, a.output_dir, a.smoke, (a.job_id, a.nonce) if a.preflight else None)
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if a.report:
        report = a.report.resolve()
        if report.is_relative_to(a.output_dir.resolve()) or 'input' in report.parts:
            p.error('Review report must be outside run/input; simulation artifacts are read-only')
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(text + '\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('job_id', 'nonce', 'review_kind', 'status', 'raw_rows', 'checked_assertions', 'failed_assertions', 'issues')}, ensure_ascii=False))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

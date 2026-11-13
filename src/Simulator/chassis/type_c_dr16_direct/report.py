"""Render actual direct-controller simulation results; no device/UI access."""
import argparse
import csv
import hashlib
import html
import json
from datetime import datetime, timezone
from pathlib import Path


def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--job-id', required=True)
    p.add_argument('--nonce', required=True)
    p.add_argument('--runner-exit', type=int, required=True)
    a = p.parse_args()
    out = a.output_dir.resolve()
    runner = json.loads((out/'runner_summary.json').read_text(encoding='utf-8-sig'))
    build = json.loads((out/'build_manifest.json').read_text(encoding='utf-8-sig'))
    issues = []
    for obj in (runner, build):
        if (obj['job_id'], obj['nonce']) != (a.job_id, a.nonce):
            issues.append('Identity mismatch')
    if a.runner_exit or not build['inputs_unchanged']:
        issues.append('Runner error or input changed')
    computed = {s['id']: {'rows': 0, 'active_rows': 0, 'peak_current_raw': 0,
                         'final_state': None, 'final_fault': None} for s in runner['scenarios']}
    rows = 0
    plot = []
    with (out/'raw.csv').open(newline='') as f:
        for r in csv.DictReader(f):
            rows += 1
            item = computed[r['scenario']]
            if (r['job_id'], r['nonce']) != (a.job_id, a.nonce) and 'CSV identity mismatch' not in issues:
                issues.append('CSV identity mismatch')
            item['rows'] += 1
            item['active_rows'] += r['state'] == '2'
            item['peak_current_raw'] = max(item['peak_current_raw'],
                max(abs(int(r[f'current_{mid:x}_raw'])) for mid in range(0x201, 0x209)))
            item['final_state'], item['final_fault'] = int(r['state']), int(r['fault'])
            if r['scenario'] == 'direct_continuous_20min' and int(r['sim_ms']) % 1000 == 0:
                plot.append((int(r['sim_ms']), int(r['target_201_millirpm'])/1000, float(r['model_201_rpm'])))
    for s in runner['scenarios']:
        if not s['passed'] or any(s[k] != v for k, v in computed[s['id']].items()):
            issues.append('Runner/CSV mismatch: '+s['id'])
    summary = {'schema_version': 1, 'job_id': a.job_id, 'nonce': a.nonce,
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'PASS' if not issues and runner['status'] == 'PASS' else 'FAIL',
        'software_only': True, 'direct_mode': True,
        'acceptance': 'awaiting_independent_Codex_file_readback',
        'scope': 'Unprotected direct controller C + actual DR16 parser + assumed first-order rotor; no BSP or real CAN/UART.',
        'behavior': {'S1_UP': 'immediate drive using last valid input; no dwell or neutral qualification',
                     'other_S1': 'zero command on receipt of that valid RC frame',
                     'remote_and_feedback': 'invalid RC zeros drive until next valid frame; no silence timeout; invalid feedback discarded',
                     'temperature_overspeed_stall_key_CAN_tick_gap': 'no software stop',
                     'current_encoding': 'C620 +/-16384 protocol endpoint, not a safety current limit'},
        'model': {'gain_rpm_per_raw': 0.8, 'tau_ms': 180, 'dt_ms': 1, 'can_command_hold_ms': 5,
                  'rc_frame_ms': 14, 'motor_feedback_ms': [1, 1, 8, 1, 1],
                  'continuous_motion_simulated_ms': 1200000},
        'limits': {'target_rotor_rpm': 300, 'C620_encoding_abs_max': 16384},
        'build': build, 'scenarios': runner['scenarios'],
        'csv_recomputation': {'rows': rows, 'scenarios': computed, 'issues': issues},
        'artifacts': {'raw_csv': 'raw.csv', 'raw_csv_sha256': digest(out/'raw.csv'),
                      'runner_summary': 'runner_summary.json',
                      'runner_summary_sha256': digest(out/'runner_summary.json'),
                      'report_html': 'report.html'}}
    h = html.escape
    table = ''.join(f"<tr><td>{h(s['id'])}</td><td>{'PASS' if s['passed'] else 'FAIL'}</td>"
                    f"<td>{s['rows']:,}</td><td>{s['peak_current_raw']}</td><td>{s['active_rows']:,}</td>"
                    f"<td>{h(s['first_failure'])}</td></tr>" for s in runner['scenarios'])
    lines = []
    for j, color in ((1, '#cc7400'), (2, '#176bc2')):
        points = ' '.join(f'{45+t/1200000*1000:.1f},{130-v/400*90:.1f}' for t, *values in plot for v in [values[j-1]])
        lines.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1"/>')
    page = f'''<!doctype html><html lang="en"><meta charset="utf-8"><title>DR16 direct-mode software simulation</title>
<style>body{{font:16px/1.5 system-ui;background:#f2f5f8;color:#172231;margin:30px}}main{{max-width:1200px;margin:auto}}section{{background:white;padding:24px;margin:18px 0;border-radius:10px}}table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{padding:8px;border-bottom:1px solid #ddd;text-align:left}}code{{overflow-wrap:anywhere}}.note{{border-left:5px solid #cc7400;padding-left:15px}}svg{{width:100%}}a{{color:#176bc2}}</style>
<main><h1>DR16 direct mode: software simulation</h1><p>Job <code>{h(a.job_id)}</code><br>Nonce <code>{h(a.nonce)}</code></p>
<section><h2>{summary['status']} · {len(runner['scenarios'])} scenarios · {rows:,} rows</h2>
<p>{sum(s['checks'] for s in runner['scenarios']):,} C assertions. Twenty minutes means <b>simulated time</b> at 1 ms steps, not physical endurance or wall-clock runtime.</p>
<p class="note"><b>This version has no software protection stops.</b> A valid S1 UP commands immediately. Other S1 positions command zero. Invalid RC zeros the drive until a new valid command. Silence alone has no timeout; invalid feedback is discarded. CAN-health/key inputs, high temperature/speed, a stalled model, and a clock gap do not stop the controller. +/-16384 is the C620 encoding range.</p></section>
<section><table><tr><th>Scenario</th><th>Result</th><th>Rows</th><th>Peak raw command</th><th>ACTIVE samples</th><th>Note</th></tr>{table}</table></section>
<section><h2>Twenty-minute command/response trace (1 s display sampling)</h2>
<p>Orange: 201 target; blue: assumed rotor speed. Full 1 ms rows are in raw.csv.</p>
<svg viewBox="0 0 1100 270" role="img" aria-label="Twenty minute simulated target and rotor speed"><line x1="45" y1="130" x2="1045" y2="130" stroke="#aaa"/>{''.join(lines)}<text x="45" y="260">0 min</text><text x="995" y="260">20 min</text></svg></section>
<section><h2>Evidence boundary</h2><p>Production parser/controller/wheel configuration are compiled into the host executable. Rotor gain 0.8 rpm/raw, time constant 180 ms, feedback 201/202/204/207 at 1 ms and 203 at 8 ms, RC at 14 ms, command hold 5 ms are assumptions. This does not simulate the BSP, actual CAN delivery/arbitration/bus-off, UART hardware, power, real motor load, or physical wheel direction. Controller compatibility inputs are injected; actual BSP behavior is outside this test.</p>
<p>Input hashes unchanged: {build['inputs_unchanged']}. <a href="raw.csv">Raw CSV</a> · <a href="summary.json">Summary</a> · <a href="build_manifest.json">Build manifest</a>. Independent file review remains required.</p></section></main></html>'''
    (out/'report.html').write_text(page, encoding='utf-8')
    summary['artifacts']['report_html_sha256'] = digest(out/'report.html')
    (out/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({'status': summary['status'], 'rows': rows, 'issues': issues}))
    return 0 if summary['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

"""Read simulation artifacts, independently check CSV invariants, render a report.

Standard library only. It never connects to hardware or launches other programs.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import html
import json
import math
from pathlib import Path


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def svg_plot(samples, columns, title, unit):
    if not samples:
        return ""
    width, height = 920, 225
    left, top, right, bottom = 54, 28, 18, 38
    plot_w, plot_h = width - left - right, height - top - bottom
    duration = max(float(row["sim_ms"]) for row in samples) or 1
    values = [float(row[key]) * scale for row in samples for key, _, _, scale in columns]
    magnitude = max(1, max(abs(v) for v in values)) * 1.08
    def xy(row, key, scale):
        x = left + float(row["sim_ms"]) / duration * plot_w
        y = top + plot_h / 2 - float(row[key]) * scale / magnitude * plot_h / 2
        return x, y
    parts = [f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">',
             f'<text x="{left}" y="16" class="plot-title">{html.escape(title)} · {html.escape(unit)}</text>']
    for fraction in (-1, 0, 1):
        y = top + plot_h / 2 - fraction * plot_h / 2
        parts.append(f'<line x1="{left}" y1="{y}" x2="{width-right}" y2="{y}" stroke="#e1e7ef"/>')
        parts.append(f'<text x="{left-7}" y="{y+4}" text-anchor="end">{fraction*magnitude:.0f}</text>')
    for n in range(5):
        x = left + n / 4 * plot_w
        parts.append(f'<text x="{x}" y="{height-20}" text-anchor="middle">{duration*n/4000:.1f}s</text>')
    for index, (key, label, color, scale) in enumerate(columns):
        path = " ".join(("M" if i == 0 else "L") + f"{xy(row,key,scale)[0]:.2f},{xy(row,key,scale)[1]:.2f}"
                        for i, row in enumerate(samples))
        parts.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="1.7"/>')
        parts.append(f'<text x="{left+index*205}" y="{height-2}" fill="{color}">{html.escape(label)}</text>')
    parts.append("</svg>")
    return "".join(parts)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--job-id", required=True)
    p.add_argument("--nonce", required=True)
    p.add_argument("--runner-exit", type=int, required=True)
    args = p.parse_args()
    folder = args.output_dir.resolve()
    runner = json.loads((folder / "runner_summary.json").read_text(encoding="utf-8-sig"))
    build = json.loads((folder / "build_manifest.json").read_text(encoding="utf-8-sig"))
    issues = []
    def issue(message):
        if message not in issues and len(issues) < 100:
            issues.append(message)
    for name, obj in (("runner", runner), ("build", build)):
        if obj.get("job_id") != args.job_id or obj.get("nonce") != args.nonce:
            issue(f"{name} identity mismatch")
    if not build.get("inputs_unchanged", False):
        issue("Input files changed or were not rechecked")
    if args.runner_exit != 0:
        issue(f"Runner exited {args.runner_exit}")
    limits = runner["limits"]
    known = {s["id"]: s for s in runner["scenarios"]}
    computed = {key: {"rows": 0, "peak_current_raw": 0, "active_rows": 0,
                      "first_fault_ms": None, "final_state": None,
                      "final_fault": None, "last_ms": -1} for key in known}
    samples = {key: [] for key in known}
    last_state = {}
    last_policy_clears = {}
    last_feedback_clears = {}
    feedback_clear_times = {}
    latest_remote = {}
    total_rows = 0
    with (folder / "raw.csv").open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            total_rows += 1
            name = row["scenario"]
            if name not in known:
                issue("Unexpected scenario in CSV")
                continue
            if row["job_id"] != args.job_id or row["nonce"] != args.nonce:
                issue("CSV identity mismatch")
            if row['rc_packet_sent'] == '1':
                latest_remote[name] = (int(row['switch_left']),
                    all(int(row[k]) == 0 for k in ('command_right_milli','command_forward_milli','command_cw_milli')))
            item = computed[name]
            t, state, fault = int(row["sim_ms"]), int(row["state"]), int(row["fault"])
            if t != item["last_ms"] + 1:
                issue(f"{name}: missing, reordered, or duplicate millisecond row")
            item["last_ms"] = t
            item["rows"] += 1
            item["active_rows"] += state == limits["active_state"]
            item["final_state"], item["final_fault"] = state, fault
            if state == 3 and item["first_fault_ms"] is None:
                item["first_fault_ms"] = t
            for motor in range(0x201, 0x209):
                command = int(row[f"command_{motor:x}_raw"])
                item["peak_current_raw"] = max(item["peak_current_raw"], abs(command))
                if abs(command) > limits["max_current_raw"]:
                    issue(f"{name}: current limit exceeded")
                if motor >= 0x205 and command:
                    issue(f"{name}: unused motor or feeder command nonzero")
                if state != limits["active_state"] and command:
                    issue(f"{name}: current commanded while disabled")
            for motor in range(0x201, 0x205):
                if abs(int(row[f"target_{motor:x}_millirpm"])) > limits["max_target_rotor_rpm"] * 1000:
                    issue(f"{name}: target speed exceeded")
                if not math.isfinite(float(row[f"model_{motor:x}_rpm"])):
                    issue(f"{name}: model produced nonfinite value")
            if row["policy_enabled"] == "1":
                if int(row["policy_fault_flags"]) and state == limits["active_state"]:
                    issue(f"{name}: active while transport fault remained latched")
                clears = int(row["policy_clear_count"])
                if clears > last_policy_clears.get(name, 0):
                    if state != 0 or row["policy_safe_off"] != "1":
                        issue(f"{name}: transport clear did not remain safely disabled")
                    if int(row["policy_clear_elapsed_ms"]) < 1000 or int(row["policy_clear_zero_tx_delta"]) < 20:
                        issue(f"{name}: recovery dwell or acknowledged-zero minimum was not met")
                    if int(row["policy_esr"]) != 0:
                        issue(f"{name}: transport cleared with ESR errors")
                last_policy_clears[name] = clears
            feedback_clears = int(row.get('feedback_clear_count', 0))
            if feedback_clears > last_feedback_clears.get(name, 0):
                if state != 0 or latest_remote.get(name) != (2, True) or row['can_healthy'] != '1' or int(row['policy_fault_flags']):
                    issue(f'{name}: feedback clear did not require safe DOWN and healthy CAN')
                if row['rc_valid'] != '1' or int(row['rc_age_ms']) >= 50 or int(row['feedback_temperature_c']) >= 70:
                    issue(f'{name}: feedback clear accepted stale RC or hot feedback')
                if any(abs(int(row[f'feedback_{m:x}_rpm'])) > 5 for m in range(0x201,0x205)):
                    issue(f'{name}: feedback clear while wheels moving')
                feedback_clear_times[name] = t
            if name in feedback_clear_times and t-feedback_clear_times[name] < 500 and state != 0:
                issue(f'{name}: feedback clear bypassed fresh DOWN dwell')
            last_feedback_clears[name] = feedback_clears
            if t % 100 == 0 or last_state.get(name) != state or row["invalid_injected"] == "1":
                samples[name].append(row)
            last_state[name] = state
    for name, observed in computed.items():
        expected = known[name]
        for key in ("rows", "peak_current_raw", "active_rows", "first_fault_ms", "final_state", "final_fault"):
            if observed[key] != expected[key]:
                issue(f"{name}: CSV/runner disagreement for {key}")
        if not expected["passed"]:
            issue(f'{name}: runner assertion failed: {expected["first_failure"]}')
    passed = not issues and runner.get("status") == "PASS"
    summary = {"schema_version": 1, "job_id": args.job_id, "nonce": args.nonce,
               "generated_at_utc": datetime.now(timezone.utc).isoformat(),
               "status": "PASS" if passed else "FAIL", "software_only": True,
               "acceptance": "awaiting_independent_Codex_file_readback",
               "scope": "Actual C parser/controller/CAN policy with an assumed first-order rotor and injected CAN status observations; no BSP, real retry timing, physical direction, or loaded motor performance validation.",
               "left_vertical_policy": runner["left_vertical_policy"],
               "pending_specifications": ["unused_left_vertical_arm_policy"] if runner["left_vertical_policy"] == "record-only" else [],
               "model": runner["model"], "limits": limits, "build": build,
               "scenarios": runner["scenarios"],
               "csv_recomputation": {"rows": total_rows, "scenarios": computed, "issues": issues},
               "artifacts": {"raw_csv": "raw.csv", "raw_csv_sha256": digest(folder / "raw.csv"),
                             "runner_summary": "runner_summary.json", "runner_summary_sha256": digest(folder / "runner_summary.json"),
                             "report_html": "report.html"}}
    h = html.escape
    rows = []
    for case in runner["scenarios"]:
        note = case["first_failure"] or "Expected fault injection" if case["final_fault"] else case["first_failure"] or ""
        if case["id"] == "unused_left_vertical_bias":
            note = f'Arm observed: {case["left_vertical_bias_armed"]}; policy: {runner["left_vertical_policy"]}'
        if case.get("policy_clears"):
            note = f'Qualified clear: {case["recovery_elapsed_ms"]} ms, {case["recovery_zero_tx_delta"]} zero TXOK; explicit rearm'
        if case.get('feedback_clears'):
            note = 'Feedback restored + safe DOWN: clear to DISABLED; new 500 ms DOWN, then explicit UP'
        rows.append(f'<tr><td>{h(case["id"])}</td><td class="{"pass" if case["passed"] else "fail"}">{"PASS" if case["passed"] else "FAIL"}</td>'
                    f'<td>{case["rows"]/1000:.3f}s</td><td>{case["peak_current_raw"]}</td><td>{case["arm_count"]}</td>'
                    f'<td>{case["final_state"]} / {case["final_fault"]}</td><td>{h(note)}</td></tr>')
    continuous = samples["continuous_switching_20min"]
    plots = svg_plot(continuous, [("target_201_millirpm", "201 target", "#d97706", .001),
                                 ("model_201_rpm", "201 assumed model", "#2563eb", 1)],
                     "20 minutes simulated time: repeated direction changes", "rotor rpm")
    plots += svg_plot(continuous, [(f"command_{motor:x}_raw", f"{motor:x} command", color, 1)
                                  for motor, color in zip(range(0x201, 0x205), ("#2563eb", "#d97706", "#059669", "#7c3aed"))],
                      "Four wheel current commands", "raw")
    plots += svg_plot(samples["rc_loss_and_explicit_rearm"], [("state", "state: 0 disabled, 1 ready, 2 active", "#2563eb", 1)],
                      "RC loss and explicit rearm", "state")
    plots += svg_plot(samples["can_pending_timeout_explicit_recovery"], [("state", "state: 0 off, 1 ready, 2 active, 3 fault", "#2563eb", 1),
                                                                       ("policy_clear_count", "qualified CAN clear count", "#059669", 1)],
                      "3 ms TX deadline and explicit qualified recovery", "state / count")
    if 'feedback_reconnect_requires_safe_down' in samples:
        plots += svg_plot(samples['feedback_reconnect_requires_safe_down'],
                          [('state','state: 0 off, 1 ready, 2 active, 3 fault','#2563eb',1),
                           ('feedback_clear_count','safe feedback clear count','#059669',1)],
                          'Feedback reconnect: UP cannot resume; safe DOWN and fresh dwell required','state / count')
    pending = '<p class="warn">Unused left vertical arm policy is recorded only and still awaits the task owner\'s decision.</p>' if summary["pending_specifications"] else ""
    errors = "<ul>" + "".join(f"<li>{h(x)}</li>" for x in issues) + "</ul>" if issues else ""
    text = f'''<!doctype html><html lang="en"><meta charset="utf-8"><title>DR16 software simulation</title>
<style>body{{font:15px/1.55 system-ui,sans-serif;color:#172033;background:#f3f6fa;margin:0;padding:30px}}main{{max-width:1120px;margin:auto}}h1{{font-size:28px;margin:0}}.muted{{color:#59677d}}.card{{background:white;border:1px solid #dce3ed;border-radius:12px;padding:22px;margin:18px 0}}.pass{{color:#08784a;font-weight:700}}.fail{{color:#b91c1c;font-weight:700}}.warn{{background:#fff6db;padding:12px;border-radius:7px}}table{{border-collapse:collapse;width:100%;font-size:12px}}td,th{{padding:9px 8px;border-bottom:1px solid #e5eaf1;text-align:left}}code{{overflow-wrap:anywhere}}svg{{width:100%;display:block;margin:18px 0;font:11px system-ui}}.plot-title{{font-weight:650;font-size:13px}}a{{color:#1d4ed8}}</style>
<main><h1>DR16 closed-loop software simulation</h1><p class="muted">Job <code>{h(args.job_id)}</code> · nonce <code>{h(args.nonce)}</code></p>
<div class="card"><strong class="{summary["status"].lower()}">{summary["status"]}</strong> · {total_rows:,} raw millisecond rows · {sum(s["checks"] for s in runner["scenarios"]):,} controller checks
<p>This runs the actual C parser, chassis controller and CAN policy every 1 ms for at least <b>20 minutes of simulated continuous motion</b> against an <b>assumed first-order rotor</b> and injected CAN status observations. Simulated time is not wall-clock duration or physical endurance testing. It is not a calibrated physical model, a hardware retry-timing test, or hardware validation. Expected fault scenarios pass when output stops and the specified recovery policy is obeyed.</p>{pending}{errors}</div>
<div class="card"><table><thead><tr><th>Scenario</th><th>Result</th><th>Duration</th><th>Peak command</th><th>Arms</th><th>Final state / fault</th><th>Note</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div>
<div class="card">{plots}</div>
<div class="card"><b>Assumptions and provenance</b><p>Model gain 0.8 rotor rpm per raw current unit; time constant 180 ms. Control/model step 1 ms; command hold 5 ms; DR16 frames 14 ms. Motor feedback periods: 201/202/204/207 = 1 ms, 203 = 8 ms.</p>
<p>Compiler: <code>{h(str(build.get("compiler_version", "unknown")))}</code>. Input hash verification: <b>{build.get("inputs_unchanged")}</b>. Physical joystick polarity, wheel installation, CAN/UART hardware, watchdog behavior and load performance are outside this report.</p>
<p><a href="raw.csv">Raw CSV</a> · <a href="summary.json">Summary JSON</a> · <a href="runner_summary.json">Runner results</a> · <a href="build_manifest.json">Input hashes</a></p>
<p class="muted">Independent Codex file readback is still required for task acceptance.</p></div></main></html>'''
    (folder / "report.html").write_text(text, encoding="utf-8")
    summary["artifacts"]["report_html_sha256"] = digest(folder / "report.html")
    (folder / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"status": summary["status"], "job_id": args.job_id, "nonce": args.nonce,
                      "rows": total_rows, "summary": str(folder / "summary.json"),
                      "report": str(folder / "report.html"), "issues": issues}, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Run the frozen, precompiled C simulation without invoking PowerShell scripts.

Only local simulation files are read/written; no hardware or UI access.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--job-id', required=True)
    parser.add_argument('--nonce', required=True)
    args = parser.parse_args()
    source = args.input_dir.resolve()
    output = args.output_dir.resolve()
    if not args.output_dir.is_absolute() or output == source or source in output.parents:
        raise ValueError('Output must be an absolute directory outside the input copy')
    provenance = json.loads((source / 'prebuilt_manifest.json').read_text(encoding='utf-8'))
    inputs = []
    for relative, expected in provenance['input_sha256'].items():
        path = (source / relative).resolve()
        if source not in path.parents or digest(path) != expected:
            raise ValueError('Frozen input mismatch: ' + relative)
        inputs.append({'path': str(path), 'sha256': expected})
    if output.exists() and any(output.iterdir()):
        raise ValueError('Refusing to reuse an existing output directory')
    output.mkdir(parents=True, exist_ok=True)
    manifest = {
        'schema_version': 1, 'job_id': args.job_id, 'nonce': args.nonce,
        'started_at': datetime.now().astimezone().isoformat(),
        'source_dir': str(source), 'simulation_dir': str(source / 'sim'),
        'output_dir': str(output), 'software_only': True,
        'compiler': provenance['compiler'],
        'compiler_version': provenance['compiler_version'],
        'compilation_scope': 'Frozen executable compiled by Codex; this run does not compile',
        'python': sys.executable, 'python_version': sys.version,
        'inputs': inputs, 'inputs_unchanged': False,
        'left_vertical_policy': 'ignore',
        'runner_sha256': digest(source / 'model_runner.exe'),
        'model_scope': 'Actual C parser/controller/CAN policy; assumed rotor and injected CAN status',
    }
    (output / 'compile.log').write_text(manifest['compilation_scope'] + '\n', encoding='utf-8')
    flags = {'creationflags': subprocess.CREATE_NO_WINDOW} if sys.platform == 'win32' else {}
    with (output / 'runner.log').open('wb') as log:
        run = subprocess.run(
            [str(source / 'model_runner.exe'), '--job-id', args.job_id,
             '--nonce', args.nonce, '--left-vertical-policy', 'ignore'],
            cwd=output, stdout=log, stderr=subprocess.STDOUT, timeout=120, **flags)
    manifest['runner_exit'] = run.returncode
    manifest['inputs_unchanged'] = all(digest(Path(x['path'])) == x['sha256'] for x in inputs)
    manifest['finished_at'] = datetime.now().astimezone().isoformat()
    (output / 'build_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    with (output / 'report.log').open('wb') as log:
        report = subprocess.run(
            [sys.executable, '-B', str(source / 'sim/report.py'), '--output-dir', str(output),
             '--job-id', args.job_id, '--nonce', args.nonce, '--runner-exit', str(run.returncode)],
            cwd=output, stdout=log, stderr=subprocess.STDOUT, timeout=120, **flags)
    print(json.dumps({'runner_exit': run.returncode, 'report_exit': report.returncode,
                      'inputs_unchanged': manifest['inputs_unchanged'],
                      'summary': str(output / 'summary.json')}, ensure_ascii=True))
    return 0 if run.returncode == report.returncode == 0 and manifest['inputs_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

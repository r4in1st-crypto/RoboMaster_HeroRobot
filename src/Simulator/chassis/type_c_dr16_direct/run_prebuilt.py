"""Run frozen direct-mode C tests. --compile-from is Codex preflight only."""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sources = p.add_mutually_exclusive_group(required=True)
    sources.add_argument('--input-dir', type=Path)
    sources.add_argument('--compile-from', type=Path)
    p.add_argument('--compiler', type=Path)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--job-id', required=True)
    p.add_argument('--nonce', required=True)
    a = p.parse_args()
    source = (a.input_dir or a.compile_from).resolve()
    sim = source/'sim' if a.input_dir else Path(__file__).resolve().parent
    output = a.output_dir.resolve()
    if not a.output_dir.is_absolute() or output == source or (a.input_dir and source in output.parents):
        p.error('Output must be an absolute dedicated directory, outside frozen input')
    if output.exists() and any(output.iterdir()):
        p.error('Refusing to reuse existing output')
    if a.compile_from and not a.compiler:
        found = shutil.which('zig')
        if not found:
            p.error('Install Zig 0.13.0 on PATH or pass --compiler FILE')
        a.compiler = Path(found)
    if a.input_dir:
        provenance = json.loads((source/'prebuilt_manifest.json').read_text(encoding='utf-8-sig'))
        expected = provenance['input_sha256']
        for rel, value in expected.items():
            path = (source/rel).resolve()
            if source not in path.parents or digest(path) != value:
                raise ValueError('Frozen input mismatch: '+rel)
        compiler = provenance['compiler']
        compiler_version = provenance['compiler_version']
    else:
        firmware_files = ['dr16.c', 'dr16.h', 'chassis_control.c', 'chassis_control.h', 'wheel_config.h']
        simulation_files = ['model_runner.c', 'report.py', 'run_prebuilt.py', 'independent_review.py']
        paths = [source/name for name in firmware_files] + [sim/name for name in simulation_files]
        expected = {path: digest(path) for path in paths}
        compiler = str(a.compiler.resolve())
        compiler_version = 'Zig '+subprocess.check_output([compiler,'version'],text=True).strip()+' / Clang C99 -O2'
    output.mkdir(parents=True, exist_ok=True)
    manifest = {'schema_version':1, 'job_id':a.job_id, 'nonce':a.nonce,
        'started_at':datetime.now().astimezone().isoformat(),
        'software_only':True, 'direct_mode':True, 'source_dir':str(source),
        'simulation_dir':str(sim), 'output_dir':str(output),
        'compiler':compiler, 'compiler_version':compiler_version, 'python':sys.executable,
        'compilation_scope':'Codex compiled current sources' if a.compile_from else 'Frozen executable compiled by Codex; this run does not compile',
        'inputs':[{'path':str(source/rel),'sha256':h} for rel,h in expected.items()],
        'inputs_unchanged':False}
    options = {'creationflags':subprocess.CREATE_NO_WINDOW} if sys.platform=='win32' else {}
    exe = output/'model_runner.exe' if a.compile_from else source/'model_runner.exe'
    if a.compile_from:
        env = dict(os.environ, ZIG_LOCAL_CACHE_DIR=str(output/'zig-local-cache'),
                   ZIG_GLOBAL_CACHE_DIR=str(output/'zig-global-cache'))
        cmd = [compiler,'cc','-std=c99','-Wall','-Wextra','-Werror','-O2','-I',str(source),
               str(source/'dr16.c'),str(source/'chassis_control.c'),
               str(sim/'model_runner.c'),'-o',str(exe)]
        with (output/'compile.log').open('wb') as log:
            subprocess.run(cmd,cwd=output,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120,**options)
    else:
        (output/'compile.log').write_text(manifest['compilation_scope']+'\n',encoding='utf-8')
    manifest['runner_sha256'] = digest(exe)
    with (output/'runner.log').open('wb') as log:
        started = datetime.now().astimezone()
        run = subprocess.run([str(exe),'--job-id',a.job_id,'--nonce',a.nonce],
                             cwd=output,stdout=log,stderr=subprocess.STDOUT,timeout=120,**options)
        manifest['runner_elapsed_seconds'] = (datetime.now().astimezone()-started).total_seconds()
    manifest['runner_exit'] = run.returncode
    manifest['inputs_unchanged'] = all(digest(source/r)==h for r,h in expected.items())
    manifest['finished_at'] = datetime.now().astimezone().isoformat()
    (output/'build_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    with (output/'report.log').open('wb') as log:
        started = datetime.now().astimezone()
        report = subprocess.run([sys.executable,'-B',str(sim/'report.py'),'--output-dir',str(output),
            '--job-id',a.job_id,'--nonce',a.nonce,'--runner-exit',str(run.returncode)],
            cwd=output,stdout=log,stderr=subprocess.STDOUT,timeout=120,**options)
        report_seconds = (datetime.now().astimezone()-started).total_seconds()
    print(json.dumps({'runner_exit':run.returncode,'report_exit':report.returncode,
        'runner_elapsed_seconds':manifest['runner_elapsed_seconds'],'report_elapsed_seconds':report_seconds,
        'inputs_unchanged':manifest['inputs_unchanged'],'summary':str(output/'summary.json')}))
    return 0 if run.returncode==report.returncode==0 and manifest['inputs_unchanged'] else 1


if __name__=='__main__':
    raise SystemExit(main())

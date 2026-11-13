"""Independent direct-mode CSV/provenance review. No controller imports or hardware."""
import argparse
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

SCENARIOS = {
    'direct_continuous_20min': 1200000,
    'up_immediate_other_switch_positions_zero': 720,
    'rc_gap_and_invalid_keep_last_valid': 2500,
    'feedback_gap_and_invalid_keep_last_valid': 2800,
    'temperature_and_overspeed_do_not_stop': 800,
    'can_key_and_tick_gap_do_not_stop': 1400,
    'stalled_model_does_not_shutdown': 3000,
}


def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def review(output, task=None, preflight=None):
    output = Path(output).resolve()
    runner, summary, build = (read(output/f) for f in ('runner_summary.json','summary.json','build_manifest.json'))
    identity = tuple(preflight) if preflight else (task['job_id'],task['nonce'])
    checks, failures, issues = 0, 0, []
    def require(ok, message):
        nonlocal checks, failures
        checks += 1
        if not ok:
            failures += 1
            if len(issues)<60: issues.append(message)
    for name, obj in (('runner',runner),('summary',summary),('build',build)):
        require((obj['job_id'],obj['nonce'])==identity, name+' identity')
        require(obj['software_only'] and obj['direct_mode'], name+' direct/software scope')
    require(summary['build']==build and summary['scenarios']==runner['scenarios'], 'embedded summaries')
    require(summary['status']==runner['status']=='PASS' and build['runner_exit']==0 and build['inputs_unchanged'], 'run status')
    require(Path(build['output_dir']).resolve()==output, 'output path')
    require(list(SCENARIOS)==[s['id'] for s in runner['scenarios']], 'seven ordered direct scenarios')
    if task:
        require(output==Path(task['output_dir']).resolve()/'run', 'task output')
        require(task['scenario_ids']==list(SCENARIOS), 'task scenarios')
        for group in ('source_sha256','working_copy_sha256'):
            for path,h in task['input'][group].items():
                require(digest(path)==h, group+': '+path)
        frozen = task['input']['working_copy_sha256']
        require({Path(x['path']).resolve() for x in build['inputs']}==
            {Path(p).resolve() for p in frozen if Path(p).name!='prebuilt_manifest.json'}, 'complete frozen input list')
        for item in build['inputs']:
            require(item['sha256']==frozen.get(item['path']), 'build/frozen hash '+item['path'])
        source = Path(build['source_dir'])
        provenance = read(source/'prebuilt_manifest.json')
        require(build['runner_sha256']==provenance['runner_sha256']==digest(source/'model_runner.exe'), 'frozen executable binding')
        for rel,h in provenance['input_sha256'].items():
            require(digest(source/rel)==h, 'prebuilt input '+rel)
    else:
        for item in build['inputs']:
            require(digest(item['path'])==item['sha256'], 'preflight input '+item['path'])
        require(digest(output/'model_runner.exe')==build['runner_sha256'], 'preflight executable')
    for key,filename in (('raw_csv','raw.csv'),('runner_summary','runner_summary.json'),('report_html','report.html')):
        require(summary['artifacts'][key]==filename and summary['artifacts'][key+'_sha256']==digest(output/filename), 'artifact '+filename)
    require(all(x in (output/'report.html').read_text(encoding='utf-8') for x in identity), 'HTML identity')
    data = {}
    with (output/'raw.csv').open(newline='') as f:
        for raw in csv.DictReader(f):
            r = {k:(v if k in ('job_id','nonce','scenario','phase') else float(v) if k.startswith('model_') else int(v)) for k,v in raw.items()}
            name,t = r['scenario'],r['sim_ms']
            if name not in data:
                data[name]={'rows':0,'active_rows':0,'peak_current_raw':0,'final_state':None,'final_fault':None,
                    'phase_rows':{},'phase_states':{},'last_rc':None,'last_rc_ms':None,'rc_frames':0,'rc_bad':0,
                    'counts':[0]*4,'last_fb_ms':[0]*4,'speeds':[0]*4,'max_rc_age_ms':0,
                    'max_feedback_age_ms':0,'min_online_wheels':4,'previous':None}
            s=data[name]; context=f'{name}@{t}: '
            require((r['job_id'],r['nonce'])==identity, context+'identity')
            require(t==s['rows'], context+'consecutive model millisecond')
            if r['rc_result']==0:
                s['last_rc']=(r['switch_left'],r['input_right'],r['input_forward'],r['input_cw'])
                s['last_rc_ms']=r['controller_ms'];s['rc_frames']+=1
            elif r['rc_result']>0:
                s['rc_bad']+=1
            require(r['rc_frames']==s['rc_frames'] and r['rc_bad']==s['rc_bad'], context+'valid/invalid RC counters')
            age=(r['controller_ms']-s['last_rc_ms'])&0xffffffff if s['last_rc_ms'] is not None else 0xffffffff
            require(r['rc_age_ms']==age, context+'last valid RC timestamp retained')
            expected_state=2 if s['last_rc'] and s['last_rc'][0]==1 else 0
            require(r['state']==expected_state and r['fault']==0, context+'direct switch only; no protection state')
            require(r['state'] not in (1,3), context+'READY and FAULT absent')
            currents=[r[f'current_{i:x}_raw'] for i in range(0x201,0x209)]
            require(max(map(abs,currents))<=16384, context+'protocol encoding bound')
            require(all(x==0 for x in currents[4:]), context+'slots 5..8 zero')
            require(expected_state==2 or not any(currents), context+'valid non-UP frame zeroes immediately')
            if expected_state==2:
                _,right,forward,cw=s['last_rc'];right/=1000;forward/=1000;yaw=cw/1000*0.65
                mix=[-forward-right+yaw,-forward+right+yaw,forward+right+yaw,forward-right+yaw]
                scale=max(1,max(map(abs,mix)))
                targets=[x/scale*300000 for x in mix]
            else: targets=[0]*4
            online=0
            for j,mid in enumerate(range(0x201,0x205)):
                require(abs(r[f'target_{mid:x}_millirpm']-targets[j])<=2, context+f'{mid:x} direct mixer/held RC')
                received=bool(r['feedback_mask']&(1<<j)) and (j!=2 or t%8==0) and not r['bad_feedback']
                s['counts'][j]+=int(received)
                if received:
                    s['last_fb_ms'][j]=r['controller_ms'];s['speeds'][j]=r[f'feedback_{mid:x}_rpm']
                require(r[f'count_{mid:x}']==s['counts'][j] and r[f'last_{mid:x}_ms']==s['last_fb_ms'][j],
                        context+f'{mid:x} valid feedback only')
                require(r[f'feedback_{mid:x}_rpm']==s['speeds'][j], context+f'{mid:x} missing/invalid preserves speed')
                fb_age=(r['controller_ms']-s['last_fb_ms'][j])&0xffffffff
                online+=s['counts'][j]>0 and fb_age<=30
                s['max_feedback_age_ms']=max(s['max_feedback_age_ms'],fb_age)
                require(math.isfinite(r[f'model_{mid:x}_rpm']), context+'finite assumed model')
            require(r['online_wheels']==online, context+'freshness diagnostic only')
            s['rows']+=1;s['active_rows']+=r['state']==2
            s['peak_current_raw']=max(s['peak_current_raw'],max(map(abs,currents)))
            s['final_state'],s['final_fault']=r['state'],r['fault']
            s['max_rc_age_ms']=max(s['max_rc_age_ms'],age)
            s['min_online_wheels']=min(s['min_online_wheels'],online)
            s['phase_rows'][r['phase']]=s['phase_rows'].get(r['phase'],0)+1
            s['phase_states'].setdefault(r['phase'],set()).add(r['state'])
            s['previous']=r
    require(list(data)==list(SCENARIOS), 'complete CSV scenarios')
    require(sum(s['rows'] for s in data.values())==1211220==summary['csv_recomputation']['rows'], 'exact total rows')
    for result in runner['scenarios']:
        s=data[result['id']]
        for k in ('rows','active_rows','peak_current_raw','final_state','final_fault'):
            require(s[k]==result[k]==summary['csv_recomputation']['scenarios'][result['id']][k], result['id']+' metric '+k)
        require(result['passed'] and not result['failures'] and s['rows']==SCENARIOS[result['id']], result['id']+' pass/coverage')
    require(data['direct_continuous_20min']['active_rows']==1200000, '20 simulated minutes ACTIVE at every step')
    require(data['rc_gap_and_invalid_keep_last_valid']['max_rc_age_ms']>1500, 'RC loss exceeds previous timeout while driving')
    require(data['feedback_gap_and_invalid_keep_last_valid']['max_feedback_age_ms']>2000 and
            data['feedback_gap_and_invalid_keep_last_valid']['min_online_wheels']==0, 'all-feedback loss exercised without a stop')
    for name in ('temperature_and_overspeed_do_not_stop','can_key_and_tick_gap_do_not_stop','stalled_model_does_not_shutdown'):
        require(data[name]['active_rows']==data[name]['rows'], name+' never disabled')
    require(data['temperature_and_overspeed_do_not_stop']['peak_current_raw']==16384, 'high feedback exercises full protocol endpoint')
    require(1500<data['stalled_model_does_not_shutdown']['peak_current_raw']<=16384,
            'stalled PI has no former 1500 cap; conditional integration may settle below endpoint')
    for s in data.values():
        for k in ('last_rc','last_rc_ms','rc_frames','rc_bad','counts','last_fb_ms','speeds','previous'):
            s.pop(k,None)
        s['phase_states']={k:sorted(v) for k,v in s['phase_states'].items()}
    return {'schema_version':1,'job_id':identity[0],'nonce':identity[1],
        'reviewed_at_utc':datetime.now(timezone.utc).isoformat(),
        'review_kind':'Codex_preflight_not_WorkBuddy_execution' if preflight else 'independent_numeric_review',
        'status':'PASS' if not failures else 'FAIL','accepted':False,'software_only':True,'direct_mode':True,
        'review_script_sha256':digest(__file__),'task_sha256':None if preflight else digest(task['_path']),
        'raw_sha256':digest(output/'raw.csv'),'raw_rows':sum(s['rows'] for s in data.values()),
        'checked_assertions':checks,'failed_assertions':failures,'issues':issues,'scenarios':data,
        'limitations':['No hardware/BSP or physical CAN delivery is tested.',
            'This verifies intentional absence of software protection stops, not machine safety.',
            '20 minutes is simulated time; the rotor model is an uncalibrated assumption.',
            'Executor/UI provenance requires root review; this numeric report does not accept the task.']}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--preflight',action='store_true')
    p.add_argument('--job-id');p.add_argument('--nonce')
    a=p.parse_args()
    if a.preflight:
        if a.task or not a.job_id or not a.nonce:p.error('Preflight requires explicit identity and no task')
        task=None
    else:
        if not a.task:p.error('Formal review requires --task')
        task=read(a.task);task['_path']=str(a.task)
    if a.report.resolve().is_relative_to(a.output_dir.resolve()):
        p.error('Review report must be outside simulation output')
    result=review(a.output_dir,task,(a.job_id,a.nonce) if a.preflight else None)
    a.report.parent.mkdir(parents=True,exist_ok=True)
    a.report.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('job_id','nonce','status','raw_rows','checked_assertions','failed_assertions','issues')}))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())

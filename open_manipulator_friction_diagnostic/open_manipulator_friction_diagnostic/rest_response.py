"""Extract genuine commanded-sine/current/encoder trials; never use hand data."""
import json
from pathlib import Path
import shutil
import numpy as np
from .automatic_calibration import assess_rest_trace, file_hash, select_rest, write_new

ENCODER_STEP_RAD = .000011984224905356572


def extract(folder, response_profile=None, *, allow_neutral_response=False):
    folder=Path(folder)
    e=json.loads((folder/'execution.json').read_text())
    neutral=e.get('neutral_current_response',False)
    if neutral and not allow_neutral_response:
        raise ValueError('Current-reporting diagnostic is not dither-selection evidence')
    reviewed = (e.get('target_joint') in (3,4,5) and not neutral or
                e.get('purpose')=='rest_selection' or any(k in e for k in
                    ('rest_selection_plan','rest_selection_geometry_sha256','rest_selection_parameters')))
    if (e.get('kind')!='rest_response' or not e.get('finished') or not e.get('passed') or
            e['target_joint'] not in (range(1,7) if neutral or reviewed else (1,2,6)) or e['probe']['fault']!=0 or
            (not e['mock'] and e.get('register_restored') is not True)):
        raise ValueError('Need a completed, restored, fault-free rest-response acquisition')
    if (folder/'human_contact_review.json').exists():
        raise ValueError('Contacted experiment is not identification evidence')
    data=np.genfromtxt(folder/'samples.csv',delimiter=',',names=True)
    if reviewed:
        from .rest_selection import validate_acquisition
        validate_acquisition(e,folder,data)
    traces=[];results=[];joint=e['target_joint']
    for i, block in enumerate(e['sine_blocks']):
        rows=data[data['phase']==i]
        if len(rows)<100:raise ValueError('Missing sine block')
        t=rows['time_s']-rows['time_s'][0]
        trace=dict(**block, trace_id=file_hash(folder/'samples.csv')[:16]+f'/block{i}',
            source='synthetic' if e['mock'] else 'hardware', joint=joint,identity=e['identity'],
            contact_free=True,contact_assumption='no reported contact during known-input program',
            input_kind='known_sine',condition=e['condition'],ramp_s=e['sine_ramp_s'],
            t=t.tolist(),q=rows[f'j{joint}_q'].tolist(),command_nm=rows['command_nm'].tolist(),
            current_proxy_nm=rows[f'j{joint}_effort'].tolist())
        result=assess_rest_trace(trace,ENCODER_STEP_RAD,response_profile)
        traces.append(trace);results.append(dict(block=i,**block,**result))
    sources = ('samples.csv','execution.json','geometry_review.json') if reviewed else ('samples.csv','execution.json')
    return traces,dict(source_files_sha256={n:file_hash(folder/n) for n in sources},
        condition=e['condition'],blocks=results,physical_breakaway_identified=False,
        hardware_deployable=False)


def _selection(runs, joint, identity, conditions, mock, response_profile=None, selection_policy=None):
    traces=[]
    for run in runs:
        values,_=extract(run,response_profile)
        if any(t['joint']!=joint or t['identity']!=identity for t in values):
            raise ValueError('Rest joint/robot/baseline identity mismatch')
        if not mock and any(t['source']!='hardware' for t in values):
            raise ValueError('synthetic rest records require explicit mock mode')
        traces.extend(values)
    result=select_rest(traces,ENCODER_STEP_RAD,conditions,response_profile,selection_policy)
    result['missing_conditions']=sorted(set(conditions)-{t['condition'] for t in traces})
    return result


def _decision(selection):
    chosen=selection['selected']
    result = dict(joint=selection['joint'],identity=selection['identity'],source=selection['source'],
        stage=selection['stage'],required_conditions=selection['required_conditions'],
        trace_content_sha256=selection['trace_content_sha256'],
        selected={k:chosen[k] for k in ('amplitude_nm','frequency_hz')} if chosen else None)
    if 'response_profile' in selection:
        result['response_profile'] = selection['response_profile']
    if 'selection_policy' in selection:
        result['selection_policy'] = selection['selection_policy']
    return result


def archive(runs, output, joint, identity, conditions, mock=False, response_profile=None,
            selection_policy=None):
    """Copy source records and reselect the waveform; incomplete evidence stays incomplete."""
    runs=list(map(Path,runs));output=Path(output)
    selection=_selection(runs,joint,identity,conditions,mock,response_profile,selection_policy)
    output.mkdir(parents=True,exist_ok=False)
    files={}
    for i,run in enumerate(runs):
        target=output/f'run{i}';target.mkdir()
        execution=json.loads((run/'execution.json').read_text())
        names=['samples.csv','execution.json','register_restore.json']
        if execution.get('purpose')=='rest_selection':names.append('geometry_review.json')
        for name in names:
            if name=='register_restore.json' and not (run/name).exists():continue
            shutil.copyfile(run/name,target/name)
            files[f'run{i}/{name}']=file_hash(target/name)
    write_new(output/'selection.json',selection)
    files['selection.json']=file_hash(output/'selection.json')
    meta = dict(schema='rest_response_archive_v1',joint=joint,
        identity=identity,conditions=sorted(set(conditions)),run_count=len(runs),
        files_sha256=files,decision=_decision(selection),hardware_deployable=False)
    if response_profile is not None:
        meta['response_profile'] = response_profile
    if selection_policy is not None:
        meta['selection_policy'] = selection_policy
    write_new(output/'manifest.json',meta)
    load_archive(output,mock)
    return selection


def load_archive(folder, mock=False):
    folder=Path(folder);meta=json.loads((folder/'manifest.json').read_text())
    if meta.get('schema')!='rest_response_archive_v1' or not 1<=meta['run_count']<=100:
        raise ValueError('Invalid rest archive')
    required={'selection.json'}|{f'run{i}/{n}' for i in range(meta['run_count'])
                                for n in ('execution.json','samples.csv')}
    for i in range(meta['run_count']):
        execution=json.loads((folder/f'run{i}/execution.json').read_text())
        if execution.get('purpose')=='rest_selection':required.add(f'run{i}/geometry_review.json')
    optional={f'run{i}/register_restore.json' for i in range(meta['run_count'])}
    if not required<=set(meta['files_sha256'])<=required|optional:
        raise ValueError('Incomplete rest archive')
    for name,digest in meta['files_sha256'].items():
        if file_hash(folder/name)!=digest:raise ValueError('Changed rest evidence: '+name)
    runs=[folder/f'run{i}' for i in range(meta['run_count'])]
    result=_selection(runs,meta['joint'],meta['identity'],meta['conditions'],mock,
                      meta.get('response_profile'),meta.get('selection_policy'))
    if _decision(result)!=meta['decision']:
        raise ValueError('Rest selection is not reproducible from archived records')
    return result

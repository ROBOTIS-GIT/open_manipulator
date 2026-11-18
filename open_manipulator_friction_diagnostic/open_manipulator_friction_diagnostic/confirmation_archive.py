"""Reconstruct a prospective configuration check from immutable raw records."""
from datetime import datetime
import json
from pathlib import Path
import numpy as np

from .automatic_calibration import content_hash,file_hash
from . import fresh_configuration_confirmation as method


def _read(path):
    return json.loads(Path(path).read_text())


def load_confirmation(path,model,fraction=.95):
    """No hardware access, fitting, or trust in a saved `passed` boolean."""
    from .automatic_motion import ingest
    path=Path(path);proof=_read(path)
    if (proof.get('schema')!='prospective_configuration_confirmation_v1'
            or proof.get('automatic_deployment_allowed') is not False
            or proof.get('hardware_qualified') is not False):
        raise ValueError('Unknown prospective confirmation archive')
    contract_path=Path(proof['contract_path']);manifest_path=Path(proof['manifest_path'])
    physical=Path(proof['physical_path']);plan=manifest_path.parent
    bindings={contract_path:proof['contract_sha256'],manifest_path:proof['manifest_sha256'],
        physical/'status.json':proof['physical_status_sha256'],
        physical/'session/analysis/evidence.json':proof['analysis_evidence_sha256'],
        physical/'session/analysis/cycles.json':proof['analysis_cycles_sha256'],
        plan/'inputs/baseline.yaml':proof['baseline_sha256'],
        plan/'inputs/frozen_candidate.json':proof['model_file_sha256']}
    if any(file_hash(p)!=digest for p,digest in bindings.items()):
        raise ValueError('Changed prospective confirmation evidence')
    contract=_read(contract_path);manifest=_read(manifest_path);status=_read(physical/'status.json')
    if (contract.get('schema')!='frozen_configuration_confirmation_contract_v1'
            or contract.get('candidate_file_sha256')!=proof['model_file_sha256']
            or contract.get('plan_manifest_sha256')!=proof['manifest_sha256']
            or contract.get('review_source_sha256')!=file_hash(method.__file__)
            or contract.get('minimum_new_configurations')!=3
            or contract.get('repeats_per_configuration_speed')!=2
            or contract.get('mean_error_policy')!=dict(absolute_nm=.08,relative=.20,
                all_directional_cycle_means_required=True)
            or contract.get('old_configuration_cv_failure_retained') is not True
            or contract.get('automatic_deployment_allowed') is not False
            or contract.get('hardware_qualified') is not False):
        raise ValueError('Changed or relaxed prospective confirmation contract')
    if (status.get('completed') is not True or status.get('all_off') is not True
            or status.get('failure') or status.get('model_refitted') is not False
            or datetime.fromisoformat(contract['created_utc'])>=datetime.fromisoformat(status['started_utc'])
            or _read(plan/'inputs/frozen_candidate.json')!=model):
        raise ValueError('Incomplete acquisition or model not frozen before collection')
    for relative,digest in manifest['artifacts'].items():
        p=Path(relative)
        if p.is_absolute() or '..' in p.parts or file_hash(plan/p)!=digest:
            raise ValueError('Changed frozen acquisition artifact')
    analysis=physical/'session/analysis';evidence=_read(analysis/'evidence.json')
    cycles=_read(analysis/'cycles.json');raws=list((physical/'session/runs').glob('*/collection'))
    if len(raws)!=1:raise ValueError('Ambiguous prospective raw acquisition')
    raw=raws[0]
    for relative,digest in evidence['source_files_sha256'].items():
        p=Path(relative)
        if p.is_absolute() or '..' in p.parts or file_hash(raw/p)!=digest:
            raise ValueError('Changed prospective raw source')
    if file_hash(raw/'plan.json')!=manifest['artifacts']['inputs/fresh_validation/plan.json']:
        raise ValueError('Physical acquisition used a different plan')
    canonical=ingest(raw,plan/'inputs/baseline.yaml')
    if (canonical['evidence']!=evidence or not canonical['reference_check']['passed']
            or not _read(analysis/'reference_check.json')['passed']
            or len(cycles)!=len(canonical['cycles'])
            or len(cycles)!=contract['expected_cycles']):
        raise ValueError('Incomplete or unstable canonical acquisition')
    derived={'features','signed_gravity_ros_nm','directional_bias_ros_nm',
             'positive_resistance_proxy_nm','negative_resistance_proxy_nm'}
    for saved,again in zip(cycles,canonical['cycles']):
        if saved.keys()!=again.keys():raise ValueError('Canonical cycle schema differs')
        for key in saved:
            if saved[key]==again[key]:continue
            if key not in derived:raise ValueError('Canonical raw cycle differs')
            x,y=np.asarray(saved[key]),np.asarray(again[key])
            if x.shape!=y.shape or not np.allclose(x,y,atol=2e-14,rtol=0):
                raise ValueError('Canonical derived cycle differs beyond numeric roundoff')
    conditions=[c for c in _read(plan/'inputs/fresh_validation/conditions.json')
                if c['role']=='fresh_final_validation']
    result=method.review_confirmation(model,cycles,evidence,conditions,contract['speeds_rad_s'],
        _read(plan/'inputs/previous_cycles.json'),fraction=fraction)
    return dict(result,archive_sha256=file_hash(path),candidate_content_sha256=content_hash(model),
                canonical_raw_reextraction_matched=True)

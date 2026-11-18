"""Replace measured axes while preserving an already observed bank exactly.

This is an explicitly supervised candidate. Replacement axes require their
own frozen four-release evidence, including the exact dither and fraction.
"""
from copy import deepcopy
import json
from pathlib import Path

from .automatic_calibration import file_hash
from .kinetic_bank import ARRAYS, bank_fractions, compose, retune_completed_bank, verify_prior_controller


def replace_joints(previous, replacements):
    old_fractions = bank_fractions(previous)
    if not replacements:
        raise ValueError('No replacement axes')
    incoming = compose(replacements)
    baseline = lambda p: {k:v for k,v in p.items() if not k.startswith('stateful_')}
    if (baseline(incoming) != baseline(previous)
            or incoming['stateful_observer_window'] != previous['stateful_observer_window']):
        raise ValueError('Replacement changes gravity baseline or shared observer')
    result = deepcopy(previous)
    for i, enabled in enumerate(incoming['stateful_joint_mask']):
        if not enabled:
            continue
        if f'joint{i+1}' not in old_fractions:
            raise ValueError('Replacement cannot silently enable an untested axis')
        prefix = f'stateful_bank.joint{i+1}.'
        for key in list(result):
            if key.startswith(prefix):
                del result[key]
        result.update({k:deepcopy(v) for k,v in incoming.items() if k.startswith(prefix)})
        for name in ARRAYS:
            result['stateful_'+name][i] = incoming['stateful_'+name][i]
    return result


def load_update(bank, urdf, baseline, requested_fractions, *, _ancestors=()):
    from .coast_handoff import load_parameters
    from .kinetic_bank import load_bank
    update = bank['replacement']
    if (not isinstance(requested_fractions,dict)
            or update['requested_fractions'] != requested_fractions):
        raise ValueError('Specify every actual joint fraction from the replacement manifest')
    prior, hand = Path(update['prior_bank']), Path(update['prior_hand'])
    if file_hash(prior/'candidate.json') != update['prior_bank_sha256']:
        raise ValueError('Previous bank changed')
    prior_manifest = json.loads((prior/'candidate.json').read_text())
    if (prior_manifest.get('schema') not in
            ('supervised_kinetic_bank_v1','supervised_kinetic_bank_fraction_retest_v1')
            or prior_manifest.get('identity') != bank['identity']):
        raise ValueError('Expected an original or fraction-retest bank; replacement chains need a new consolidated archive')
    names = {'execution.json', *[f'restore_j{j}.json' for j in range(1,7)]}
    if set(update['hand_files_sha256']) != names:
        raise ValueError('Incomplete previous hand restoration evidence')
    for name, digest in update['hand_files_sha256'].items():
        if file_hash(hand/name) != digest:
            raise ValueError('Previous hand evidence changed: '+name)
    report = json.loads((hand/'execution.json').read_text())
    if report.get('candidate_sha256') != update['prior_bank_sha256']:
        raise ValueError('Previous hand used another bank')
    verify_prior_controller(report, Path(__file__).resolve().parents[2])
    for j in range(1,7):
        restored = json.loads((hand/f'restore_j{j}.json').read_text())
        if not (restored.get('target_joint') == j and restored.get('passed')
                and restored.get('torque_enable_verified_off')
                and restored.get('torque_enabled') is False):
            raise ValueError('Previous six-axis OFF restoration is incomplete')
    previous_request = report.get('kinetic_fraction_request', report.get('kinetic_fraction'))
    previous = load_bank(prior, urdf, baseline, previous_request, _ancestors=_ancestors)
    # Validate the complete recorded output and all fault/recording checks. An
    # identity retune changes no fractions and grants no new qualification.
    previous = retune_completed_bank(previous, report, bank_fractions(previous))
    retest = update.get('unreplaced_fraction_retest', {})
    if retest:
        old = bank_fractions(previous)
        replaced = {f'joint{r["joint"]}' for r in update['axes']}
        if (not isinstance(retest, dict) or not set(retest) <= set(old)-replaced or
                any(not isinstance(v,dict) or set(v) != {'from','to'} or
                    v['from'] != old[j] or v['to'] != requested_fractions.get(j)
                    for j,v in retest.items())):
            raise ValueError('Explicit unreplaced fraction retest differs from prior/requested values')
        # Same checked physical report as a standalone fraction retest; only
        # declared gains change, never model coefficients, support or dither.
        previous = retune_completed_bank(previous, report, {j:v['to'] for j,v in retest.items()})
    replacements = []
    for row in update['axes']:
        path = Path(row['path'])
        model = json.loads((path/'candidate.json').read_text())
        joint = row['joint']
        if (row.get('kind') != 'supervised_coast_series' or model['joint'] != joint
                or model['identity'] != bank['identity']
                or file_hash(path/'candidate.json') != row['candidate_sha256']
                or file_hash(path/'coast_series.json') != row['series_sha256']
                or f'joint{joint}' not in requested_fractions):
            raise ValueError('Changed replacement joint/model/release evidence')
        from .kinetic_support import preserve_support
        matches = [r for r in prior_manifest['components'] if r['joint'] == joint]
        if len(matches) != 1:
            raise ValueError('Ambiguous original joint model')
        old_path = Path(matches[0]['path'])
        if not old_path.is_absolute():
            old_path = prior/old_path
        if file_hash(old_path/'candidate.json') != matches[0]['candidate_sha256']:
            raise ValueError('Original joint model changed')
        preserve_support(json.loads((old_path/'candidate.json').read_text()), model)
        replacements.append(load_parameters(path, urdf, baseline, requested_fractions[f'joint{joint}']))
    result = replace_joints(previous, replacements)
    if bank_fractions(result) != requested_fractions:
        raise ValueError('Replacement request also changes an unreplaced axis')
    return result

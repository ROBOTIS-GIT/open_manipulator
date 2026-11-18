"""Bind transformed kinetic targets to the acquisitions already used to fit them."""
from .automatic_calibration import content_hash, payload_hash


def cycle_lineage(cycle):
    source=cycle.get('source_provenance',{})
    ids=source.get('cycle_ids',[]);hashes=source.get('payload_hashes',[])
    if (not isinstance(ids,list) or not all(isinstance(v,str) and v for v in ids)
            or not isinstance(hashes,list) or not all(isinstance(v,str) and len(v)==64 for v in hashes)):
        raise ValueError('Invalid source cycle provenance')
    return dict(cycle_ids=list(dict.fromkeys([cycle['cycle_id']]+ids)),
                payload_hashes=list(dict.fromkeys([payload_hash(cycle)]+hashes)),
                complete=complete_cycle_lineage(cycle))


def model_lineage(model):
    ids=list(model.get('development_cycle_ids',[]))
    # Older candidates already recorded development IDs in their fit/CV reviews.
    ids.extend(row['cycle_id'] for row in model.get('training_review',{}).get('rows',[]) if 'cycle_id' in row)
    for fold in model.get('selection',{}).get('folds',[]):
        ids.extend(fold.get('training_cycle_ids',[]));ids.extend(fold.get('held_cycle_ids',[]))
    return dict(cycle_ids=list(dict.fromkeys(ids)),
                payload_hashes=list(model.get('development_payload_hashes',[])))


def complete_cycle_lineage(cycle):
    source=cycle.get('source_provenance',{})
    return source.get('complete',True) is True and (not cycle.get('observation_hypothesis') or bool(
        source.get('cycle_ids') and source.get('payload_hashes')))


def development_provenance(cycles, previous=None):
    lineage=[cycle_lineage(c) for c in cycles]
    if previous is not None:lineage.append(model_lineage(previous))
    complete=all(complete_cycle_lineage(c) for c in cycles)
    if previous is not None:
        complete &= previous.get('development_lineage_complete') is not False
        if previous.get('observation_hypothesis'):
            complete &= previous.get('development_lineage_complete') is True
    return dict(development_sha256=content_hash(cycles),
        development_cycle_ids=list(dict.fromkeys(v for x in lineage for v in x['cycle_ids'])),
        development_payload_hashes=list(dict.fromkeys(v for x in lineage for v in x['payload_hashes'])),
        development_lineage_complete=bool(complete))


def validation_independent(model, cycles):
    development=model_lineage(model);validation=[cycle_lineage(c) for c in cycles]
    ids=set(development['cycle_ids']);hashes=set(development['payload_hashes'])
    if (content_hash(cycles)==model.get('development_sha256') or
            any(ids.intersection(c['cycle_ids']) or hashes.intersection(c['payload_hashes'])
                for c in validation)):
        raise ValueError('Development data cannot serve as fresh validation')
    valid_hashes=bool(hashes) and all(isinstance(h,str) and len(h)==64 for h in hashes)
    # A legacy transformed fit without source bindings cannot prove independence.
    complete=(model.get('development_lineage_complete') is not False and
        (not model.get('observation_hypothesis') or model.get('development_lineage_complete') is True))
    return valid_hashes and complete and all(complete_cycle_lineage(c) for c in cycles)

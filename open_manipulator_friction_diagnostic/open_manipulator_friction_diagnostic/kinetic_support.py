"""Measured model support and explicit replacement checks, never motion limits.

An overall speed/load box is not evidence that all combinations were measured.
Keep the observed load intervals at each speed when newer archives provide them.
These checks preserve data coverage; they do not certify prediction or stability.
"""
import math
from bisect import bisect_left


def _interval(values):
    if len(values)!=2 or not all(math.isfinite(x) for x in values) or values[0]>values[1]:
        raise ValueError('Invalid measured support interval')
    return values


def measured_support(cycles, joint):
    if not cycles or any(c['joint']!=joint for c in cycles):
        raise ValueError('Support requires one joint of measured cycles')
    rows=[]
    for speed in sorted({c['speed'] for c in cycles}):
        if not math.isfinite(speed) or speed<=0:raise ValueError('Invalid measured speed')
        cs=[c for c in cycles if c['speed']==speed]
        intervals=[_interval([min(c['signed_gravity_ros_nm']),max(c['signed_gravity_ros_nm'])]) for c in cs]
        rows.append(dict(speed_rad_s=speed,gravity_intervals_nm=intervals,
                         cycle_ids=[c['cycle_id'] for c in cs]))
    return dict(schema='measured_speed_load_support_v1',joint=joint+1,by_speed=rows,
                full_speed_load_rectangle_measured=False,independent_validation=False)


def speed_load_coverage(support, velocities, loads):
    """Require the load at both adjacent measured speed knots, preserving gaps.

    This describes interpolation support only. It neither clips compensation
    nor establishes accuracy in another posture or after a different history.
    """
    rows=support['by_speed'];speeds=[r['speed_rad_s'] for r in rows]
    if (not speeds or any(not math.isfinite(v) or v<=0 for v in speeds)
            or any(b<=a for a,b in zip(speeds,speeds[1:]))
            or len(velocities)!=len(loads)
            or not all(math.isfinite(x) for x in [*velocities,*loads])):
        raise ValueError('Invalid joint speed/load coverage inputs')
    intervals=[[_interval(x) for x in r['gravity_intervals_nm']] for r in rows]
    measured=[]
    for velocity,load in zip(velocities,loads):
        speed=abs(velocity)
        if speed<speeds[0]-1e-12 or speed>speeds[-1]+1e-12:
            measured.append(False);continue
        high=min(bisect_left(speeds,speed),len(speeds)-1)
        low=high if math.isclose(speed,speeds[high],rel_tol=1e-10,abs_tol=1e-12) else max(0,high-1)
        measured.append(all(any(a-1e-6<=load<=b+1e-6 for a,b in intervals[k]) for k in (low,high)))
    return dict(measured=measured,measured_fraction=sum(measured)/len(measured) if measured else None,
                motion_restricted=False,posture_history_validated=False,
                criterion='load present at both adjacent speed knots; disconnected intervals preserved')


def _bounds(model):
    speeds=model['speed_knots_rad_s'];speed=_interval(model['speed_range_rad_s'])
    load=_interval(model['gravity_range_nm'])
    if (len(speeds)<2 or not all(math.isfinite(v) and v>0 for v in speeds)
            or any(b<=a for a,b in zip(speeds,speeds[1:])) or speed!=[speeds[0],speeds[-1]]):
        raise ValueError('Measured speed metadata differs from runtime knots')
    return speed,load


def _covers(intervals, interval):
    lo,hi=_interval(interval);end=lo
    for a,b in sorted(_interval(row) for row in intervals):
        if b<end-1e-9:continue
        if a>end+1e-9:return False
        end=max(end,b)
        if end>=hi-1e-9:return True
    return False


def preserve_support(previous, candidate):
    """Reject a replacement that silently drops previously recorded support."""
    if previous['joint']!=candidate['joint']:raise ValueError('Different joint models')
    old=_bounds(previous);new=_bounds(candidate)
    for name,a,b in zip(('speed','gravity load'),old,new):
        if b[0]>a[0]+1e-9 or b[1]<a[1]-1e-9:
            raise ValueError(f'Replacement shrinks measured {name} support: {a} -> {b}')
    detailed=previous.get('measurement_support')
    if detailed:
        updated=candidate.get('measurement_support')
        if not updated:raise ValueError('New model is missing previous speed/load support metadata')
        for row in detailed['by_speed']:
            intervals=[x for r in updated['by_speed'] if abs(r['speed_rad_s']-row['speed_rad_s'])<1e-12
                       for x in r['gravity_intervals_nm']]
            if any(not _covers(intervals,x) for x in row['gravity_intervals_nm']):
                raise ValueError('Replacement shrinks measured speed/load combinations')
    return dict(preserved=True,joint=candidate['joint'],previous_speed_rad_s=old[0],
        candidate_speed_rad_s=new[0],previous_gravity_nm=old[1],candidate_gravity_nm=new[1],
        per_speed_intervals_checked=bool(detailed),full_speed_load_rectangle_measured=False,
        hardware_qualified=False)

"""Compare paired records on identical measured angles, without fitting a model.

Different spans can have different means even for the same repeatable spatial
pattern. This diagnostic preserves both full-span and common-span statistics;
it never replaces full-span validation with a favourable central window.
"""
import numpy as np
from .fit import spatial_weights


def compare_spatial_support(rows):
    groups = {}
    for row in rows:
        q = np.asarray(row['position_rad'], float)
        fields = ['friction_samples_nm']
        if 'positive_effort_samples_nm' in row:
            fields += ['positive_effort_samples_nm', 'negative_effort_samples_nm']
        if (q.ndim != 1 or len(q) < 2 or not np.isfinite(q).all() or
                np.any(np.diff(q) <= 0)):
            raise ValueError('Strictly increasing finite measured angles required')
        values = {k: np.asarray(row[k], float) for k in fields}
        if any(v.shape != q.shape or not np.isfinite(v).all() for v in values.values()):
            raise ValueError('Incomplete spatial current records')
        speed = float(row['speed'])
        if not np.isfinite(speed) or speed <= 0 or row['joint'] not in range(6):
            raise ValueError('Invalid joint/speed')
        groups.setdefault((row['joint'], round(speed, 12)), []).append((row, q, values))
    results = []
    for (joint, speed), group in sorted(groups.items()):
        low, high = max(x[1][0] for x in group), min(x[1][-1] for x in group)
        cycles = []
        for row, q, values in group:
            weights = spatial_weights(q)
            matched = None
            if high > low:
                grid = np.unique(np.r_[low, q[(q > low) & (q < high)], high])
                w = spatial_weights(grid)
                matched = {k: float(w @ np.interp(grid, q, v)) for k, v in values.items()}
            cycles.append(dict(source=row.get('source_folder'), pose=row.get('validation_pose', ''),
                repeat=row['repeat'], full_interval_rad=[float(q[0]), float(q[-1])],
                full_means_nm={k: float(weights @ v) for k, v in values.items()},
                common_means_nm=matched,
                retained_spatial_fraction=float(max(0., high-low)/(q[-1]-q[0]))))
        results.append(dict(joint=joint+1, speed_rad_s=speed,
            common_interval_rad=[float(low), float(high)] if high > low else None,
            common_support_exists=bool(high > low), cycles=cycles))
    return dict(schema='measured_spatial_support_review_v1', groups=results,
        diagnostic_only=True, full_span_validation_replaced=False,
        independent_output_torque=False, hardware_deployable=False)

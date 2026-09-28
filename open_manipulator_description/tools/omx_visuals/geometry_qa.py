import os
import hashlib,json
from pathlib import Path
import numpy as np
import trimesh
from inspect_alignment import source_parts

root=Path(os.environ['OMX_BUILD_DIR'])
rows=[]
for kind in ['leader','follower']:
    parts,meshes=source_parts(kind)
    for pid, group in meshes.items():
        for m in group:
            if m.visual.material.name not in ['Printed charcoal | provisional finish','Moulded motor housing']:
                continue
            key=hashlib.sha256(m.vertices.tobytes()+m.faces.tobytes()).hexdigest()[:24]
            cache=np.load(root/'decimation'/f'{key}_lod.npz')
            reduced=trimesh.Trimesh(cache['vertices'],cache['faces'],process=True)
            # Bidirectional sampled surface distance, not a certified Hausdorff bound.
            a=trimesh.sample.sample_surface(m,256,seed=2026)[0]
            b=trimesh.sample.sample_surface(reduced,256,seed=2026)[0]
            distances=np.r_[trimesh.proximity.closest_point(reduced,a)[1],trimesh.proximity.closest_point(m,b)[1]]*1000
            row=dict(model=kind,part=pid,name=parts[pid]['sourceName'],triangles=len(reduced.faces),
                     sampled_max_mm=float(distances.max()),sampled_p95_mm=float(np.percentile(distances,95)))
            rows.append(row)
    print(kind,'sampled parts',sum(r['model']==kind for r in rows),flush=True)
(root/'geometry_qa.json').write_text(json.dumps(rows,indent=2))
print('max',max(r['sampled_max_mm'] for r in rows),'mm',flush=True)

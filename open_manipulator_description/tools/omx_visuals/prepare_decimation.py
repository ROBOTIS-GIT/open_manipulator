import os
import hashlib, json
import numpy as np
from inspect_alignment import source_parts
from pathlib import Path

out=Path(os.environ['OMX_BUILD_DIR'])/'decimation'
out.mkdir(exist_ok=True)
jobs=[]
for kind in ['leader','follower']:
    metadata, parts=source_parts(kind)
    for pid,meshes in parts.items():
        for m in meshes:
            key=hashlib.sha256(m.vertices.tobytes()+m.faces.tobytes()).hexdigest()[:24]
            name=metadata[pid]['sourceName']
            ratio=.12
            if 'CASE_M_DUMMY' in name or 'CASE_B_DUMMY' in name:ratio=.35
            if name.startswith('PR33_C02_F1_'):ratio=.4
            if '_BASE' in name:ratio=.25
            np.savez(out/f'{key}.npz',vertices=m.vertices,faces=m.faces)
            jobs.append(dict(key=key,target=max(48,round(len(m.faces)*ratio))))
(out/'jobs.json').write_text(json.dumps(jobs))
print('meshes',len(jobs),flush=True)

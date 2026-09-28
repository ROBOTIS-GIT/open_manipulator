import os
import json,re,subprocess
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import xml.etree.ElementTree as ET

ROOT=Path(os.environ['OMX_REPO_ROOT'])
REPO=ROOT
SOURCE=Path(os.environ.get('OMX_MODEL_DIR', '.'))
BASE_REVISION=os.environ.get('OMX_BASE_REVISION', 'd31000d90c679af9c982e73de8b12d777c5ff7dd')
PERM=np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1.]])

def source_parts(kind):
    data=json.loads((SOURCE/f'{kind}.json').read_text())
    parts={p['id']:p for p in data['parts'] if p['group']<1000}
    scene=trimesh.load(SOURCE/f'{kind}.glb',process=False)
    result={i:[] for i in parts}
    for node in scene.graph.nodes_geometry:
        match=re.match(r'part_(\d+)',node)
        if not match or int(match[1]) not in parts: continue
        i=int(match[1]); transform,geometry=scene.graph[node]
        m=scene.geometry[geometry].copy();m.apply_transform(PERM@transform)
        result[i].append(m)
    missing=[i for i,v in result.items() if not v]
    assert all(kind=="follower" and parts[i]["group"]==92 for i in missing), missing
    for i in missing: del parts[i]; del result[i]
    return parts,result

def baseline(kind):
    short='omx_'+kind[0]
    xml=ET.ElementTree(ET.fromstring(subprocess.check_output(['git','-C',str(REPO),'show',f'{BASE_REVISION}:open_manipulator_description/urdf/{short}/{short}_arm.urdf.xacro'])))
    links=xml.findall('.//link');pos={links[0].attrib['name']:np.zeros(3)}
    for j in xml.findall('.//joint'):
        o=j.find('origin');assert o.attrib.get('rpy','0 0 0')=='0 0 0'
        pos[j.find('child').attrib['link']]=pos[j.find('parent').attrib['link']]+np.fromstring(o.attrib['xyz'],sep=' ')
    meshes=[];paths=[];origins=[]
    for l in links:
        fname=Path(l.find('visual/geometry/mesh').attrib['filename']).name
        p=REPO/f'open_manipulator_description/meshes/{short}'/fname
        m=trimesh.load(p,process=False);m.apply_scale(.001)
        origin=pos[l.attrib['name']];m.apply_translation(origin)
        meshes.append(m);paths.append(p);origins.append(origin)
    return meshes,paths,origins

def semantic(kind,group):
    if kind=='leader':
        for i,end in enumerate([11,21,40,59,73,83,92]):
            if group<=end:return i
    else:
        if group>=92:return 3 if group<=96 else 5
        if group in [79,80,81,90,91]:return 7
        if group==78 or 82<=group<=89:return 6
        for i,end in enumerate([19,29,41,53,67,77]):
            if group<=end:return i
    raise ValueError(group)

if __name__=='__main__':
    rows=[]
    for kind in ['leader','follower']:
        parts,meshes=source_parts(kind);old,paths,origins=baseline(kind)
        trees=[]
        for m in old:
            pts=np.vstack([m.vertices,m.triangles_center])
            trees.append(cKDTree(pts))
        for i,p in parts.items():
            pts=np.vstack([m.vertices for m in meshes[i]])[::max(1,sum(len(m.vertices) for m in meshes[i])//300)]
            costs=[float(np.median(t.query(pts)[0])*1000) for t in trees]
            assigned=semantic(kind,p['group']);best=int(np.argmin(costs))
            row=dict(kind=kind,id=i,group=p['group'],name=p['sourceName'],semantic=assigned,nearest=best,nearest_mm=round(costs[best],3),assigned_mm=round(costs[assigned],3))
            rows.append(row)
            if assigned!=best and costs[assigned]>2:print(row)
    Path(__file__).with_name('alignment.json').write_text(json.dumps(rows,indent=2))

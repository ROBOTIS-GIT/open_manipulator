import os
import bpy, json, sys
import numpy as np
from pathlib import Path

root=Path(sys.argv[sys.argv.index('--')+1])
jobs=json.loads((root/'jobs.json').read_text())
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
for i,job in enumerate(jobs):
    key=job['key'];data=np.load(root/f'{key}.npz')
    mesh=bpy.data.meshes.new(key)
    mesh.from_pydata(data['vertices'].tolist(),[],data['faces'].tolist());mesh.update()
    obj=bpy.data.objects.new(key,mesh);bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active=obj;obj.select_set(True)
    # Merge coincident imported split-normal vertices before QEM.
    bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=1e-7)
    bpy.ops.object.mode_set(mode='OBJECT')
    if len(mesh.polygons)>job['target']:
        mod=obj.modifiers.new('Simulation LOD','DECIMATE')
        mod.ratio=min(1,job['target']/len(mesh.polygons));mod.use_collapse_triangulate=True
        bpy.ops.object.modifier_apply(modifier=mod.name)
    mesh=obj.data;mesh.calc_loop_triangles()
    vertices=np.empty((len(mesh.vertices),3),np.float64);mesh.vertices.foreach_get('co',vertices.ravel())
    faces=np.array([t.vertices[:] for t in mesh.loop_triangles],np.int64)
    np.savez(root/f'{key}_lod.npz',vertices=vertices,faces=faces)
    bpy.data.objects.remove(obj,do_unlink=True)
    bpy.data.meshes.remove(mesh)
    if i%50==0:print('DECIMATED',i+1,'/',len(jobs),flush=True)
print('DECIMATION_COMPLETE',flush=True)

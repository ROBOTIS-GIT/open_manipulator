"""Regression checks, import smoke tests and actual MuJoCo render evidence."""
import os

import hashlib
import json
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np
from lxml import etree as ET
from PIL import Image
import xacro
import trimesh

from build_assets import PKG, REPO, OUT, old_text

def expand(kind, prefix=''):
    short = 'omx_' + kind[0]
    wrapper = OUT / f'{kind}_{prefix}wrapper.xacro'
    wrapper.write_text(f'''<robot name="{short}" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="{PKG}/urdf/{short}/{short}_arm.urdf.xacro"/>
    <xacro:{short} prefix="{prefix}"/></robot>''')
    xml = xacro.process_file(str(wrapper)).toxml()
    root = ET.fromstring(xml.encode())
    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename').replace('package://open_manipulator_description/', str(PKG) + '/')
        assert Path(filename).is_file(), filename
        mesh.set('filename', filename)
    for tex in root.findall('.//texture'):
        filename = tex.get('filename').replace('package://open_manipulator_description/', str(PKG) + '/')
        assert Path(filename).is_file(), filename
        tex.set('filename', filename)
    mj = ET.SubElement(root, 'mujoco')
    ET.SubElement(mj, 'compiler', discardvisual='false', strippath='false', fusestatic='false')
    path = OUT / f'{kind}_{prefix}expanded.urdf'
    path.write_bytes(ET.tostring(root, pretty_print=True))
    return path

def canonical_without_visuals(root):
    for link in root.findall('.//link'):
        for collision in link.findall('collision'):
            collision.attrib.pop('name', None)
        for visual in list(link.findall('visual')):
            link.remove(visual)
    return ET.tostring(root, method='c14n')

def regression(kind):
    short = 'omx_' + kind[0]
    rel = f'open_manipulator_description/urdf/{short}/{short}_arm.urdf.xacro'
    parser = ET.XMLParser(remove_blank_text=True)
    before = ET.fromstring(old_text(rel).encode(), parser)
    after = ET.parse(str(REPO / rel), parser).getroot()
    assert canonical_without_visuals(before) == canonical_without_visuals(after), 'Nonvisual URDF change'
    static_rel = f'open_manipulator_description/urdf/{short}/{short}.urdf'
    static_before = ET.fromstring(old_text(static_rel).encode(), parser)
    static_after = ET.parse(str(REPO / static_rel), parser).getroot()
    for link in static_after.findall('link'):
        if not link.get('name', '').startswith('link'):
            continue
        for mesh in link.findall('visual/geometry/mesh'):
            assert mesh.get('filename').endswith('.obj'), 'Stale standalone URDF visual'
            path = mesh.get('filename').replace('package://open_manipulator_description/', str(PKG) + '/')
            assert Path(path).is_file(), path
    assert canonical_without_visuals(static_before) == canonical_without_visuals(static_after), 'Nonvisual standalone URDF change'
    manifest = json.loads((PKG / f'meshes/{short}/visual_manifest.json').read_text())
    assert all(p['group'] < 1000 for p in manifest['parts'])
    for r in manifest['visuals']:
        m = manifest['materials'][r['material']]
        if 'texture' in m:
            loaded = trimesh.load(PKG / f'meshes/{short}' / r['file'], force='mesh', process=False)
            assert loaded.visual.material.image is not None, r['file']
            assert loaded.visual.uv is not None, r['file']
    for row in manifest['collision']:
        path = PKG / f'meshes/{short}' / row['file']
        original = subprocess.check_output(['git', '-C', str(REPO), 'show', f'{os.environ["OMX_BASE_REVISION"]}:{path.relative_to(REPO)}'])
        assert original == path.read_bytes(), path
    return dict(nonvisual_urdf_unchanged=True, collision_files_unchanged=True,
                cable_parts=0, visual_triangles=sum(r['triangles'] for r in manifest['visuals']),
                visual_obj_bytes=sum(r['bytes'] for r in manifest['visuals']))

def render(model, kind):
    model.vis.global_.offwidth = 1280
    model.vis.global_.offheight = 960
    model.vis.headlight.ambient[:] = .35
    model.vis.headlight.diffuse[:] = .65
    data = mujoco.MjData(model)
    opt = mujoco.MjvOption()
    opt.geomgroup[3] = 0
    if kind == 'leader': opt.geomgroup[0] = 0
    with mujoco.Renderer(model, height=960, width=1280) as renderer:
        for pose, values in [('zero', None), ('bent', [.3, -.5, .75, -.25, .5, .3, -.3])]:
            data.qpos[:] = 0
            if values is not None:
                data.qpos[:] = values[:model.nq]
            mujoco.mj_forward(model, data)
            assert np.isfinite(data.xpos).all()
            for angle, azimuth in [('front', 130), ('rear', -50)]:
                cam = mujoco.MjvCamera()
                cam.lookat[:] = [.1, 0, .11]
                cam.distance = .58
                cam.azimuth = azimuth
                cam.elevation = -25
                renderer.update_scene(data, camera=cam, scene_option=opt)
                Image.fromarray(renderer.render()).save(OUT / f'{kind}_{pose}_{angle}.png')
    return dict(nq=model.nq, bodies=model.nbody, geoms=model.ngeom, meshes=model.nmesh,
                finite_forward_poses=2, rendered_views=4)

def compare_mjcf(model):
    # Compile baseline with absolute mesh paths in an output-only temporary file.
    rel = 'open_manipulator_description/mujoco/omx/omx.xml'
    root = ET.fromstring(old_text(rel).encode())
    root.find('compiler').set('meshdir', str(PKG))
    temp = OUT / 'baseline_follower.xml'
    temp.write_bytes(ET.tostring(root))
    original = mujoco.MjModel.from_xml_path(str(temp))
    for attr in ['body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_ipos', 'body_iquat',
                 'jnt_pos', 'jnt_axis', 'jnt_range', 'dof_damping', 'dof_armature',
                 'actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange', 'eq_data']:
        np.testing.assert_allclose(getattr(original, attr), getattr(model, attr), atol=1e-12, rtol=0,
                                   err_msg=attr)
    before = np.where(original.geom_group == 3)[0]
    after = np.where(model.geom_group == 3)[0]
    for attr in ['geom_pos', 'geom_quat', 'geom_friction', 'geom_contype', 'geom_conaffinity']:
        np.testing.assert_allclose(getattr(original, attr)[before], getattr(model, attr)[after], atol=1e-12, rtol=0)
    a, b = mujoco.MjData(original), mujoco.MjData(model)
    for _ in range(250):
        mujoco.mj_step(original, a)
        mujoco.mj_step(model, b)
        np.testing.assert_allclose(a.qpos, b.qpos, atol=1e-9, rtol=0)
    assert np.isfinite(b.qpos).all()
    return True

if __name__ == '__main__':
    report = dict(mujoco=mujoco.__version__, models={})
    for kind in ['leader', 'follower']:
        row = regression(kind)
        for prefix in ['', 'robot_']:
            path = expand(kind, prefix)
            start = time.perf_counter()
            model = mujoco.MjModel.from_xml_path(str(path))
            row[f'urdf_{prefix or "unprefixed"}_load_seconds'] = round(time.perf_counter()-start, 3)
        if kind == 'follower':
            model = mujoco.MjModel.from_xml_path(str(PKG / 'mujoco/omx/omx.xml'))
            row['native_mjcf_physics_unchanged'] = compare_mjcf(model)
        else:
            model = mujoco.MjModel.from_xml_path(str(PKG / 'mujoco/omx_l/omx_l.xml'))
            row['native_mjcf_loaded'] = True
        for j in range(model.nq):
            d = mujoco.MjData(model)
            for angle in [-.3, .3]:
                d.qpos[:] = 0
                d.qpos[j] = angle
                mujoco.mj_forward(model, d)
                assert np.isfinite(d.xpos).all()
        row['individual_joint_forward_poses'] = model.nq * 2
        row.update(render(model, kind))
        report['models'][kind] = row
        print(kind, row, flush=True)
    (OUT / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')

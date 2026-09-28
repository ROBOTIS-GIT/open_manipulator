"""Create cable-free simulation visuals from the locally supplied OMX assemblies.

Collision meshes and all kinematic/dynamic parameters stay byte-for-byte unchanged.
Never downloads or publishes source CAD. See the generated manifest for provenance.
"""
import os

import collections
import hashlib
import json
import re
import subprocess
from pathlib import Path

import numpy as np
import trimesh
from lxml import etree as ET

from inspect_alignment import ROOT, REPO, SOURCE, source_parts, baseline, semantic

PKG = REPO / 'open_manipulator_description'
OUT = Path(os.environ['OMX_BUILD_DIR'])
LABELS = {
    'Printed charcoal | provisional finish': 'printed_charcoal',
    'MLCC | tan ceramic': 'ceramic',
    'Header and IC | black polymer': 'black_electronics',
    'USB shell and solder | nickel tin': 'nickel_tin',
    'Contacts | gold plating': 'gold_contacts',
    'JST | ivory moulded polymer': 'ivory_connectors',
    'Unpowered LED | pale translucent package': 'unpowered_led',
    'Power terminal | green polymer': 'green_terminal',
    'OpenRB web | top': 'openrb_top',
    'OpenRB web | bottom': 'openrb_bottom',
    'OpenRB | blue solder mask': 'blue_pcb',
    'Steel hardware': 'steel_hardware',
    'Moulded motor housing': 'motor_housing',
    'DYNAMIXEL | LED cover | milky white diffuser': 'white_led_diffuser',
    'Motor fasteners | black oxide steel': 'black_fasteners',
    '6704ZZ | ground chrome steel races': 'bearing_races',
    '6704ZZ | satin pressed steel shields': 'bearing_shields',
    '6704ZZ | recessed steel clearance': 'bearing_recess',
}

def name_for(material):
    return LABELS.get(material.name, re.sub('[^a-z0-9]+', '_', material.name.lower()).strip('_'))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def old_text(relative):
    return subprocess.check_output(['git', '-C', str(REPO), 'show', f'{os.environ["OMX_BASE_REVISION"]}:{relative}']).decode()

def ownership(kind, part):
    parent = semantic(kind, part['group'])
    # Output horn / its center screw rotate with the downstream frame.
    # Idler retaining caps and their screws are mounted to the case, not its horn.
    n = part['sourceName']
    if 'HORN_DUMMY' in n or 'HORN_IDLE2_DUMMY' in n or 'BTS2_M2_6X8_5' in n:
        return parent + 1
    if n.startswith('DC11_A01_IDLER_DUMMY'):
        return parent + 1
    return parent

def simplify(mesh):
    key = hashlib.sha256(mesh.vertices.tobytes()+mesh.faces.tobytes()).hexdigest()[:24]
    cache = np.load(OUT / 'decimation' / f'{key}_lod.npz')
    clean = trimesh.Trimesh(vertices=cache['vertices'], faces=cache['faces'], process=True)
    if mesh.visual.material.baseColorTexture is not None:
        # PCB images use planar UVs. Reproject the affine UV map after QEM.
        design = np.column_stack([mesh.vertices, np.ones(len(mesh.vertices))])
        uv_map = np.linalg.lstsq(design, mesh.visual.uv, rcond=None)[0]
        error = np.max(np.abs(design @ uv_map - mesh.visual.uv))
        if error > 1e-4:
            clean = mesh.copy()
            uv = mesh.visual.uv.copy()
        else:
            uv = np.column_stack([clean.vertices, np.ones(len(clean.vertices))]) @ uv_map
        normal = clean.face_normals.mean(axis=0)
        if np.linalg.norm(normal) > .99:
            # Give zero-thickness Gerber/PCB plates 10 um inward backing.
            # Extrude the boundary only; do not make a prism for every triangle.
            normal /= np.linalg.norm(normal)
            n = len(clean.vertices)
            boundary = clean.edges[trimesh.grouping.group_rows(clean.edges_sorted, require_count=1)]
            faces = [clean.faces, clean.faces[:, ::-1] + n]
            faces += [np.array([[a, a+n, b+n], [a, b+n, b]]) for a,b in boundary]
            clean = trimesh.Trimesh(vertices=np.vstack([clean.vertices, clean.vertices-normal*.00001]),
                                    faces=np.vstack(faces), process=False)
            uv = np.vstack([uv, uv])
        clean.visual = trimesh.visual.TextureVisuals(uv=uv, material=mesh.visual.material)
        return clean
    clean.remove_unreferenced_vertices()
    return trimesh.graph.smooth_shade(clean, angle=np.radians(35), facet_minarea=None)

def export_obj(path, meshes, material):
    """One material per OBJ, with shared MTL and portable relative texture paths."""
    lines = ['# OMX visual mesh; coordinates in metres', 'mtllib materials.mtl',
             f'usemtl {material}']
    offset = 0
    # A whole file either has UVs or not because it has one source material.
    textured = all(getattr(m.visual, 'uv', None) is not None for m in meshes)
    for m in meshes:
        lines += ['v ' + ' '.join(f'{x:.7f}' for x in p) for p in m.vertices]
        lines += ['vn ' + ' '.join(f'{x:.6f}' for x in p) for p in m.vertex_normals]
        if textured:
            lines += ['vt ' + ' '.join(f'{x:.7f}' for x in p) for p in m.visual.uv]
        for f in m.faces:
            indices = f + offset + 1
            lines.append('f ' + ' '.join(f'{i}/{i}/{i}' if textured else f'{i}//{i}' for i in indices))
        offset += len(m.vertices)
    path.write_text('\n'.join(lines) + '\n')

def build(kind):
    short = 'omx_' + kind[0]
    base = PKG / 'meshes' / short
    dest = base / 'visual'
    dest.mkdir(exist_ok=True)
    parts, source = source_parts(kind)
    old, old_paths, origins = baseline(kind)
    materials = {}
    grouped = collections.defaultdict(list)
    part_log = []
    for pid, part in parts.items():
        link = ownership(kind, part)
        before = after = 0
        for m in source[pid]:
            mat = m.visual.material
            key = name_for(mat)
            if key not in materials:
                rgba = (mat.baseColorFactor / 255).tolist() if mat.baseColorFactor is not None else [1., 1., 1., 1.]
                linear_rgba = list(rgba)
                rgba[:3] = [12.92*x if x <= .0031308 else 1.055*x**(1/2.4)-.055 for x in rgba[:3]]
                info = dict(name=key, source=mat.name, rgba=rgba, source_linear_rgba=linear_rgba,
                            metallic=float(mat.metallicFactor if mat.metallicFactor is not None else 1),
                            roughness=float(mat.roughnessFactor if mat.roughnessFactor is not None else 1))
                if mat.baseColorTexture is not None:
                    texture_dir = dest / 'textures'
                    texture_dir.mkdir(exist_ok=True)
                    texture = mat.baseColorTexture.convert('RGB')
                    texture.thumbnail((1024, 1024))
                    texture.save(texture_dir / f'{key}.png', optimize=True)
                    info['texture'] = f'visual/textures/{key}.png'
                materials[key] = info
            simple = simplify(m)
            before += len(m.faces)
            after += len(simple.faces)
            simple.apply_translation(-origins[link])
            grouped[(link, key)].append(simple)
        part_log.append(dict(id=pid, source=part['sourceName'], group=part['group'],
                             link=link, source_triangles=before, triangles=after))
    rows = []
    for (link, key), meshes in sorted(grouped.items()):
        filename = f'link{link}_{key}.obj'
        path = dest / filename
        export_obj(path, meshes, key)
        bounds = np.array([np.min([m.bounds[0] for m in meshes], axis=0), np.max([m.bounds[1] for m in meshes], axis=0)])
        rows.append(dict(link=link, material=key, file=f'visual/{filename}',
                         triangles=sum(len(m.faces) for m in meshes), bytes=path.stat().st_size,
                         bounds=bounds.tolist(), sha256=sha(path)))
    mtl = ['# Classic OBJ materials; MuJoCo uses the matching explicit MJCF materials.']
    for key, m in sorted(materials.items()):
        rgb = ' '.join(f'{x:.6f}' for x in m['rgba'][:3])
        specular = .25 + .45 * m['metallic']
        mtl += [f'newmtl {key}', f'Kd {rgb}', f'Ka {rgb}',
                f'Ks {specular:.4f} {specular:.4f} {specular:.4f}',
                f'Ns {max(2, (1-m["roughness"])*160):.3f}', 'd 1', 'illum 2']
        if 'texture' in m:
            mtl += [f'map_Kd {m["texture"].removeprefix("visual/")}']
        mtl += ['']
    (dest / 'materials.mtl').write_text('\n'.join(mtl))
    if (base / 'materials.mtl').exists():
        (base / 'materials.mtl').unlink()  # Remove only our superseded generated MTL.
    relative = f'open_manipulator_description/urdf/{short}/{short}_arm.urdf.xacro'
    text = old_text(relative)
    # Replace only visual blocks; preserve comments, joints, collision and inertia text.
    def replace_link(match):
        index = int(re.search(r'link(\d+)', match[0])[1])
        visual = []
        for row in rows:
            if row['link'] != index:
                continue
            m = materials[row['material']]
            rgba = ' '.join(f'{v:.6f}' for v in m['rgba'])
            visual += [f'         <visual name="${{prefix}}link{index}_{m["name"]}">',
                       '            <origin xyz="0 0 0" rpy="0 0 0" />',
                       f'            <geometry><mesh filename="${{meshes_file_direction}}/{row["file"]}" /></geometry>',
                       f'            <material name="${{prefix}}{short}_{m["name"]}">',
                       f'               <color rgba="{rgba}" />']
            if 'texture' in m:
                visual += [f'               <texture filename="${{meshes_file_direction}}/{m["texture"]}" />']
            visual += ['            </material>', '         </visual>']
        block = re.sub(r'         <visual>.*?</visual>', '\n'.join(visual), match[0], flags=re.S)
        return block.replace('<collision>', f'<collision name="${{prefix}}link{index}_collision">')
    text = re.sub(r'<link name="\$\{prefix\}link\d+".*?</link>', replace_link, text, flags=re.S)
    (REPO / relative).write_text(text)
    manifest = dict(model=kind, units='metres', source_glb_sha256=sha(SOURCE / f'{kind}.glb'),
                    source_json_sha256=sha(SOURCE / f'{kind}.json'), excluded_groups='>=1000 (all cable assemblies)',
                    materials=materials, visuals=rows, parts=part_log,
                    collision_policy='Original STL and references preserved unchanged; not regenerated from render meshes.',
                    collision=[dict(file=p.name, sha256=sha(p), bytes=p.stat().st_size) for p in old_paths])
    (base / 'visual_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(kind, 'triangles', sum(r['triangles'] for r in rows), 'OBJ MB', round(sum(r['bytes'] for r in rows)/1e6, 2), flush=True)
    return manifest

def update_mjcf(manifest):
    rel = 'open_manipulator_description/mujoco/omx/omx.xml'
    text = old_text(rel)
    root = ET.fromstring(text.encode())
    asset = root.find('asset')
    for key, m in manifest['materials'].items():
        attrs = dict(name=f'omx_{key}', rgba=' '.join(f'{x:.6f}' for x in m['rgba']),
                     specular=f'{.25 + .45*m["metallic"]:.4f}', shininess=f'{(1-m["roughness"])*.6:.4f}')
        if 'texture' in m:
            ET.SubElement(asset, 'texture', name=f'omx_{key}', type='2d',
                          file=f'../../meshes/omx_f/{m["texture"]}')
            attrs['texture'] = f'omx_{key}'
        ET.SubElement(asset, 'material', **attrs)
    for r in manifest['visuals']:
        meshname = f'link{r["link"]}_{r["material"]}'
        ET.SubElement(asset, 'mesh', name=meshname, file=f'meshes/omx_f/{r["file"]}')
    for i in range(8):
        body = root.find(f'.//body[@name="link{i}"]')
        for g in list(body.findall('geom')):
            if g.get('group') == '2':
                body.remove(g)
        for r in manifest['visuals']:
            if r['link'] == i:
                ET.SubElement(body, 'geom', name=f'visual_link{i}_{r["material"]}', type='mesh',
                              mesh=f'link{i}_{r["material"]}', material=f'omx_{r["material"]}',
                              contype='0', conaffinity='0', group='2', density='0')
    ET.indent(root, space='  ')
    (REPO / rel).write_bytes(ET.tostring(root, pretty_print=True))

if __name__ == '__main__':
    for kind in ['leader', 'follower']:
        manifest = build(kind)
        if kind == 'follower':
            update_mjcf(manifest)

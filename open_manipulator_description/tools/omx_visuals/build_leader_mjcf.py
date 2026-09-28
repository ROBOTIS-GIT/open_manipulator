"""Generate a passive, textured MuJoCo leader model from the unchanged URDF chain."""
import os

import json
from pathlib import Path
import mujoco
from lxml import etree as ET
from validate_assets import expand
from build_assets import PKG, OUT

manifest=json.loads((PKG/'meshes/omx_l/visual_manifest.json').read_text())
model=mujoco.MjModel.from_xml_path(str(expand('leader')))
temp=OUT/'leader_generated.xml'
mujoco.mj_saveLastXML(str(temp),model)
root=ET.parse(str(temp)).getroot()
root.find('compiler').set('meshdir','../../')
asset=root.find('asset')
for mesh in asset.findall('mesh'):
    mesh.set('file',str(Path(mesh.get('file')).relative_to(PKG)))
for key,m in manifest['materials'].items():
    attrs=dict(name=f'omx_{key}',rgba=' '.join(f'{x:.6f}' for x in m['rgba']),
               specular=f'{.25+.45*m["metallic"]:.4f}',shininess=f'{(1-m["roughness"])*.6:.4f}')
    if 'texture' in m:
        ET.SubElement(asset,'texture',name=f'omx_{key}',type='2d',file=f'../../meshes/omx_l/{m["texture"]}')
        attrs['texture']=f'omx_{key}'
    ET.SubElement(asset,'material',**attrs)
material_by_mesh={Path(r['file']).stem:r['material'] for r in manifest['visuals']}
for geom in root.findall('.//geom'):
    mesh=geom.get('mesh')
    if mesh in material_by_mesh:
        geom.set('material',f'omx_{material_by_mesh[mesh]}')
        geom.attrib.pop('rgba',None)
        geom.set('group','2');geom.set('density','0')
    else:
        geom.set('group','3')
root.insert(1,ET.Comment(' Passive leader model generated from omx_l_arm.urdf.xacro. No actuator/controller tuning is implied. '))
folder=PKG/'mujoco/omx_l';folder.mkdir(exist_ok=True)
ET.indent(root,space='  ')
(folder/'omx_l.xml').write_bytes(ET.tostring(root,pretty_print=True))
scene=(PKG/'mujoco/omx/scene.xml').read_text().replace('omx scene','omx leader scene').replace('file="omx.xml"','file="omx_l.xml"')
(folder/'scene.xml').write_text(scene)
compiled=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
print('leader native MJCF',compiled.nq,compiled.nmesh,flush=True)

"""Refresh checked-in standalone URDF visuals without changing their other settings."""
import copy
import re
from lxml import etree as ET
from build_assets import PKG
from validate_assets import expand

for kind in ['leader', 'follower']:
    short = 'omx_' + kind[0]
    expanded = ET.parse(str(expand(kind))).getroot()
    path = PKG / f'urdf/{short}/{short}.urdf'
    original = path.read_text()

    def replace_link(match):
        name = re.search(r'<link name="([^"]+)"', match[0])[1]
        link = expanded.find(f'link[@name="{name}"]')
        assert link is not None, name
        visuals = []
        for element in link.findall('visual'):
            visual = copy.deepcopy(element)
            visual.tail = None
            for resource in visual.findall('.//*[@filename]'):
                resource.set('filename', resource.get('filename').replace(
                    str(PKG) + '/', 'package://open_manipulator_description/'))
            ET.indent(visual, space='  ', level=2)
            visuals.append('    ' + ET.tostring(visual, encoding='unicode'))
        block = re.sub(r'    <visual(?:\s[^>]*)?>.*?</visual>\n', '', match[0], flags=re.S)
        return block.replace('>\n', '>\n' + '\n'.join(visuals) + '\n', 1)

    updated = re.sub(r'<link name="link\d+".*?</link>', replace_link, original, flags=re.S)
    path.write_text(updated)
    print(f'Synchronized {short}.urdf', flush=True)

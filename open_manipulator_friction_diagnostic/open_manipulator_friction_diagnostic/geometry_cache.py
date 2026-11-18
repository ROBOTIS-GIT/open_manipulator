"""Reuse completed fixed-trajectory geometry reviews, never actual-start homing."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np

from . import generalization_motion
from .protocol import JOINTS, sha


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _mesh_path(filename, urdf_path):
    uri = urlparse(filename)
    if uri.scheme == 'package':
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(uri.netloc)) / unquote(uri.path.lstrip('/'))
    if uri.scheme == 'file' and uri.netloc in ('', 'localhost'):
        return Path(unquote(uri.path))
    if uri.scheme:
        raise ValueError('Unsupported collision mesh URI: ' + filename)
    path = Path(filename)
    return path if path.is_absolute() else urdf_path.parent / path


def _inputs(trajectory, urdf_path):
    import pinocchio
    import coal
    root = ET.fromstring(urdf_path.read_text())
    geometry = ET.Element('robot')
    meshes = {}
    # Ignore hardware mode and visual assets. Preserve every link/joint field
    # consumed by the geometry model, including fixed transforms and mesh scale.
    for node in root:
        if node.tag not in ('link', 'joint'):
            continue
        for visual in list(node.findall('visual')):
            node.remove(visual)
        for mesh in node.findall('collision/geometry/mesh'):
            path = _mesh_path(mesh.attrib['filename'], urdf_path).resolve()
            if path not in meshes:
                meshes[path] = sha(path)
            mesh.attrib['filename'] = 'sha256:' + meshes[path]
        for child in node.iter():
            child.text = None
            child.tail = None
        geometry.append(node)
    arrays = {}
    for name in ('t', 'q', 'v', 'a'):
        array = np.ascontiguousarray(trajectory[name], dtype='<f8')
        if not np.isfinite(array).all():
            raise ValueError('Non-finite trajectory cannot use a geometry review')
        arrays[name] = dict(shape=list(array.shape), sha256=hashlib.sha256(array.tobytes()).hexdigest())
    xml = ET.canonicalize(ET.tostring(geometry, encoding='unicode'))
    return dict(schema=1, trajectory=arrays, geometry_sha256=hashlib.sha256(xml.encode()).hexdigest(),
                checker_sha256=sha(generalization_motion.__file__), cache_implementation_sha256=sha(__file__),
                joint_names=JOINTS,
                pinocchio_version=pinocchio.__version__, coal_version=coal.__version__)


def _passed(result, samples):
    try:
        return (result['passed'] is True and result['samples'] == samples
                and result['collision_samples'] == 0 and result['collision_pairs'] > 0
                and 0 < result['unique_positions'] <= samples
                and result['minimum_distance_m'] >= result['minimum_required_distance_m'] > 0
                and result['floor_minimum_m'] >= result['minimum_table_clearance_m'] > 0
                and np.isfinite([result[k] for k in (
                    'minimum_distance_m', 'minimum_required_distance_m', 'floor_minimum_m',
                    'minimum_table_clearance_m', 'table_plane_world_z_m')]).all())
    except (KeyError, TypeError, ValueError):
        return False


def review(trajectory, urdf_path, *, checker=None, cache_dir=None, seed_dir=None):
    """Read an exact packaged/local match or run and atomically save a review.

    Packaged results can be prepared once off-robot. Local results survive raw
    collection rotation. Missing/corrupt/stale entries always require a check.
    """
    started = time.monotonic()
    urdf_path = Path(urdf_path)
    inputs = _inputs(trajectory, urdf_path)
    key = _digest(inputs)
    cache_dir = Path(cache_dir) if cache_dir is not None else Path.home()/'.ros/omy_friction/geometry_cache'
    if seed_dir is None:
        from .artifact import resource_path
        try:
            seed_dir = resource_path('calibration/geometry_cache')
        except (ImportError, LookupError):
            seed_dir = Path(__file__).resolve().parents[1]/'calibration/geometry_cache'
    filename = key + '.json'
    for kind, folder in (('packaged', Path(seed_dir)), ('local', cache_dir)):
        path = folder/filename
        try:
            entry = json.loads(path.read_text())
            result = entry['geometry']
            if (entry['inputs'] != inputs or entry['geometry_sha256'] != _digest(result)
                    or not _passed(result, len(trajectory['q']))):
                continue
        except (OSError, ValueError, TypeError, KeyError):
            continue
        print(f'GEOMETRY cache hit ({kind}); fixed trajectory review reused', flush=True)
        return result, dict(hit=True, source=kind, key=key, path=str(path),
                            elapsed_s=time.monotonic()-started)
    print(f"GEOMETRY cache miss; checking {len(trajectory['q'])} trajectory samples before motor startup", flush=True)
    result = (checker or generalization_motion.check_geometry)(trajectory['q'], urdf_path)
    info = dict(hit=False, source='computed', key=key, elapsed_s=time.monotonic()-started)
    if _passed(result, len(trajectory['q'])):
        # Refuse to publish a proof if mesh/code inputs changed during checking.
        if _inputs(trajectory, urdf_path) != inputs:
            raise ValueError('Geometry inputs changed during review; result was not cached')
        entry = dict(inputs=inputs, geometry=result, geometry_sha256=_digest(result))
        name = None
        try:
            cache_dir.mkdir(parents=True, exist_ok=True)
            fd, name = tempfile.mkstemp(prefix='.geometry-', suffix='.tmp', dir=cache_dir)
            with os.fdopen(fd, 'w') as stream:
                json.dump(entry, stream, indent=2, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, cache_dir/filename)
            info['path'] = str(cache_dir/filename)
        except OSError as exc:
            # The actual completed geometry check is still usable this run.
            info['cache_write_error'] = str(exc)
            print(f'GEOMETRY review passed; cache could not be saved: {exc}', flush=True)
        finally:
            if name and Path(name).exists():
                Path(name).unlink()
    print(f"GEOMETRY review finished in {time.monotonic()-started:.1f} s; passed={result.get('passed')}", flush=True)
    return result, info

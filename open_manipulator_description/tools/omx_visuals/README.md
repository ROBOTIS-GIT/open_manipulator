# Rebuilding the OMX visuals

The packaged OBJ/MTL/PNG files work without Blender or this tooling at runtime.
This pipeline is for maintainers updating the visual assets. It was run with
Blender 5.2.1 LTS and the pinned Python packages in `requirements.txt`.

Input: locally supplied `leader.glb`, `leader.json`, `follower.glb`, `follower.json`
from the revised assembly model. Each JSON maps CAD parts to assembly groups;
GLB nodes retain `part_<id>` names. Input source files are not included here.

```bash
python3 -m venv /tmp/omx-assets-venv
/tmp/omx-assets-venv/bin/pip install -r requirements.txt
/tmp/omx-assets-venv/bin/python run.py \
  --source-models /path/to/source/models \
  --blender /path/to/blender \
  --output /tmp/omx-asset-build
```

This modifies only the OMX visual outputs, the two arm Xacros, and the native
MuJoCo visual definitions in the local repository. It preserves the baseline
STLs and mechanical/control parameters. It never publishes assets or modifies
the input GLB/JSON files. Keep unrelated local edits out of the generated Xacros
and MJCF before rebuilding: those files are regenerated from `--base-revision`.

The baseline revision is pinned to the upstream version used for this update,
not moving `HEAD`. That revision must exist in the local Git object database.
For a later mechanical revision, deliberately update the baseline and recheck
part-to-link mapping in `inspect_alignment.py` and `build_assets.py`.

The transform from the glTF Y-up source to the URDF zero pose is
`(x, y, z)_URDF = (z, x, y)_glTF`. The appropriate cumulative link origin is
subtracted before export. Visuals use metres; legacy collision STL uses millimetres.

Decimation is per component. Housing and base features receive higher budgets
than fasteners. UV plates are reprojected only when their UV map is affine;
otherwise the source UV geometry is retained. Planar board decals receive a
10-micrometre inward backing for importers that require nonzero mesh volume.

Validation without the source files or Blender:

```bash
python run.py --validate-only --output /tmp/omx-asset-check
```

Checks include references, OBJ texture loading, prefix expansion, direct URDF
and native MJCF import, joint poses, unchanged collision files, unchanged
Follower dynamics and a 250-step baseline/new trajectory comparison. Actual
MuJoCo images and `validation.json` are written to the output directory.
Rendering requires an available OpenGL context. On headless Linux configure
MuJoCo EGL/OSMesa as appropriate for that host.

`geometry_qa.json` contains bidirectional **sampled** surface distances for frames
and motor housings. These are not certified global Hausdorff bounds or a collision
clearance certification. Sparse pose/view checks are not full joint-range testing.

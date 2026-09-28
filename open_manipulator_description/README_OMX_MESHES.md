# OMX simulation visuals

OMX Leader and Follower use cable-free, material-separated Wavefront OBJ visuals.
The existing STL collision meshes, link frames, joint names, axes, limits, inertias
and controller configuration are retained. No extra joints are introduced for
decorative parts.

## Files and units

- `meshes/omx_l/visual/` and `meshes/omx_f/visual/`: OBJ meshes in **metres**,
  a shared `materials.mtl`, and board textures in `textures/`.
- The original STL files remain in their original locations and use millimetres,
  with the existing `0.001` scale in URDF/MJCF.
- `visual_manifest.json`: per-material mesh counts, source hashes and part-to-link
  ownership. Cable assembly groups are excluded from the export.
- `urdf/omx_l/` and `urdf/omx_f/`: existing ROS entry points; no launch/API change.
- `mujoco/omx/scene.xml`: existing Follower simulation with updated visuals.
- `mujoco/omx_l/scene.xml`: passive Leader model, generated from the Leader URDF.
  This contains no invented actuator/controller calibration.

The separate motor output horns are attached to their downstream links; the motor
cases and idler retaining caps remain on the case side. The source XL430 housing
is an integrated CAD solid; this update does not create a new actuator mechanism
or infer internal gear dynamics from its visual shape.

## Using the models

Build/source `open_manipulator_description` as usual for ROS. Existing model and
prefix arguments continue to work. Keep each `visual` directory intact: OBJ,
MTL and texture files must travel together. CMake already installs the entire
`meshes` and `mujoco` directories.

For MuJoCo, load either native scene directly:

```python
import mujoco

follower = mujoco.MjModel.from_xml_path(
    "open_manipulator_description/mujoco/omx/scene.xml")
leader = mujoco.MjModel.from_xml_path(
    "open_manipulator_description/mujoco/omx_l/scene.xml")
```

On macOS, interactive `mujoco.viewer.launch_passive` generally needs `mjpython`
instead of the regular Python executable. Native MJCF explicitly binds each
material and texture; it does not depend on MuJoCo interpreting OBJ MTL files.
MuJoCo URDF import is also checked, but native MJCF is the intended path for board
textures and the existing Follower actuator/contact configuration.

Visuals are non-colliding and contribute no inferred mass to native MJCF. Geom
group 2 is visual geometry; group 3 is the existing collision geometry. Keep group
3 hidden when inspecting appearance. The Follower's red TCP marker is inherited
from the existing model and is not part of the physical gripper.

## Compatibility and limits

The OBJ/MTL assets use conventional diffuse/specular materials with explicit
normals and UV coordinates. URDF includes matching color/texture references.
glTF linear colors are converted for these classic material pipelines; procedural
Blender shaders are not dependencies. Results can vary with renderer lighting.

Verified locally: Xacro expansion with empty/nonempty prefixes, all mesh and
texture references, direct URDF import, native MJCF loading, forward kinematics,
and actual MuJoCo rendered views. MuJoCo 3.14.0 was used for full checks;
both native scenes also compile and step with MuJoCo 3.3.7. ROS RViz/Gazebo and Isaac Sim runtime checks
remain to be performed on those platforms; they are not claimed as tested here.

This is an **appearance update**, not a new system-identification release. The
existing collision models are retained to avoid changing contact behavior during
the visual update. They do not newly resolve every PCB component, hole or camera
feature. Mass/COM/inertia have not been remeasured for revised electronic parts.

## Regeneration

`tools/omx_visuals/` contains the export and validation pipeline. Source assembly
GLB/JSON inputs are not bundled. See that directory's README for requirements and
rebuild commands. Original film, CAD and web assets are not modified by rebuilding.

The design follows the separation used in MuJoCo Menagerie robot models:
[UR5e](https://github.com/google-deepmind/mujoco_menagerie/blob/main/universal_robots_ur5e/ur5e.xml)
and [Franka FR3](https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_fr3/fr3.xml).
No UR or Franka geometry was copied into these assets.

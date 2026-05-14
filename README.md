<p align="center">
  <img src="https://huggingface.co/datasets/fracapuano/blogs/resolve/main/orca-arm-logo.png" width="640" alt="orca_arm logo" />
</p>

# orca_arm

URDF and MJCF descriptions for Orca robot embodiments, plus every referenced
mesh bundled inside the package:

- **OrcaBot**: a bimanual OpenArm with two OrcaHand end effectors.
- **OrcaPanda**: a reference integration showing a right OrcaHand mounted on a
  Franka Emika Panda arm.

This repository does *not* come with controllers, IK, motion planning, or a simulator---bring your own!

## Install

```bash
git clone https://github.com/fracapuano/orca_arm.git
pip install -e .            # core: paths to URDF / MJCF / meshes
pip install -e .[viz]       # optionally adds meshcat + yourdfpy for the visualizer
```

## What you get

The package exposes absolute paths to the bundled URDF and MJCF files. OrcaBot
remains the default embodiment; OrcaPanda is opt-in:

```python
import orca_arm

orca_arm.URDF_PATH             # path to orcabot.urdf
orca_arm.MJCF_PATH             # path to orcabot.xml (MuJoCo)
orca_arm.ORCAPANDA_URDF_PATH   # path to orcapanda.urdf
orca_arm.ORCAPANDA_MJCF_PATH   # path to orcapanda.xml (MuJoCo)
```

The URDF and MJCF reference meshes via paths relative to their own location, so any tool that resolves mesh paths from the URDF/MJCF file finds them without further configuration. No environment variables, no `package://` resolver setup.

## Visualize

You can visualize the bundled embodiments with a meshcat viewer for a quick
inspection:

```bash
python visualize_orcabot.py                              # OrcaBot live FK loop
python visualize_orcabot.py --idle                       # OrcaBot home configuration
python visualize_orcabot.py --embodiment orcapanda       # OrcaPanda live FK loop
python visualize_orcabot.py --embodiment orcapanda --idle
```

## MuJoCo

Load the bundled MJCF directly:

```python
import mujoco
import orca_arm

model = mujoco.MjModel.from_xml_path(orca_arm.MJCF_PATH)
data = mujoco.MjData(model)
```

This is the entry point for any MuJoCo-based stack consuming MJCF.
Use `orca_arm.ORCAPANDA_MJCF_PATH` for the Panda reference integration.

## Other URDF-consuming simulators

PyBullet, SAPIEN, ManiSkill, Isaac Sim, Drake, ROS — all accept the URDF path. PyBullet example:

```python
import pybullet as p
import orca_arm

p.connect(p.DIRECT)
robot = p.loadURDF(orca_arm.URDF_PATH)
```

The same pattern works for any other URDF loader: hand it `orca_arm.URDF_PATH`.
Use `orca_arm.ORCAPANDA_URDF_PATH` for the Panda reference integration.

## Forward kinematics with yourdfpy

For pure kinematics — link / joint queries, forward kinematics, scene graph — without a physics simulator:

```python
import numpy as np
import yourdfpy
import orca_arm

robot = yourdfpy.URDF.load(orca_arm.URDF_PATH)

q = np.zeros(len(robot.actuated_joint_names))
robot.update_cfg(q)

# 4x4 world transform of any link in the current configuration
T_world_link, _ = robot.scene.graph.get("<link_name>")
```

## Reference Embodiments

OrcaPanda is included as a concrete integration example, not as a replacement
for OrcaBot or a commitment that every future arm should get a bespoke builder.
It keeps the upstream Panda arm through `panda_link8`, omits the stock Franka
hand, prefixes the OrcaHand links and joints with `orcahand_right_`, and mounts
the OrcaHand root with one fixed adapter joint:

```text
panda_link8 -> panda_link8_to_orcahand_joint -> orcahand_right_ForeArmStructure-Model_e18f2368
```

The generated model has 24 actuated joints: 7 Panda arm joints and 17 OrcaHand
joints. The intent is to demonstrate how OrcaHand can be packaged with an
external robot arm for downstream applications.

The Panda source description comes from `franka_ros_repo/franka_description`
on the upstream `franka_ros` `noetic-devel` branch, currently pinned by the
submodule at commit `35e1f654426e04bc9f83b73af4ab68a3fb145c84`. See
`orca_arm/THIRD_PARTY_NOTICES.md` for bundled asset attribution.

## Layout

| Path | Purpose |
| --- | --- |
| `orca_arm/orcabot.urdf` | Bimanual robot description |
| `orca_arm/orcabot.xml` | MuJoCo MJCF, generated from the URDF |
| `orca_arm/orcapanda.urdf` | Mono Panda + OrcaHand robot description |
| `orca_arm/orcapanda.xml` | MuJoCo MJCF for the Panda + OrcaHand embodiment |
| `orca_arm/assets/` | Bundled mesh files (`.stl`, `.dae`) |
| `visualize_orcabot.py` | Meshcat viewer (live FK, or optionally `--idle`) |
| `build_orcabot_urdf.py` | Regenerates the URDF from the OpenArm + OrcaHand source descriptions |
| `build_orcabot_mjcf.py` | Regenerates the MJCF from the URDF |
| `build_orcapanda_urdf.py` | Regenerates the Panda + OrcaHand URDF |
| `build_orcapanda_mjcf.py` | Regenerates the Panda + OrcaHand MJCF |
| `tests/` | Checks every referenced mesh resolves and FK is well-defined |


We are also releasing `build_*.py` scripts to regenerate the URDF and MJCF from
the OpenArm, Franka Panda, and OrcaHand source descriptions.
These are only relevant if you are updating the assets themselves; ordinary downstream use does not require running them!

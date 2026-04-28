<p align="center">
  <img src="https://huggingface.co/datasets/fracapuano/blogs/resolve/main/orca-arm-logo.png" width="640" alt="orca_arm logo" />
</p>

# orca_arm

URDF and MJCF descriptions for the **OrcaArm**---a bimanual OpenArm with two OrcaHand end effectors, plus every referenced mesh bundled inside the package: use it 

This repository does *not* come with controllers, IK, motion planning, or a simulator---bring your own!

## Install

```bash
git clone https://github.com/fracapuano/orca_arm.git
pip install -e .            # core: paths to URDF / MJCF / meshes
pip install -e .[viz]       # optionally adds meshcat + yourdfpy for the visualizer
```

## What you get

The package exposes absolute paths to the bundled URDF and MJCF files of the OrcaArm:

```python
import orca_arm

orca_arm.URDF_PATH    # path to orcabot.urdf
orca_arm.MJCF_PATH    # path to orcabot.xml (MuJoCo)
```

The URDF and MJCF reference meshes via paths relative to their own location, so any tool that resolves mesh paths from the URDF/MJCF file finds them without further configuration. No environment variables, no `package://` resolver setup.

## Visualize

You can visualize the OrcaArm with a meshcat viewer for a quick inspection:

```bash
python visualize_orcabot.py            # live FK loop with a random joint sweep - no collision detection here
python visualize_orcabot.py --idle     # static home configuration
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

## Other URDF-consuming simulators

PyBullet, SAPIEN, ManiSkill, Isaac Sim, Drake, ROS — all accept the URDF path. PyBullet example:

```python
import pybullet as p
import orca_arm

p.connect(p.DIRECT)
robot = p.loadURDF(orca_arm.URDF_PATH)
```

The same pattern works for any other URDF loader: hand it `orca_arm.URDF_PATH`.

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

## Layout

| Path | Purpose |
| --- | --- |
| `orca_arm/orcabot.urdf` | Bimanual robot description |
| `orca_arm/orcabot.xml` | MuJoCo MJCF, generated from the URDF |
| `orca_arm/assets/` | Bundled mesh files (`.stl`, `.dae`) |
| `visualize_orcabot.py` | Meshcat viewer (live FK, or optionally `--idle`) |
| `build_orcabot_urdf.py` | Regenerates the URDF from the OpenArm + OrcaHand source descriptions |
| `build_orcabot_mjcf.py` | Regenerates the MJCF from the URDF |
| `tests/` | Checks every referenced mesh resolves and FK is well-defined |


We are also releasing `build_*.py` scripts to regenerate the URDF and MJCF from the OpenArm and OrcaHand source descriptions.
These are only relevant if you are updating the assets themselves; ordinary downstream use does not require running them!

# Third-Party Notices

This package bundles generated robot descriptions and mesh assets from the
following upstream description packages so the URDF/MJCF files can be loaded
without external ROS package resolution.

## Franka Panda URDF Assets

- Source: `franka_description` from `https://github.com/frankaemika/franka_ros`
- Branch: `noetic-devel`
- Commit: `35e1f654426e04bc9f83b73af4ab68a3fb145c84`
- Upstream package version: `0.10.1`
- License: Apache License 2.0
- Copyright notice from upstream `NOTICE`:

```text
franka_ros

Copyright 2017 Franka Emika GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
```

The `orcapanda` URDF uses the Panda Xacro, joint limits, and visual and
collision meshes from `franka_description`, with mesh paths rewritten into the
local `orca_arm/assets/` package directory.

## Franka Panda MuJoCo Assets

- Source: `franka_emika_panda` from
  `https://github.com/google-deepmind/mujoco_menagerie`
- Commit: `d7cbaf53e50bee13ae467cc807931aef2f9efe46`
- License: Apache License 2.0

The `orcapanda` MJCF uses the MuJoCo Menagerie Panda no-hand model and mesh
assets for the Panda arm. Those files are vendored under
`orca_arm/assets/franka_emika_panda/`; see the copied `LICENSE` and `README.md`
in that directory for upstream terms and generation notes.

## OrcaHand

- Source: `https://github.com/orcahand/orcahand_description`
- License: MIT License

The OrcaHand URDF and meshes are bundled into the generated OrcaBot and
OrcaPanda descriptions with side-specific link, joint, and mesh prefixes.

## OpenArm

- Source: `https://github.com/enactic/openarm_description`
- License: Apache License 2.0

OpenArm source descriptions and meshes are used for the OrcaBot embodiment.

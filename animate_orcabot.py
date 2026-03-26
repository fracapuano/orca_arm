#!/usr/bin/env python3
"""Animate the orcabot URDF with random joint motions in meshcat."""
import yourdfpy
import meshcat
import meshcat.geometry as g
import numpy as np
import os
import time

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "orcabot.urdf")

print("Loading URDF...")
robot = yourdfpy.URDF.load(URDF_PATH)
print(f"Robot: {robot.robot.name} — {len(robot.actuated_joint_names)} actuated joints")

# ── Meshcat setup ──────────────────────────────────────────────────────────
vis = meshcat.Visualizer()
print(f"\nOpen in browser: {vis.url()}")
vis.open()
vis.delete()

# ── Build name mapping: trimesh scene graph name -> meshcat path ───────────
# We add all geometries once, then just update transforms each frame.
scene = robot.scene
geom_map = {}  # meshcat_path -> scene graph node name

for name in scene.graph.nodes_geometry:
    try:
        transform, geometry_name = scene.graph.get(name)
    except Exception:
        continue
    if geometry_name is None or geometry_name not in scene.geometry:
        continue
    mesh = scene.geometry[geometry_name]
    if not hasattr(mesh, 'vertices') or not hasattr(mesh, 'faces'):
        continue

    vertices = np.array(mesh.vertices, dtype=np.float32)
    faces = np.array(mesh.faces, dtype=np.uint32)

    color = 0xCCCCCC
    if hasattr(mesh, 'visual') and hasattr(mesh.visual, 'main_color'):
        c = mesh.visual.main_color
        if c is not None and len(c) >= 3:
            color = int(c[0]) << 16 | int(c[1]) << 8 | int(c[2])

    safe_name = name.replace("/", "_").replace(" ", "_")
    mpath = f"robot/{safe_name}"

    vis[mpath].set_object(
        g.TriangularMeshGeometry(vertices, faces),
        g.MeshPhongMaterial(color=color, reflectivity=0.5)
    )
    vis[mpath].set_transform(transform.astype(np.float64))
    geom_map[mpath] = name

print(f"Added {len(geom_map)} geometries to scene.")

# ── Joint limits ───────────────────────────────────────────────────────────
n_joints = len(robot.actuated_joint_names)
lower = np.zeros(n_joints)
upper = np.zeros(n_joints)

for i, jname in enumerate(robot.actuated_joint_names):
    for j in robot.robot.joints:
        if j.name == jname and j.limit is not None:
            lower[i] = j.limit.lower
            upper[i] = j.limit.upper
            break

# ── Animation loop ─────────────────────────────────────────────────────────
print("\nAnimating... Press Ctrl+C to stop.")
print("Joints will smoothly interpolate between random targets.\n")

dt = 0.05  # 20 fps
interp_time = 2.0  # seconds to reach each target
steps_per_target = int(interp_time / dt)

current_cfg = np.zeros(n_joints)
target_cfg = lower + np.random.random(n_joints) * (upper - lower)

try:
    step = 0
    while True:
        # Smooth interpolation toward target
        t = (step % steps_per_target) / steps_per_target
        # Ease in-out
        t_smooth = 0.5 - 0.5 * np.cos(np.pi * t)

        if step % steps_per_target == 0:
            # New target
            start_cfg = current_cfg.copy()
            target_cfg = lower + np.random.random(n_joints) * (upper - lower)

        cfg = start_cfg + t_smooth * (target_cfg - start_cfg)
        current_cfg = cfg

        # Update URDF configuration
        robot.update_cfg(cfg)

        # Get updated scene and refresh meshcat transforms
        scene = robot.scene
        for mpath, scene_name in geom_map.items():
            try:
                transform, _ = scene.graph.get(scene_name)
                vis[mpath].set_transform(transform.astype(np.float64))
            except Exception:
                pass

        time.sleep(dt)
        step += 1

except KeyboardInterrupt:
    print("\nStopped.")

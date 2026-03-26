#!/usr/bin/env python3
"""Visualize the orcabot URDF in the browser using meshcat."""
import yourdfpy
import meshcat
import meshcat.geometry as g
import numpy as np
import os
import time
import trimesh

urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "orcabot.urdf")
print(f"Loading URDF: {urdf_path}")

robot = yourdfpy.URDF.load(urdf_path)
print(f"Robot: {robot.robot.name}")
print(f"Links: {len(robot.robot.links)}, Joints: {len(robot.robot.joints)}")

# Get the trimesh scene with all geometries and transforms
scene = robot.scene

# Start meshcat server
vis = meshcat.Visualizer()
print(f"\nOpen this URL in your browser: {vis.url()}")
vis.open()

# Clear the scene
vis.delete()

# Add each geometry from the trimesh scene into meshcat
for name, geom_name in scene.graph.to_flattened().items():
    try:
        transform, geometry_name = scene.graph.get(name)
    except Exception:
        continue

    if geometry_name is None:
        continue

    if geometry_name not in scene.geometry:
        continue

    mesh = scene.geometry[geometry_name]

    if not hasattr(mesh, 'vertices') or not hasattr(mesh, 'faces'):
        continue

    vertices = np.array(mesh.vertices, dtype=np.float32)
    faces = np.array(mesh.faces, dtype=np.uint32)

    # Get color from the mesh
    color = 0xCCCCCC  # default grey
    if hasattr(mesh, 'visual') and hasattr(mesh.visual, 'main_color'):
        c = mesh.visual.main_color
        if c is not None and len(c) >= 3:
            color = int(c[0]) << 16 | int(c[1]) << 8 | int(c[2])

    # Sanitize the name for meshcat path
    safe_name = name.replace("/", "_").replace(" ", "_")

    vis[f"robot/{safe_name}"].set_object(
        g.TriangularMeshGeometry(vertices, faces),
        g.MeshPhongMaterial(color=color, reflectivity=0.5)
    )
    vis[f"robot/{safe_name}"].set_transform(transform.astype(np.float64))

print(f"\nLoaded {len(scene.geometry)} geometries into meshcat.")
print("Viewer is running. Press Ctrl+C to stop.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nDone.")

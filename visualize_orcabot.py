
"""Visualize the orcabot URDF in meshcat with a live wrist-triad readout."""
import argparse
import yourdfpy
import meshcat
import meshcat.geometry as g
import numpy as np
import time
import orca_arm

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--idle",
    action="store_true",
    help="Skip the random joint sweep; show the robot at its home configuration "
         "with the live wrist triad and wait for Ctrl+C.",
)
args = parser.parse_args()

urdf_path = orca_arm.URDF_PATH
print(f"Loading URDF: {urdf_path}")

robot = yourdfpy.URDF.load(urdf_path)
print(f"Robot: {robot.robot.name}")
print(f"Links: {len(robot.robot.links)}, Joints: {len(robot.robot.joints)}")

scene = robot.scene

vis = meshcat.Visualizer()
print(f"\nOpen this URL in your browser: {vis.url()}")
vis.open()
vis.delete()

# ── Robot meshes: create once, refresh transforms each tick ────────────────
geom_map = {}  # meshcat_path -> scene-graph node name

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
        g.MeshPhongMaterial(color=color, reflectivity=0.5),
    )
    vis[mpath].set_transform(transform.astype(np.float64))
    geom_map[mpath] = name

print(f"Loaded {len(scene.geometry)} geometries into meshcat.")

# ── Wrist-triad geometry: cylinders created once, transforms updated each tick
AXIS_LEN = 0.15
AXIS_R = 0.005


def axis_local_transform(axis_dir, length):
    """Pose of a cylinder (default +Y, centered at origin) so it extends from
    the parent origin along axis_dir for `length` meters."""
    axis = np.asarray(axis_dir, dtype=float)
    y = np.array([0., 1., 0.])
    if np.allclose(axis, y):
        R = np.eye(3)
    elif np.allclose(axis, -y):
        R = np.diag([1., -1., -1.])
    else:
        v = np.cross(y, axis)
        s = float(np.linalg.norm(v))
        c = float(np.dot(y, axis))
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + K @ K * ((1 - c) / (s ** 2))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = axis * (length / 2)
    return T


axis_specs = (
    ("x", np.array([1.0, 0.0, 0.0]), 0xff0000),
    ("y", np.array([0.0, 1.0, 0.0]), 0x00ff00),
    ("z", np.array([0.0, 0.0, 1.0]), 0x0000ff),
)
axis_local_T = {n: axis_local_transform(d, AXIS_LEN) for n, d, _ in axis_specs}


def find_carpals_link(scene_, side):
    return next(
        (n for n in scene_.graph.nodes
         if f"orcahand_{side}_" in n and "Carpals" in n),
        None,
    )


right_link = find_carpals_link(scene, "right")
left_link = find_carpals_link(scene, "left")

side_links = {"right": right_link, "left": left_link}
triad_sides = []
for side, link in side_links.items():
    if link is None:
        continue
    triad_sides.append((side, link))
    for axis_name, _, color in axis_specs:
        vis[f"ik_target/{side}/{axis_name}"].set_object(
            g.Cylinder(height=AXIS_LEN, radius=AXIS_R),
            g.MeshLambertMaterial(color=color),
        )
    print(f"  Triad bound to {side} IK target ({link})")

if not triad_sides:
    print("Warning: no carpals link found; no IK-target triads will be drawn.")


def wrist_world_transforms(scene_):
    """{side: 4x4 world transform} for the wrist triads at the current FK.

    Position tracks each side's carpals link; orientation is forced to world
    identity so both wrists' IK target frames are aligned with the global
    X/Y/Z axes."""
    out = {}
    for side, link in side_links.items():
        if link is None:
            continue
        T_raw, _ = scene_.graph.get(link)
        T = T_raw.astype(np.float64).copy()
        T[:3, :3] = np.eye(3)
        out[side] = T
    return out


def push_to_meshcat(scene_):
    """Push current FK state to meshcat: link meshes + wrist triads."""
    for mpath, scene_name in geom_map.items():
        try:
            transform, _ = scene_.graph.get(scene_name)
            vis[mpath].set_transform(transform.astype(np.float64))
        except Exception:
            pass
    for side, T_world in wrist_world_transforms(scene_).items():
        for axis_name in axis_local_T:
            vis[f"ik_target/{side}/{axis_name}"].set_transform(
                T_world @ axis_local_T[axis_name]
            )


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

# ── Live FK loop ───────────────────────────────────────────────────────────
if args.idle:
    robot.update_cfg(np.zeros(n_joints))
    push_to_meshcat(robot.scene)
    print("\nIdle mode: robot held at home configuration.")
    print("Press Ctrl+C to stop.\n")
    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        pass
else:
    print("\nLive FK loop running. Triad tracks the wrist as joints move.")
    print("Press Ctrl+C to stop.\n")

    dt = 0.05
    interp_time = 2.0
    steps_per_target = int(interp_time / dt)
    rng = np.random.default_rng()

    current_cfg = np.zeros(n_joints)
    start_cfg = current_cfg.copy()
    target_cfg = lower + rng.random(n_joints) * (upper - lower)

    try:
        step = 0
        while True:
            if step % steps_per_target == 0:
                start_cfg = current_cfg.copy()
                target_cfg = lower + rng.random(n_joints) * (upper - lower)
            t = (step % steps_per_target) / steps_per_target
            t_smooth = 0.5 - 0.5 * np.cos(np.pi * t)
            current_cfg = start_cfg + t_smooth * (target_cfg - start_cfg)

            robot.update_cfg(current_cfg)
            push_to_meshcat(robot.scene)

            time.sleep(dt)
            step += 1

    except KeyboardInterrupt:
        pass

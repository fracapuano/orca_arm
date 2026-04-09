#!/usr/bin/env python3
"""
Convert orcabot.urdf -> orcabot.xml (MJCF) via MuJoCo's URDF compiler.

MuJoCo strips mesh directories by default; we inject a
<mujoco><compiler strippath="false" discardvisual="false"/></mujoco>
block into the URDF root so the absolute file:// mesh paths written by
build_orcabot_urdf.py are preserved.
"""
import os
import xml.etree.ElementTree as ET
import mujoco

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_IN = os.path.join(BASE_DIR, "orcabot.urdf")
URDF_TMP = os.path.join(BASE_DIR, "orcabot_mjcf_input.urdf")
MJCF_OUT = os.path.join(BASE_DIR, "orcabot.xml")

tree = ET.parse(URDF_IN)
root = tree.getroot()

# Remove any prior <mujoco> directive and reinject ours
for m in root.findall("mujoco"):
    root.remove(m)
mj = ET.Element("mujoco")
compiler = ET.SubElement(mj, "compiler")
compiler.set("strippath", "false")
compiler.set("discardvisual", "false")
compiler.set("balanceinertia", "true")
compiler.set("fusestatic", "false")
root.insert(0, mj)

# MuJoCo's URDF parser doesn't understand file:// — strip the scheme.
# MuJoCo also can't load .dae; convert any DAE to STL via trimesh once,
# caching the result next to the original.
import trimesh
DAE_CACHE = os.path.join(BASE_DIR, "mesh_cache", "dae2stl")
os.makedirs(DAE_CACHE, exist_ok=True)

def dae_to_stl(src_path):
    base = os.path.basename(src_path).rsplit(".", 1)[0]
    # Disambiguate by directory hash so different dae files with the same
    # basename don't collide.
    dir_hash = str(abs(hash(os.path.dirname(src_path))))[:8]
    dst = os.path.join(DAE_CACHE, f"{base}_{dir_hash}.stl")
    if not os.path.exists(dst):
        mesh = trimesh.load(src_path, force="mesh")
        mesh.export(dst)
    return dst

for mesh in root.iter("mesh"):
    fn = mesh.get("filename", "")
    if fn.startswith("file://"):
        fn = fn[len("file://"):]
    if fn.lower().endswith(".dae"):
        fn = dae_to_stl(fn)
    mesh.set("filename", fn)

ET.indent(tree, space="  ")
tree.write(URDF_TMP, xml_declaration=True, encoding="unicode")

print(f"[1/2] Compiling {URDF_TMP} with MuJoCo...")
model = mujoco.MjModel.from_xml_path(URDF_TMP)
print(f"  nq={model.nq} nv={model.nv} nbody={model.nbody} njnt={model.njnt}")

print(f"[2/3] Saving MJCF to {MJCF_OUT}...")
mujoco.mj_saveLastXML(MJCF_OUT, model)

# ── Step 3: Post-process to add damping + position actuators ──────────────
# The URDF compiler emits no <default>, no <actuator>, and no joint damping,
# so the bare MJCF behaves like a free-falling pendulum chain. Inject:
#  - a <default> with two classes (arm_joint, hand_joint) carrying damping
#  - a <position> actuator per hinge joint, classed by name
# Tag each hinge joint with the appropriate class so the defaults take effect.
print(f"[3/3] Injecting damping + position actuators...")

mtree = ET.parse(MJCF_OUT)
mroot = mtree.getroot()

# Stiff enough to hold the arm against gravity at the default pose without
# visible sag or oscillation. Damping near critical for the dominant inertia.
ARM_KP, ARM_KV, ARM_DAMP = 2000.0, 100.0, 20.0
HAND_KP, HAND_KV, HAND_DAMP = 20.0, 1.0, 0.5

def joint_class(jname):
    return "arm_joint" if jname.startswith("openarm_") else "hand_joint"

# <default> block with two joint classes
default = ET.Element("default")
arm_class = ET.SubElement(default, "default")
arm_class.set("class", "arm_joint")
ET.SubElement(arm_class, "joint", {"damping": str(ARM_DAMP), "armature": "0.5"})
ET.SubElement(arm_class, "position", {
    "kp": str(ARM_KP), "kv": str(ARM_KV), "ctrlrange": "-3.14 3.14",
    "forcerange": "-200 200",
})
hand_class = ET.SubElement(default, "default")
hand_class.set("class", "hand_joint")
ET.SubElement(hand_class, "joint", {"damping": str(HAND_DAMP), "armature": "0.001"})
ET.SubElement(hand_class, "position", {
    "kp": str(HAND_KP), "kv": str(HAND_KV), "ctrlrange": "-2 2",
    "forcerange": "-5 5",
})
# Insert <default> right after <compiler>
insert_at = 1
for i, child in enumerate(mroot):
    if child.tag == "compiler":
        insert_at = i + 1
        break
mroot.insert(insert_at, default)

# Tag every hinge joint in the worldbody with its class
hinge_joint_names = []
for joint in mroot.iter("joint"):
    # skip joints inside <default> blocks
    parents_in_default = False
    # quick check: only tag joints that have a "name" attribute
    name = joint.get("name")
    if not name:
        continue
    cls = joint_class(name)
    joint.set("class", cls)
    hinge_joint_names.append(name)

# Build <actuator> block referencing each joint
actuator = ET.Element("actuator")
for name in hinge_joint_names:
    cls = joint_class(name)
    pos = ET.SubElement(actuator, "position")
    pos.set("class", cls)
    pos.set("name", f"act_{name}")
    pos.set("joint", name)
mroot.append(actuator)

ET.indent(mtree, space="  ")
mtree.write(MJCF_OUT, xml_declaration=False, encoding="unicode")

# Validate the patched MJCF reloads cleanly and stepping is stable
m2 = mujoco.MjModel.from_xml_path(MJCF_OUT)
d2 = mujoco.MjData(m2)
import numpy as np
for _ in range(500):
    mujoco.mj_step(m2, d2)
assert np.isfinite(d2.qpos).all(), "qpos went non-finite during step test"
print(f"  patched MJCF: nq={m2.nq} nu={m2.nu} actuators wired, 500 steps stable.")
print("Done.")

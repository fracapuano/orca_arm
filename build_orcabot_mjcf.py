#!/usr/bin/env python3
"""
Convert orca_arm/orcabot.urdf -> orca_arm/orcabot.xml (MJCF) via MuJoCo's
URDF compiler.

The input URDF references meshes by paths relative to itself
(e.g. "assets/foo.stl"). MuJoCo's URDF parser resolves those against the
URDF's parent directory when strippath="false". DAE meshes are converted
to STL on the fly (MuJoCo can't load DAE) and the converted files are
written into the same orca_arm/assets/ tree so they get shipped with the
package. The final MJCF also references meshes via relative paths.
"""
import os
import xml.etree.ElementTree as ET
import mujoco
import trimesh
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.join(BASE_DIR, "orca_arm")
ASSETS_DIR = os.path.join(PACKAGE_DIR, "assets")
URDF_IN = os.path.join(PACKAGE_DIR, "orcabot.urdf")
URDF_TMP = os.path.join(PACKAGE_DIR, "_orcabot_mjcf_input.urdf")
MJCF_OUT = os.path.join(PACKAGE_DIR, "orcabot.xml")

tree = ET.parse(URDF_IN)
root = tree.getroot()

# Inject <mujoco> compiler directive: keep mesh paths as written so the
# relative "assets/..." references survive into the saved MJCF.
for m in root.findall("mujoco"):
    root.remove(m)
mj = ET.Element("mujoco")
compiler = ET.SubElement(mj, "compiler")
compiler.set("strippath", "false")
compiler.set("discardvisual", "false")
compiler.set("balanceinertia", "true")
compiler.set("fusestatic", "false")
root.insert(0, mj)

# MuJoCo can't load DAE; convert any DAE referenced in the URDF to STL
# and rewrite the URDF reference. Converted STLs land in orca_arm/assets/
# so they ship with the package alongside the URDF.
def dae_to_stl_relative(rel_path):
    """rel_path is relative to the URDF location (orca_arm/). Convert the
    DAE there to a sibling STL and return the new relative path."""
    src_abs = os.path.join(PACKAGE_DIR, rel_path)
    base = os.path.basename(rel_path).rsplit(".", 1)[0]
    new_name = f"{base}.stl"
    new_rel = os.path.join(os.path.dirname(rel_path), new_name)
    new_abs = os.path.join(PACKAGE_DIR, new_rel)
    if os.path.abspath(new_abs) == os.path.abspath(src_abs):
        new_rel = os.path.join(os.path.dirname(rel_path), f"{base}_from_dae.stl")
        new_abs = os.path.join(PACKAGE_DIR, new_rel)
    if not os.path.exists(new_abs):
        mesh = trimesh.load(src_abs, force="mesh")
        mesh.export(new_abs)
    return new_rel

for mesh in root.iter("mesh"):
    fn = mesh.get("filename", "")
    if fn.lower().endswith(".dae"):
        mesh.set("filename", dae_to_stl_relative(fn))

ET.indent(tree, space="  ")
tree.write(URDF_TMP, xml_declaration=True, encoding="unicode")

print(f"[1/3] Compiling {URDF_TMP} with MuJoCo...")
model = mujoco.MjModel.from_xml_path(URDF_TMP)
print(f"  nq={model.nq} nv={model.nv} nbody={model.nbody} njnt={model.njnt}")

print(f"[2/3] Saving MJCF to {MJCF_OUT}...")
mujoco.mj_saveLastXML(MJCF_OUT, model)
try:
    os.remove(URDF_TMP)
except OSError:
    pass

# ── Step 3: Post-process: relativize mesh paths + add damping/actuators ──
print(f"[3/3] Injecting damping + position actuators (and relativizing mesh paths)...")

mtree = ET.parse(MJCF_OUT)
mroot = mtree.getroot()

# mj_saveLastXML may emit absolute mesh paths even when the input was
# relative; rewrite them back to be relative to the MJCF location.
for mesh in mroot.iter("mesh"):
    fn = mesh.get("file", "")
    if fn and os.path.isabs(fn):
        try:
            mesh.set("file", os.path.relpath(fn, PACKAGE_DIR))
        except ValueError:
            pass

ARM_KP, ARM_KV, ARM_DAMP = 2000.0, 100.0, 20.0
HAND_KP, HAND_KV, HAND_DAMP = 20.0, 1.0, 0.5

def joint_class(jname):
    return "arm_joint" if jname.startswith("openarm_") else "hand_joint"

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
insert_at = 1
for i, child in enumerate(mroot):
    if child.tag == "compiler":
        insert_at = i + 1
        break
mroot.insert(insert_at, default)

hinge_joint_names = []
for joint in mroot.iter("joint"):
    name = joint.get("name")
    if not name:
        continue
    cls = joint_class(name)
    joint.set("class", cls)
    hinge_joint_names.append(name)

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

m2 = mujoco.MjModel.from_xml_path(MJCF_OUT)
d2 = mujoco.MjData(m2)
for _ in range(500):
    mujoco.mj_step(m2, d2)
assert np.isfinite(d2.qpos).all(), "qpos went non-finite during step test"
print(f"  patched MJCF: nq={m2.nq} nu={m2.nu} actuators wired, 500 steps stable.")
print("Done.")

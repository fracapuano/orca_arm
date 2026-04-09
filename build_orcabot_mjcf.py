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

print(f"[2/2] Saving MJCF to {MJCF_OUT}...")
mujoco.mj_saveLastXML(MJCF_OUT, model)
print("Done.")

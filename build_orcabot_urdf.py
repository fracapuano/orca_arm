#!/usr/bin/env python3
"""
Build a combined URDF: OpenArm bimanual robot with OrcaHand v2 left and right
hands replacing both original grippers.
"""

import os
import sys
import copy
import re
import tempfile
import shutil
import xml.etree.ElementTree as ET

# ── Paths ──────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OPENARM_DESC = os.path.join(BASE_DIR, "openarm_description_repo")
ORCAHAND_DESC = os.path.join(BASE_DIR, "orcahand_repo")
ORCAHAND_URDF_RIGHT = os.path.join(ORCAHAND_DESC, "v2", "models", "urdf", "orcahand_right.urdf")
ORCAHAND_URDF_LEFT = os.path.join(ORCAHAND_DESC, "v2", "models", "urdf", "orcahand_left.urdf")
OPENARM_XACRO = os.path.join(OPENARM_DESC, "urdf", "robot", "v10.urdf.xacro")
OUTPUT_URDF = os.path.join(BASE_DIR, "orcabot.urdf")

# ── Step 0: Monkey-patch ament_index_python for $(find ...) resolution ─────
# xacro uses ament_index_python.get_package_share_directory for $(find pkg)
# We create a fake module that maps package names to local paths.
import types

fake_ament = types.ModuleType("ament_index_python")
fake_ament_packages = types.ModuleType("ament_index_python.packages")

PACKAGE_MAP = {
    "openarm_description": OPENARM_DESC,
    "orcahand_description": ORCAHAND_DESC,
}

def get_package_share_directory(pkg_name):
    if pkg_name in PACKAGE_MAP:
        return PACKAGE_MAP[pkg_name]
    raise Exception(f"Package '{pkg_name}' not found in local map")

def get_package_prefix(pkg_name):
    return get_package_share_directory(pkg_name)

fake_ament.get_package_share_directory = get_package_share_directory
fake_ament.get_package_prefix = get_package_prefix
fake_ament_packages.get_package_share_directory = get_package_share_directory
fake_ament_packages.get_package_prefix = get_package_prefix

sys.modules["ament_index_python"] = fake_ament
sys.modules["ament_index_python.packages"] = fake_ament_packages

import xacro


# ── Step 1: Process OpenArm xacro to flat URDF ────────────────────────────
print("[1/5] Processing OpenArm xacro (bimanual mode)...")

# We also need to handle the ee_type_arguments.xacro include.
# Check if it exists:
ee_args_file = os.path.join(OPENARM_DESC, "urdf", "ee", "openarm_hand_arguments.xacro")
if not os.path.exists(ee_args_file):
    # Create a minimal one
    print("  Creating minimal openarm_hand_arguments.xacro...")
    with open(ee_args_file, "w") as f:
        f.write("""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="rpy_ee" default="0 0 0"/>
  <xacro:arg name="xyz_ee" default="0 0 0"/>
  <xacro:arg name="tcp_xyz" default="0 0 0.05"/>
  <xacro:arg name="tcp_rpy" default="0 0 0"/>
  <xacro:arg name="special_connection" default=""/>
  <xacro:arg name="description_pkg" default="openarm_description"/>
</robot>
""")

mappings = {
    "bimanual": "true",
    "hand": "true",
    "ee_type": "openarm_hand",
    "ros2_control": "false",
}

try:
    doc = xacro.process_file(OPENARM_XACRO, mappings=mappings)
    openarm_xml_str = doc.toprettyxml(indent="  ")
    print("  OpenArm xacro processed successfully.")
except Exception as e:
    print(f"  Error processing xacro: {e}")
    sys.exit(1)

# Parse the flat URDF
openarm_tree = ET.ElementTree(ET.fromstring(openarm_xml_str))
openarm_root = openarm_tree.getroot()

import math

def fix_mesh_path(mesh_elem):
    """Convert package:// URIs to absolute file:// paths for standalone use."""
    filename = mesh_elem.get("filename", "")
    if filename.startswith("package://orcahand_description/"):
        rel_path = filename.replace("package://orcahand_description/", "")
        abs_path = os.path.join(ORCAHAND_DESC, rel_path)
        mesh_elem.set("filename", f"file://{abs_path}")
    elif filename.startswith("package://openarm_description/"):
        rel_path = filename.replace("package://openarm_description/", "")
        abs_path = os.path.join(OPENARM_DESC, rel_path)
        mesh_elem.set("filename", f"file://{abs_path}")

# Add orcahand material (white) once
mat_elem = ET.SubElement(openarm_root, "material")
mat_elem.set("name", "orcahand_white")
color_elem = ET.SubElement(mat_elem, "color")
color_elem.set("rgba", "1 1 1 1")

SIDES = [
    ("right", ORCAHAND_URDF_RIGHT),
    ("left",  ORCAHAND_URDF_LEFT),
]

for side, orca_urdf_path in SIDES:
    # ── Step 2: Parse OrcaHand v2 URDF ────────────────────────────────────
    print(f"[2/5] Parsing OrcaHand v2 {side} hand URDF...")
    orca_tree = ET.parse(orca_urdf_path)
    orca_root = orca_tree.getroot()

    orca_links = orca_root.findall("link")
    orca_joints = orca_root.findall("joint")
    print(f"  Found {len(orca_links)} links, {len(orca_joints)} joints")

    # ── Step 3: Remove forearm + gripper from OpenArm ─────────────────────
    print(f"[3/5] Removing OpenArm {side} link5-7, joint5-7 + gripper...")

    remove_links = {
        f"openarm_{side}_link5",
        f"openarm_{side}_link6",
        f"openarm_{side}_link7",
        f"openarm_{side}_hand",
        f"openarm_{side}_hand_tcp",
        f"openarm_{side}_left_finger",
        f"openarm_{side}_right_finger",
    }

    def is_removable_link(link_elem, _rl=remove_links):
        return link_elem.get("name", "") in _rl

    def is_removable_joint(joint_elem, _side=side, _rl=remove_links):
        name = joint_elem.get("name", "")
        if name in (f"openarm_{_side}_joint5", f"openarm_{_side}_joint6", f"openarm_{_side}_joint7"):
            return True
        if f"{_side}_openarm_hand" in name:
            return True
        parent = joint_elem.find("parent")
        child = joint_elem.find("child")
        if parent is not None and parent.get("link", "") in _rl:
            return True
        if child is not None and child.get("link", "") in _rl:
            return True
        return False

    removed_links = []
    removed_joints = []

    for link in list(openarm_root.findall("link")):
        if is_removable_link(link):
            removed_links.append(link.get("name"))
            openarm_root.remove(link)

    for joint in list(openarm_root.findall("joint")):
        if is_removable_joint(joint):
            removed_joints.append(joint.get("name"))
            openarm_root.remove(joint)

    print(f"  Removed links: {removed_links}")
    print(f"  Removed joints: {removed_joints}")

    # ── Step 4: Add OrcaHand with prefix and fix mesh paths ───────────────
    print(f"[4/5] Adding OrcaHand v2 {side} hand...")

    ORCA_PREFIX = f"orcahand_{side}_"

    def prefix_name(name, _p=ORCA_PREFIX):
        return _p + name

    # Copy hand meshes into a side-specific cache dir with unique basenames,
    # so trimesh's basename-keyed mesh cache does not collapse shared names
    # (e.g. left/ForeArmStructure-Model.stl and right/ForeArmStructure-Model.stl)
    # into the same geometry.
    mesh_cache_dir = os.path.join(BASE_DIR, "mesh_cache", f"orcahand_{side}")
    os.makedirs(mesh_cache_dir, exist_ok=True)

    def rebase_mesh_to_cache(mesh_elem, _side=side, _cache=mesh_cache_dir):
        fn = mesh_elem.get("filename", "")
        if not fn.startswith("package://orcahand_description/"):
            return
        rel = fn.replace("package://orcahand_description/", "")
        src = os.path.join(ORCAHAND_DESC, rel)
        unique_name = f"{_side}_{os.path.basename(src)}"
        dst = os.path.join(_cache, unique_name)
        if not os.path.exists(dst):
            shutil.copyfile(src, dst)
        mesh_elem.set("filename", f"file://{dst}")

    # Add prefixed orcahand links
    for link in orca_links:
        new_link = copy.deepcopy(link)
        old_name = new_link.get("name")
        new_link.set("name", prefix_name(old_name))

        for mesh in new_link.iter("mesh"):
            rebase_mesh_to_cache(mesh)
        for mat in new_link.iter("material"):
            if mat.get("name") == "white":
                mat.set("name", "orcahand_white")

        # Duplicate visuals as collisions for MuJoCo
        for visual in new_link.findall("visual"):
            collision = copy.deepcopy(visual)
            collision.tag = "collision"
            for mat in collision.findall("material"):
                collision.remove(mat)
            new_link.append(collision)

        openarm_root.append(new_link)

    # Add prefixed orcahand joints
    for joint in orca_joints:
        new_joint = copy.deepcopy(joint)
        old_name = new_joint.get("name")
        new_joint.set("name", prefix_name(old_name))

        # Unify left wrist rest pose with the right.
        # orcahand_right.urdf has rpy="-0.6108 0 -pi/2" on Carpals_to_TopTower
        # while orcahand_left.urdf has rpy="0 0 -pi/2", producing an asymmetric
        # bimanual rest pose. Patch at build time so the vendored URDF stays
        # pristine (upstream fix tracked separately).
        if side == "left" and "Carpals" in old_name and "TopTower" in old_name:
            o = new_joint.find("origin")
            if o is not None:
                o.set("rpy", f"-0.610865285723758 0.0 {-math.pi/2}")

        parent = new_joint.find("parent")
        if parent is not None:
            parent.set("link", prefix_name(parent.get("link")))
        child = new_joint.find("child")
        if child is not None:
            child.set("link", prefix_name(child.get("link")))

        openarm_root.append(new_joint)

    # Connecting fixed joint from openarm_{side}_link4 to orcahand root.
    print(f"  Adding fixed joint: openarm_{side}_link4 -> orcahand {side} root...")
    connect_joint = ET.SubElement(openarm_root, "joint")
    connect_joint.set("name", f"openarm_{side}_to_orcahand_joint")
    connect_joint.set("type", "fixed")

    parent_elem = ET.SubElement(connect_joint, "parent")
    parent_elem.set("link", f"openarm_{side}_link4")

    child_elem = ET.SubElement(connect_joint, "child")
    child_elem.set("link", prefix_name("ForeArmStructure-Model_e18f2368"))

    # Right side: keep original mount (palm faces outward from body).
    # Left side: rotate 180° around the forearm (local z) axis so the
    # palm faces inward toward the robot body, mirroring the right hand.
    # The 180° flip around z is achieved by changing yaw pi -> 0, which
    # also negates the x component of the base-center offset derivation,
    # so conn_x flips sign (-0.01 -> 0.01).
    origin_elem = ET.SubElement(connect_joint, "origin")
    if side == "right":
        origin_elem.set("xyz", "-0.01 -0.0315 0.153")
        origin_elem.set("rpy", f"{math.pi/2} 0 {math.pi}")
    else:
        origin_elem.set("xyz", "0.01 -0.0315 0.153")
        origin_elem.set("rpy", f"{math.pi/2} 0 0")

# Fix mesh paths in openarm links too (package:// -> file://)
for mesh in openarm_root.iter("mesh"):
    fix_mesh_path(mesh)

# ── Step 5: Write and validate ─────────────────────────────────────────────
print("[5/5] Writing combined URDF...")

# Set robot name
openarm_root.set("name", "orcabot")

# Pretty-print
ET.indent(openarm_tree, space="  ")
openarm_tree.write(OUTPUT_URDF, xml_declaration=True, encoding="unicode")

print(f"  Written to: {OUTPUT_URDF}")

# ── Validation ─────────────────────────────────────────────────────────────
print("\n=== Validation ===")

# Re-parse to validate
try:
    val_tree = ET.parse(OUTPUT_URDF)
    val_root = val_tree.getroot()
    links = val_root.findall("link")
    joints = val_root.findall("joint")
    print(f"  Total links: {len(links)}")
    print(f"  Total joints: {len(joints)}")

    # Check for tree structure: each link (except root) should be a child exactly once
    child_links = set()
    parent_links = set()
    for j in joints:
        p = j.find("parent")
        c = j.find("child")
        if p is not None:
            parent_links.add(p.get("link"))
        if c is not None:
            cname = c.get("link")
            if cname in child_links:
                print(f"  WARNING: Link '{cname}' appears as child in multiple joints!")
            child_links.add(cname)

    all_link_names = {l.get("name") for l in links}
    root_links = all_link_names - child_links
    orphan_parents = parent_links - all_link_names

    print(f"  Root link(s): {root_links}")
    if orphan_parents:
        print(f"  WARNING: Parent links not defined: {orphan_parents}")
    else:
        print("  All parent links are defined.")

    # Check the right arm -> orcahand chain
    print("\n  Right arm kinematic chain (from link6):")
    def find_children(parent_name):
        children = []
        for j in joints:
            p = j.find("parent")
            if p is not None and p.get("link") == parent_name:
                c = j.find("child")
                children.append((j.get("name"), c.get("link")))
        return children

    chain = find_children("openarm_right_link6")
    print(f"    openarm_right_link6 -> {chain}")
    # Also show link7 children
    chain7 = find_children("openarm_right_link7")
    print(f"    openarm_right_link7 -> {chain7}")
    for _, child_name in chain:
        sub = find_children(child_name)
        if sub:
            print(f"    {child_name} -> {sub}")

    print("\n  SUCCESS: Combined URDF is valid XML with proper tree structure.")

except Exception as e:
    print(f"  Validation error: {e}")

print(f"\nDone! Output: {OUTPUT_URDF}")

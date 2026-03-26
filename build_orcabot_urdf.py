#!/usr/bin/env python3
"""
Build a combined URDF: OpenArm bimanual robot with OrcaHand v2 right hand
replacing the original right gripper.
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
ORCAHAND_URDF = os.path.join(ORCAHAND_DESC, "v2", "models", "urdf", "orcahand_right.urdf")
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

# ── Step 2: Parse OrcaHand v2 right URDF ──────────────────────────────────
print("[2/5] Parsing OrcaHand v2 right hand URDF...")
orca_tree = ET.parse(ORCAHAND_URDF)
orca_root = orca_tree.getroot()

# Collect all orcahand links and joints
orca_links = orca_root.findall("link")
orca_joints = orca_root.findall("joint")
orca_materials = orca_root.findall("material")

print(f"  Found {len(orca_links)} links, {len(orca_joints)} joints")

# ── Step 3: Remove right forearm + gripper from OpenArm ────────────────────
print("[3/5] Removing OpenArm right link5-7, joint5-7 + gripper...")

# Attach the orcahand at link4 (elbow). Remove link5-7, joint5-7, and gripper.
# The orcahand's ForeArmStructure replaces the entire forearm from the elbow down.
right_remove_links = {
    "openarm_right_link5",
    "openarm_right_link6",
    "openarm_right_link7",
    "openarm_right_hand",
    "openarm_right_hand_tcp",
    "openarm_right_left_finger",
    "openarm_right_right_finger",
}

def is_right_removable_link(link_elem):
    return link_elem.get("name", "") in right_remove_links

def is_right_removable_joint(joint_elem):
    name = joint_elem.get("name", "")
    # Remove joint5 (link4 -> link5), joint6, joint7
    if name in ("openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7"):
        return True
    if "right_openarm_hand" in name:
        return True
    # Check if parent or child is a removable link
    parent = joint_elem.find("parent")
    child = joint_elem.find("child")
    if parent is not None and parent.get("link", "") in right_remove_links:
        return True
    if child is not None and child.get("link", "") in right_remove_links:
        return True
    return False

removed_links = []
removed_joints = []

for link in list(openarm_root.findall("link")):
    if is_right_removable_link(link):
        removed_links.append(link.get("name"))
        openarm_root.remove(link)

for joint in list(openarm_root.findall("joint")):
    if is_right_removable_joint(joint):
        removed_joints.append(joint.get("name"))
        openarm_root.remove(joint)

print(f"  Removed links: {removed_links}")
print(f"  Removed joints: {removed_joints}")

# ── Step 4: Add OrcaHand with prefix and fix mesh paths ───────────────────
print("[4/5] Adding OrcaHand v2 right hand...")

ORCA_PREFIX = "orcahand_right_"
ORCA_MESH_BASE = os.path.join(ORCAHAND_DESC, "v2", "models", "assets", "right")
OPENARM_MESH_BASE = OPENARM_DESC

def prefix_name(name):
    return ORCA_PREFIX + name

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

# Add orcahand material (white) if not already present
has_white_material = False
for mat in openarm_root.findall("material"):
    if mat.get("name") == "orcahand_white":
        has_white_material = True
        break

if not has_white_material:
    mat_elem = ET.SubElement(openarm_root, "material")
    mat_elem.set("name", "orcahand_white")
    color_elem = ET.SubElement(mat_elem, "color")
    color_elem.set("rgba", "1 1 1 1")

# Add prefixed orcahand links
for link in orca_links:
    new_link = copy.deepcopy(link)
    old_name = new_link.get("name")
    new_link.set("name", prefix_name(old_name))

    # Fix mesh paths and material references
    for mesh in new_link.iter("mesh"):
        fix_mesh_path(mesh)
    for mat in new_link.iter("material"):
        if mat.get("name") == "white":
            mat.set("name", "orcahand_white")

    # The orcahand URDF only has <visual> geometry, no <collision>.
    # MuJoCo needs collision geometry to render. Duplicate each visual
    # as a collision element so the hand appears in MuJoCo.
    for visual in new_link.findall("visual"):
        collision = copy.deepcopy(visual)
        collision.tag = "collision"
        # Remove material from collision (not needed)
        for mat in collision.findall("material"):
            collision.remove(mat)
        new_link.append(collision)

    openarm_root.append(new_link)

# Add prefixed orcahand joints
for joint in orca_joints:
    new_joint = copy.deepcopy(joint)
    old_name = new_joint.get("name")
    new_joint.set("name", prefix_name(old_name))

    # Fix parent/child link references
    parent = new_joint.find("parent")
    if parent is not None:
        parent.set("link", prefix_name(parent.get("link")))
    child = new_joint.find("child")
    if child is not None:
        child.set("link", prefix_name(child.get("link")))

    openarm_root.append(new_joint)

# Add the connecting fixed joint from openarm_right_link4 to orcahand root.
# The orcahand's ForeArmStructure replaces the entire forearm from the elbow.
# Orientation: the orcahand forearm extends along its local +y axis
# (TopTower is at y=0.052 from ForeArmStructure). The openarm extends
# along +z. We rotate to align: rpy=(pi/2, 0, pi) maps orca +y -> arm +z.
import math
print("  Adding fixed joint: openarm_right_link4 -> orcahand root...")
connect_joint = ET.SubElement(openarm_root, "joint")
connect_joint.set("name", "openarm_right_to_orcahand_joint")
connect_joint.set("type", "fixed")

parent_elem = ET.SubElement(connect_joint, "parent")
parent_elem.set("link", "openarm_right_link4")

child_elem = ET.SubElement(connect_joint, "child")
child_elem.set("link", prefix_name("ForeArmStructure-Model_e18f2368"))

# Place the orcahand so its base center sits at the tip of link4's arm axis.
#
# After rotation rpy=(pi/2,0,pi), the orca base center (local -0.01, -0.058, 0)
# maps to (+0.01, 0, -0.058) in link4's frame relative to the connection point.
#
# To center the base on the arm axis (x=0, y=0) at link4's tip (z=0.095):
#   conn_x + 0.01 = 0    =>  conn_x = -0.01
#   conn_y + 0    = 0     =>  conn_y = 0
#   conn_z - 0.058 = 0.095 => conn_z = 0.153
origin_elem = ET.SubElement(connect_joint, "origin")
origin_elem.set("xyz", "-0.01 -0.0315 0.153")
origin_elem.set("rpy", f"{math.pi/2} 0 {math.pi}")

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

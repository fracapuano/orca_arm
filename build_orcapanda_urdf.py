#!/usr/bin/env python3
"""
Build a combined URDF: a mono Franka Emika Panda arm with one right OrcaHand v2
mounted at the Panda flange.
"""

import copy
import hashlib
import math
import os
import shutil
import sys
import types
import xml.etree.ElementTree as ET


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FRANKA_DESC = os.path.join(BASE_DIR, "franka_ros_repo", "franka_description")
ORCAHAND_DESC = os.path.join(BASE_DIR, "orcahand_repo")
FRANKA_XACRO = os.path.join(FRANKA_DESC, "robots", "panda", "panda.urdf.xacro")
ORCAHAND_URDF_RIGHT = os.path.join(
    ORCAHAND_DESC, "v2", "models", "urdf", "orcahand_right.urdf"
)
PACKAGE_DIR = os.path.join(BASE_DIR, "orca_arm")
ASSETS_DIR = os.path.join(PACKAGE_DIR, "assets")
OUTPUT_URDF = os.path.join(PACKAGE_DIR, "orcapanda.urdf")


fake_ament = types.ModuleType("ament_index_python")
fake_ament_packages = types.ModuleType("ament_index_python.packages")

PACKAGE_MAP = {
    "franka_description": FRANKA_DESC,
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


os.makedirs(ASSETS_DIR, exist_ok=True)


def stage_mesh(src_abs_path, name_hint=None):
    """Copy src into assets/ with a unique flat name and return a URDF-relative path."""
    base = os.path.basename(src_abs_path)
    if name_hint:
        name = f"{name_hint}_{base}"
    else:
        h = hashlib.md5(src_abs_path.encode()).hexdigest()[:8]
        name = f"{h}_{base}"
    dst = os.path.join(ASSETS_DIR, name)
    if not os.path.exists(dst):
        shutil.copyfile(src_abs_path, dst)
    return f"assets/{name}"


def stage_franka_mesh(mesh_elem):
    filename = mesh_elem.get("filename", "")
    if not filename.startswith("package://franka_description/"):
        return
    rel_path = filename.replace("package://franka_description/", "")
    src = os.path.join(FRANKA_DESC, rel_path)
    if "/visual/" in rel_path:
        hint = "panda_visual"
    elif "/collision/" in rel_path:
        hint = "panda_collision"
    else:
        hint = "panda"
    mesh_elem.set("filename", stage_mesh(src, name_hint=hint))


def stage_orcahand_mesh(mesh_elem):
    filename = mesh_elem.get("filename", "")
    if not filename.startswith("package://orcahand_description/"):
        return
    rel_path = filename.replace("package://orcahand_description/", "")
    src = os.path.join(ORCAHAND_DESC, rel_path)
    mesh_elem.set("filename", stage_mesh(src, name_hint="right"))


def add_world_root(root):
    if root.find("./link[@name='world']") is not None:
        return
    world = ET.Element("link")
    world.set("name", "world")
    root.insert(0, world)

    joint = ET.Element("joint")
    joint.set("name", "world_to_panda_joint")
    joint.set("type", "fixed")
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": "panda_link0"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    root.insert(1, joint)


def prefix_name(name, prefix="orcahand_right_"):
    return prefix + name


print("[1/5] Processing Franka Panda xacro...")
try:
    doc = xacro.process_file(
        FRANKA_XACRO,
        mappings={
            "arm_id": "panda",
            "hand": "false",
            "gazebo": "false",
        },
    )
    panda_xml_str = doc.toprettyxml(indent="  ")
    print("  Panda xacro processed successfully.")
except Exception as exc:
    print(f"  Error processing Panda xacro: {exc}")
    sys.exit(1)

panda_tree = ET.ElementTree(ET.fromstring(panda_xml_str))
panda_root = panda_tree.getroot()
add_world_root(panda_root)

mat_elem = ET.SubElement(panda_root, "material")
mat_elem.set("name", "orcahand_white")
color_elem = ET.SubElement(mat_elem, "color")
color_elem.set("rgba", "1 1 1 1")

print("[2/5] Parsing OrcaHand v2 right hand URDF...")
orca_tree = ET.parse(ORCAHAND_URDF_RIGHT)
orca_root = orca_tree.getroot()
orca_links = orca_root.findall("link")
orca_joints = orca_root.findall("joint")
print(f"  Found {len(orca_links)} links, {len(orca_joints)} joints.")

print("[3/5] Adding right OrcaHand links and joints...")
for link in orca_links:
    new_link = copy.deepcopy(link)
    new_link.set("name", prefix_name(new_link.get("name")))

    for mesh in new_link.iter("mesh"):
        stage_orcahand_mesh(mesh)
    for material in new_link.iter("material"):
        if material.get("name") == "white":
            material.set("name", "orcahand_white")

    for visual in new_link.findall("visual"):
        collision = copy.deepcopy(visual)
        collision.tag = "collision"
        for material in collision.findall("material"):
            collision.remove(material)
        new_link.append(collision)

    panda_root.append(new_link)

for joint in orca_joints:
    new_joint = copy.deepcopy(joint)
    new_joint.set("name", prefix_name(new_joint.get("name")))

    parent = new_joint.find("parent")
    if parent is not None:
        parent.set("link", prefix_name(parent.get("link")))
    child = new_joint.find("child")
    if child is not None:
        child.set("link", prefix_name(child.get("link")))

    panda_root.append(new_joint)

print("[4/5] Mounting OrcaHand to panda_link8 and staging Panda meshes...")
connect_joint = ET.SubElement(panda_root, "joint")
connect_joint.set("name", "panda_link8_to_orcahand_joint")
connect_joint.set("type", "fixed")
ET.SubElement(connect_joint, "parent", {"link": "panda_link8"})
ET.SubElement(
    connect_joint,
    "child",
    {"link": prefix_name("ForeArmStructure-Model_e18f2368")},
)
ET.SubElement(
    connect_joint,
    "origin",
    {
        "xyz": "0 0 0.0575",
        "rpy": f"{math.pi / 2} 0 {math.pi}",
    },
)

for mesh in panda_root.iter("mesh"):
    stage_franka_mesh(mesh)

print("[5/5] Writing combined URDF...")
panda_root.set("name", "orcapanda")
ET.indent(panda_tree, space="  ")
panda_tree.write(OUTPUT_URDF, xml_declaration=True, encoding="unicode")
print(f"  Written to: {OUTPUT_URDF}")

print("\n=== Validation ===")
try:
    val_root = ET.parse(OUTPUT_URDF).getroot()
    links = val_root.findall("link")
    joints = val_root.findall("joint")
    all_links = {link.get("name") for link in links}
    child_links = set()
    parent_links = set()
    duplicate_children = set()
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None:
            parent_links.add(parent.get("link"))
        if child is not None:
            child_name = child.get("link")
            if child_name in child_links:
                duplicate_children.add(child_name)
            child_links.add(child_name)

    root_links = all_links - child_links
    orphan_parents = parent_links - all_links
    print(f"  Total links: {len(links)}")
    print(f"  Total joints: {len(joints)}")
    print(f"  Root link(s): {root_links}")
    if duplicate_children:
        raise RuntimeError(f"Duplicate child links: {duplicate_children}")
    if orphan_parents:
        raise RuntimeError(f"Undefined parent links: {orphan_parents}")
    if root_links != {"world"}:
        raise RuntimeError(f"Expected only world as root, got {root_links}")
    print("  SUCCESS: Combined Panda-Orca URDF is a valid tree.")
except Exception as exc:
    print(f"  Validation error: {exc}")
    sys.exit(1)

print(f"\nDone! Output: {OUTPUT_URDF}")

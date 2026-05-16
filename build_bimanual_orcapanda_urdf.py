#!/usr/bin/env python3
"""
Build a bimanual Franka Panda + OrcaHand URDF.

The generated robot has two side-by-side Panda arms:

  world -> left_panda_link0  -> left_panda_link8  -> orcahand_left_*
  world -> right_panda_link0 -> right_panda_link8 -> orcahand_right_*
"""

import copy
import hashlib
import math
from pathlib import Path
import shutil
import sys
import types
import xml.etree.ElementTree as ET


BASE_DIR = Path(__file__).resolve().parent
FRANKA_DESC = BASE_DIR / "franka_ros_repo" / "franka_description"
ORCAHAND_DESC = BASE_DIR / "orcahand_repo"
FRANKA_XACRO = FRANKA_DESC / "robots" / "panda" / "panda.urdf.xacro"
PACKAGE_DIR = BASE_DIR / "orca_arm"
ASSETS_DIR = PACKAGE_DIR / "assets"
OUTPUT_URDF = PACKAGE_DIR / "bimanual_orcapanda.urdf"

BASE_Y_OFFSET = 0.45
SIDES = (
    {
        "side": "left",
        "panda_prefix": "left_panda",
        "base_xyz": f"0 {BASE_Y_OFFSET} 0",
        "hand_rpy": f"{math.pi / 2} 0 0",
    },
    {
        "side": "right",
        "panda_prefix": "right_panda",
        "base_xyz": f"0 {-BASE_Y_OFFSET} 0",
        "hand_rpy": f"{math.pi / 2} 0 {math.pi}",
    },
)

PACKAGE_MAP = {
    "franka_description": str(FRANKA_DESC),
    "orcahand_description": str(ORCAHAND_DESC),
}


def get_package_share_directory(pkg_name: str) -> str:
    if pkg_name in PACKAGE_MAP:
        return PACKAGE_MAP[pkg_name]
    raise Exception(f"Package '{pkg_name}' not found in local map")


def get_package_prefix(pkg_name: str) -> str:
    return get_package_share_directory(pkg_name)


def install_fake_ament() -> None:
    fake_ament = types.ModuleType("ament_index_python")
    fake_ament_packages = types.ModuleType("ament_index_python.packages")
    fake_ament.get_package_share_directory = get_package_share_directory
    fake_ament.get_package_prefix = get_package_prefix
    fake_ament_packages.get_package_share_directory = get_package_share_directory
    fake_ament_packages.get_package_prefix = get_package_prefix
    sys.modules["ament_index_python"] = fake_ament
    sys.modules["ament_index_python.packages"] = fake_ament_packages


def stage_mesh(src_abs_path: str | Path, name_hint: str | None = None) -> str:
    """Copy src into assets/ with a stable flat name and return a URDF-relative path."""
    src_abs_path = Path(src_abs_path)
    base = src_abs_path.name
    if name_hint:
        name = f"{name_hint}_{base}"
    else:
        h = hashlib.md5(str(src_abs_path).encode()).hexdigest()[:8]
        name = f"{h}_{base}"
    dst = ASSETS_DIR / name
    if not dst.exists():
        shutil.copyfile(src_abs_path, dst)
    return f"assets/{name}"


def stage_franka_mesh(mesh_elem: ET.Element) -> None:
    filename = mesh_elem.get("filename", "")
    if not filename.startswith("package://franka_description/"):
        return
    rel_path = filename.replace("package://franka_description/", "")
    src = FRANKA_DESC / rel_path
    if "/visual/" in rel_path:
        hint = "panda_visual"
    elif "/collision/" in rel_path:
        hint = "panda_collision"
    else:
        hint = "panda"
    mesh_elem.set("filename", stage_mesh(src, name_hint=hint))


def stage_orcahand_mesh(mesh_elem: ET.Element, side: str) -> None:
    filename = mesh_elem.get("filename", "")
    if not filename.startswith("package://orcahand_description/"):
        return
    rel_path = filename.replace("package://orcahand_description/", "")
    src = ORCAHAND_DESC / rel_path
    mesh_elem.set("filename", stage_mesh(src, name_hint=side))


def process_panda_xacro(arm_id: str) -> ET.Element:
    import xacro

    doc = xacro.process_file(
        str(FRANKA_XACRO),
        mappings={
            "arm_id": arm_id,
            "hand": "false",
            "gazebo": "false",
        },
    )
    return ET.fromstring(doc.toprettyxml(indent="  "))


def append_unique_materials(dst_root: ET.Element, src_root: ET.Element) -> None:
    existing = {material.get("name") for material in dst_root.findall("material")}
    for material in src_root.findall("material"):
        name = material.get("name")
        if name in existing:
            continue
        dst_root.append(copy.deepcopy(material))
        existing.add(name)


def append_panda(root: ET.Element, panda_prefix: str, base_xyz: str) -> None:
    panda_root = process_panda_xacro(panda_prefix)
    append_unique_materials(root, panda_root)

    for link in panda_root.findall("link"):
        root.append(copy.deepcopy(link))
    for joint in panda_root.findall("joint"):
        root.append(copy.deepcopy(joint))

    base_joint = ET.SubElement(root, "joint")
    base_joint.set("name", f"world_to_{panda_prefix}_joint")
    base_joint.set("type", "fixed")
    ET.SubElement(base_joint, "parent", {"link": "world"})
    ET.SubElement(base_joint, "child", {"link": f"{panda_prefix}_link0"})
    ET.SubElement(base_joint, "origin", {"xyz": base_xyz, "rpy": "0 0 0"})


def append_orcahand(root: ET.Element, side: str, panda_prefix: str, hand_rpy: str) -> None:
    hand_path = ORCAHAND_DESC / "v2" / "models" / "urdf" / f"orcahand_{side}.urdf"
    hand_root = ET.parse(hand_path).getroot()
    hand_prefix = f"orcahand_{side}_"

    for link in hand_root.findall("link"):
        new_link = copy.deepcopy(link)
        new_link.set("name", f"{hand_prefix}{new_link.get('name')}")

        for mesh in new_link.iter("mesh"):
            stage_orcahand_mesh(mesh, side)
        for material in new_link.iter("material"):
            if material.get("name") == "white":
                material.set("name", "orcahand_white")

        for visual in new_link.findall("visual"):
            collision = copy.deepcopy(visual)
            collision.tag = "collision"
            for material in collision.findall("material"):
                collision.remove(material)
            new_link.append(collision)

        root.append(new_link)

    for joint in hand_root.findall("joint"):
        new_joint = copy.deepcopy(joint)
        old_name = new_joint.get("name", "")
        new_joint.set("name", f"{hand_prefix}{old_name}")

        if side == "left" and "Carpals" in old_name and "TopTower" in old_name:
            origin = new_joint.find("origin")
            if origin is not None:
                origin.set("rpy", f"-0.610865285723758 0.0 {-math.pi / 2}")

        parent = new_joint.find("parent")
        if parent is not None:
            parent.set("link", f"{hand_prefix}{parent.get('link')}")
        child = new_joint.find("child")
        if child is not None:
            child.set("link", f"{hand_prefix}{child.get('link')}")

        root.append(new_joint)

    mount = ET.SubElement(root, "joint")
    mount.set("name", f"{panda_prefix}_link8_to_orcahand_joint")
    mount.set("type", "fixed")
    ET.SubElement(mount, "parent", {"link": f"{panda_prefix}_link8"})
    ET.SubElement(mount, "child", {"link": f"{hand_prefix}ForeArmStructure-Model_e18f2368"})
    ET.SubElement(mount, "origin", {"xyz": "0 0 0.0575", "rpy": hand_rpy})


def validate_urdf_tree(path: Path) -> None:
    root = ET.parse(path).getroot()
    links = root.findall("link")
    joints = root.findall("joint")
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
    dynamic_joints = [joint for joint in joints if joint.get("type") != "fixed"]
    print(f"  Total links: {len(links)}")
    print(f"  Total joints: {len(joints)}")
    print(f"  Dynamic joints: {len(dynamic_joints)}")
    print(f"  Root link(s): {root_links}")
    if duplicate_children:
        raise RuntimeError(f"Duplicate child links: {duplicate_children}")
    if orphan_parents:
        raise RuntimeError(f"Undefined parent links: {orphan_parents}")
    if root_links != {"world"}:
        raise RuntimeError(f"Expected only world as root, got {root_links}")
    if len(dynamic_joints) != 48:
        raise RuntimeError(f"Expected 48 dynamic joints, got {len(dynamic_joints)}")
    print("  SUCCESS: BimanualOrcaPanda URDF is a valid tree.")


def main() -> None:
    install_fake_ament()
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)

    root = ET.Element("robot", {"name": "bimanual_orcapanda"})
    ET.SubElement(root, "link", {"name": "world"})
    material = ET.SubElement(root, "material", {"name": "orcahand_white"})
    ET.SubElement(material, "color", {"rgba": "1 1 1 1"})

    print("[1/4] Adding two Panda arms...")
    for spec in SIDES:
        print(f"  {spec['side']}: {spec['panda_prefix']} at {spec['base_xyz']}")
        append_panda(root, spec["panda_prefix"], spec["base_xyz"])

    print("[2/4] Adding left and right OrcaHands...")
    for spec in SIDES:
        append_orcahand(root, spec["side"], spec["panda_prefix"], spec["hand_rpy"])

    print("[3/4] Staging Panda meshes...")
    for mesh in root.iter("mesh"):
        stage_franka_mesh(mesh)

    print("[4/4] Writing combined URDF...")
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(OUTPUT_URDF, xml_declaration=True, encoding="unicode")
    print(f"  Written to: {OUTPUT_URDF}")

    print("\n=== Validation ===")
    validate_urdf_tree(OUTPUT_URDF)
    print(f"\nDone! Output: {OUTPUT_URDF}")


if __name__ == "__main__":
    main()

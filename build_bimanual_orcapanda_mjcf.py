#!/usr/bin/env python3
"""Build bimanual OrcaPanda MJCF from two Menagerie Panda arms and two OrcaHands."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh


BASE_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = BASE_DIR / "orca_arm"
PANDA_ASSETS_DIR = PACKAGE_DIR / "assets" / "franka_emika_panda"
PANDA_NOHAND_XML = PANDA_ASSETS_DIR / "panda_nohand.xml"
URDF_IN = PACKAGE_DIR / "bimanual_orcapanda.urdf"
URDF_TMP = PACKAGE_DIR / "_bimanual_orcapanda_mjcf_input.urdf"
URDF_COMPILED_TMP = PACKAGE_DIR / "_bimanual_orcapanda_urdf_compiled.xml"
MJCF_OUT = PACKAGE_DIR / "bimanual_orcapanda.xml"

SIDES = (
    {"side": "left", "panda_prefix": "left_panda", "base_pos": "0 0.45 0"},
    {"side": "right", "panda_prefix": "right_panda", "base_pos": "0 -0.45 0"},
)


def dae_to_stl_relative(rel_path: str) -> str:
    src_abs = PACKAGE_DIR / rel_path
    base = Path(rel_path).with_suffix(".stl")
    new_rel = str(base)
    new_abs = PACKAGE_DIR / new_rel
    if new_abs.resolve() == src_abs.resolve():
        base_path = Path(rel_path)
        new_rel = str(base_path.with_name(f"{base_path.stem}_from_dae.stl"))
        new_abs = PACKAGE_DIR / new_rel
    if not new_abs.exists():
        mesh = trimesh.load(src_abs, force="mesh")
        mesh.export(new_abs)
    return new_rel


def compile_urdf_for_hands() -> ET.Element:
    tree = ET.parse(URDF_IN)
    root = tree.getroot()

    for mujoco_tag in root.findall("mujoco"):
        root.remove(mujoco_tag)

    mujoco_tag = ET.Element("mujoco")
    compiler = ET.SubElement(mujoco_tag, "compiler")
    compiler.set("strippath", "false")
    compiler.set("discardvisual", "false")
    compiler.set("balanceinertia", "true")
    compiler.set("fusestatic", "false")
    root.insert(0, mujoco_tag)

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.lower().endswith(".dae"):
            mesh.set("filename", dae_to_stl_relative(filename))

    ET.indent(tree, space="  ")
    tree.write(URDF_TMP, xml_declaration=True, encoding="unicode")

    model = mujoco.MjModel.from_xml_path(str(URDF_TMP))
    mujoco.mj_saveLastXML(str(URDF_COMPILED_TMP), model)
    compiled_root = ET.parse(URDF_COMPILED_TMP).getroot()

    for path in (URDF_TMP, URDF_COMPILED_TMP):
        try:
            path.unlink()
        except OSError:
            pass

    return compiled_root


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    raise ValueError(f"Body not found: {name}")


def parse_pair(text: str | None) -> tuple[float, float] | None:
    if not text:
        return None
    values = [float(value) for value in text.split()]
    if len(values) != 2:
        return None
    return values[0], values[1]


def format_float(value: float) -> str:
    return f"{value:.12g}"


def rewrite_panda_asset_paths(root: ET.Element) -> None:
    for mesh in root.findall("./asset/mesh"):
        filename = mesh.get("file")
        if filename and "/" not in filename:
            mesh.set("file", f"assets/franka_emika_panda/{filename}")


def rename_panda_subtree(body: ET.Element, panda_prefix: str, base_pos: str) -> None:
    body.set("pos", base_pos)
    for child_body in body.iter("body"):
        name = child_body.get("name")
        if name is None:
            continue
        if name == "attachment":
            child_body.set("name", f"{panda_prefix}_link8")
            child_body.set("pos", "0 0 0.107")
            child_body.attrib.pop("quat", None)
        elif re.fullmatch(r"link[0-7]", name):
            child_body.set("name", f"{panda_prefix}_{name}")

    for joint in body.iter("joint"):
        name = joint.get("name")
        if name and re.fullmatch(r"joint[1-7]", name):
            joint.set("name", f"{panda_prefix}_{name}")

    for site in body.iter("site"):
        name = site.get("name")
        if name:
            site.set("name", f"{panda_prefix}_{name}")


def compose_panda_bodies(root: ET.Element, source_panda_body: ET.Element) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Panda source MJCF is missing <worldbody>")

    for child in list(worldbody):
        if child.tag == "body":
            worldbody.remove(child)

    for spec in SIDES:
        panda_body = copy.deepcopy(source_panda_body)
        rename_panda_subtree(panda_body, spec["panda_prefix"], spec["base_pos"])
        worldbody.append(panda_body)


def add_hand_defaults(root: ET.Element) -> None:
    default = root.find("default")
    if default is None:
        default = ET.Element("default")
        root.insert(1, default)

    hand_class = ET.SubElement(default, "default")
    hand_class.set("class", "hand_joint")
    ET.SubElement(hand_class, "joint", {"damping": "0.5", "armature": "0.001"})
    ET.SubElement(
        hand_class,
        "position",
        {
            "kp": "20.0",
            "kv": "1.0",
            "forcerange": "-5 5",
        },
    )


def append_hand_assets(root: ET.Element, compiled_urdf_root: ET.Element) -> None:
    dst_asset = root.find("asset")
    src_asset = compiled_urdf_root.find("asset")
    if dst_asset is None or src_asset is None:
        raise ValueError("Missing asset section while composing bimanual MJCF")

    existing_names = {mesh.get("name") for mesh in dst_asset.findall("mesh") if mesh.get("name")}
    for mesh in src_asset.findall("mesh"):
        name = mesh.get("name", "")
        filename = mesh.get("file", "")
        if name.startswith("panda_") or "panda_" in filename or name in existing_names:
            continue
        cloned = copy.deepcopy(mesh)
        if filename and os.path.isabs(filename):
            cloned.set("file", os.path.relpath(filename, PACKAGE_DIR))
        dst_asset.append(cloned)
        if name:
            existing_names.add(name)


def append_hand_bodies(root: ET.Element, compiled_urdf_root: ET.Element) -> None:
    for spec in SIDES:
        side = spec["side"]
        panda_link8 = find_body(root, f"{spec['panda_prefix']}_link8")
        hand_root = find_body(compiled_urdf_root, f"orcahand_{side}_ForeArmStructure-Model_e18f2368")
        panda_link8.append(copy.deepcopy(hand_root))


def joint_ranges(root: ET.Element) -> dict[str, tuple[float, float]]:
    ranges = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        pair = parse_pair(joint.get("range"))
        if name and pair is not None:
            ranges[name] = pair
    return ranges


def joints_in_tree_order(root: ET.Element) -> list[ET.Element]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        return []
    return [joint for joint in worldbody.iter("joint") if joint.get("name")]


def joint_home(joint: ET.Element) -> float:
    ref = joint.get("ref")
    if ref is not None:
        return float(ref)
    return 0.0


def set_home_refs(root: ET.Element) -> None:
    for joint in root.iter("joint"):
        pair = parse_pair(joint.get("range"))
        if pair is None:
            continue
        lo, hi = pair
        home = min(max(0.0, lo), hi)
        if home != 0.0:
            joint.set("ref", format_float(home))


def rebuild_contact_excludes(root: ET.Element) -> None:
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    for child in list(contact):
        contact.remove(child)
    for spec in SIDES:
        ET.SubElement(
            contact,
            "exclude",
            {
                "body1": f"{spec['panda_prefix']}_link0",
                "body2": f"{spec['panda_prefix']}_link1",
            },
        )


def rebuild_actuators(root: ET.Element, source_root: ET.Element) -> None:
    source_templates = {
        actuator.get("joint"): actuator
        for actuator in source_root.findall("./actuator/*")
        if actuator.get("joint")
    }
    ranges = joint_ranges(root)
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for child in list(actuator):
        actuator.remove(child)

    for joint in joints_in_tree_order(root):
        name = joint.get("name", "")
        new_actuator = None
        panda_match = re.fullmatch(r"(left_panda|right_panda)_joint([1-7])", name)
        if panda_match:
            source = source_templates[f"joint{panda_match.group(2)}"]
            new_actuator = copy.deepcopy(source)
            new_actuator.set("name", f"act_{name}")
            new_actuator.set("joint", name)
            actuator.append(new_actuator)
        elif name.startswith("orcahand_"):
            new_actuator = ET.SubElement(actuator, "position")
            new_actuator.set("class", "hand_joint")
            new_actuator.set("name", f"act_{name}")
            new_actuator.set("joint", name)

        if new_actuator is not None and name in ranges:
            lo, hi = ranges[name]
            new_actuator.set("ctrlrange", f"{format_float(lo)} {format_float(hi)}")


def patch_keyframes(root: ET.Element) -> None:
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")
    keys = keyframe.findall("key")
    if not keys:
        keys = [ET.SubElement(keyframe, "key", {"name": "home"})]

    joints = joints_in_tree_order(root)
    homes_by_name = {joint.get("name"): joint_home(joint) for joint in joints}
    qpos_values = [homes_by_name[joint.get("name")] for joint in joints]
    ctrl_values = [
        homes_by_name.get(actuator.get("joint"), 0.0)
        for actuator in root.findall("./actuator/*")
    ]

    for key in keys:
        key.set("qpos", " ".join(format_float(value) for value in qpos_values))
        key.set("ctrl", " ".join(format_float(value) for value in ctrl_values))


def verify_model(path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    assert model.nq == 48, f"expected nq=48, got {model.nq}"
    assert model.nu == 48, f"expected nu=48, got {model.nu}"

    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        if joint_id < 0:
            continue
        qpos_addr = model.jnt_qposadr[joint_id]
        data.ctrl[actuator_id] = model.qpos0[qpos_addr]

    for _ in range(500):
        mujoco.mj_step(model, data)

    assert np.isfinite(data.qpos).all(), "qpos went non-finite during step test"
    assert np.isfinite(data.qvel).all(), "qvel went non-finite during step test"

    for key_id in range(model.nkey):
        assert np.allclose(model.key_qpos[key_id], model.qpos0), (
            f"keyframe {model.keyframe(key_id).name!r} qpos diverges from qpos0"
        )

    body_names = {model.body(i).name for i in range(model.nbody)}
    for side in ("left", "right"):
        assert f"{side}_panda_link8" in body_names
        assert any(f"orcahand_{side}_" in name for name in body_names)

    print(
        f"  verified MJCF: nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} ngeom={model.ngeom}"
    )


def main() -> None:
    if not URDF_IN.exists():
        raise FileNotFoundError(f"Missing URDF; run build_bimanual_orcapanda_urdf.py first: {URDF_IN}")
    if not PANDA_NOHAND_XML.exists():
        raise FileNotFoundError(f"Missing vendored Panda MJCF source: {PANDA_NOHAND_XML}")

    print("[1/4] Compiling bimanual URDF once to extract OrcaHand subtrees...")
    compiled_urdf_root = compile_urdf_for_hands()

    print("[2/4] Composing two MuJoCo Menagerie Panda no-hand arms...")
    source_tree = ET.parse(PANDA_NOHAND_XML)
    source_root = source_tree.getroot()
    source_panda_body = find_body(source_root, "link0")
    tree = copy.deepcopy(source_tree)
    root = tree.getroot()
    root.set("model", "bimanual_orcapanda")

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")

    rewrite_panda_asset_paths(root)
    compose_panda_bodies(root, source_panda_body)
    add_hand_defaults(root)
    rebuild_contact_excludes(root)

    print("[3/4] Attaching left/right OrcaHand assets, bodies, and actuators...")
    append_hand_assets(root, compiled_urdf_root)
    append_hand_bodies(root, compiled_urdf_root)
    set_home_refs(root)
    rebuild_actuators(root, source_root)
    patch_keyframes(root)

    print(f"[4/4] Writing {MJCF_OUT}...")
    ET.indent(tree, space="  ")
    tree.write(MJCF_OUT, xml_declaration=False, encoding="unicode")
    verify_model(MJCF_OUT)
    print("Done.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Build orca_arm/orcapanda.xml (MJCF).

The OrcaPanda URDF remains the source of truth for the OrcaHand mount, but the
Panda arm MJCF comes from MuJoCo Menagerie instead of MuJoCo's URDF compiler.
That avoids the capsule/cylinder-only Panda approximation produced by the URDF
conversion path.
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh


BASE_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = BASE_DIR / "orca_arm"
ASSETS_DIR = PACKAGE_DIR / "assets"
PANDA_ASSETS_DIR = ASSETS_DIR / "franka_emika_panda"
PANDA_NOHAND_XML = PANDA_ASSETS_DIR / "panda_nohand.xml"
URDF_IN = PACKAGE_DIR / "orcapanda.urdf"
URDF_TMP = PACKAGE_DIR / "_orcapanda_mjcf_input.urdf"
URDF_COMPILED_TMP = PACKAGE_DIR / "_orcapanda_urdf_compiled.xml"
MJCF_OUT = PACKAGE_DIR / "orcapanda.xml"

ORCAHAND_ROOT_BODY = "orcahand_right_ForeArmStructure-Model_e18f2368"

ARM_BODY_RENAMES = {f"link{i}": f"panda_link{i}" for i in range(8)}
ARM_BODY_RENAMES["attachment"] = "panda_link8"
ARM_JOINT_RENAMES = {f"joint{i}": f"panda_joint{i}" for i in range(1, 8)}


def dae_to_stl_relative(rel_path: str) -> str:
    """Convert a URDF-relative DAE mesh to a sibling STL mesh."""
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


def compile_urdf_for_orcahand() -> ET.Element:
    """Compile the URDF so we can reuse MuJoCo's OrcaHand subtree conversion."""
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
        if not filename or "/" in filename:
            continue
        mesh.set("file", f"assets/franka_emika_panda/{filename}")


def rename_panda_tree(root: ET.Element) -> None:
    for body in root.iter("body"):
        name = body.get("name")
        if name in ARM_BODY_RENAMES:
            body.set("name", ARM_BODY_RENAMES[name])
        if body.get("name") == "panda_link8":
            body.set("pos", "0 0 0.107")
            body.attrib.pop("quat", None)

    for joint in root.iter("joint"):
        name = joint.get("name")
        if name in ARM_JOINT_RENAMES:
            joint.set("name", ARM_JOINT_RENAMES[name])

    for actuator in root.findall("./actuator/*"):
        joint = actuator.get("joint")
        if joint in ARM_JOINT_RENAMES:
            renamed = ARM_JOINT_RENAMES[joint]
            actuator.set("joint", renamed)
            actuator.set("name", f"act_{renamed}")

    for exclude in root.findall("./contact/exclude"):
        for attr in ("body1", "body2"):
            body = exclude.get(attr)
            if body in ARM_BODY_RENAMES:
                exclude.set(attr, ARM_BODY_RENAMES[body])


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


def append_orcahand_assets(root: ET.Element, compiled_urdf_root: ET.Element) -> None:
    dst_asset = root.find("asset")
    src_asset = compiled_urdf_root.find("asset")
    if dst_asset is None or src_asset is None:
        raise ValueError("Missing asset section while composing OrcaPanda MJCF")

    existing_names = {
        mesh.get("name")
        for mesh in dst_asset.findall("mesh")
        if mesh.get("name")
    }
    for mesh in src_asset.findall("mesh"):
        name = mesh.get("name", "")
        if name.startswith("panda_") or name in existing_names:
            continue
        cloned = copy.deepcopy(mesh)
        filename = cloned.get("file")
        if filename and os.path.isabs(filename):
            cloned.set("file", os.path.relpath(filename, PACKAGE_DIR))
        dst_asset.append(cloned)
        existing_names.add(name)


def append_orcahand_body(root: ET.Element, compiled_urdf_root: ET.Element) -> None:
    panda_link8 = find_body(root, "panda_link8")
    orcahand_body = find_body(compiled_urdf_root, ORCAHAND_ROOT_BODY)
    panda_link8.append(copy.deepcopy(orcahand_body))


def joint_ranges(root: ET.Element) -> dict[str, tuple[float, float]]:
    ranges = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        pair = parse_pair(joint.get("range"))
        if name and pair is not None:
            ranges[name] = pair
    return ranges


def patch_actuator_ctrlranges(root: ET.Element) -> None:
    ranges = joint_ranges(root)
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")

    for actuator_elem in actuator:
        joint = actuator_elem.get("joint")
        if joint in ranges:
            lo, hi = ranges[joint]
            actuator_elem.set("ctrlrange", f"{format_float(lo)} {format_float(hi)}")

    hand_joint_names = [
        joint.get("name")
        for joint in root.iter("joint")
        if joint.get("name", "").startswith("orcahand_right_")
    ]
    for name in hand_joint_names:
        if name is None:
            continue
        position = ET.SubElement(actuator, "position")
        position.set("class", "hand_joint")
        position.set("name", f"act_{name}")
        position.set("joint", name)
        if name in ranges:
            lo, hi = ranges[name]
            position.set("ctrlrange", f"{format_float(lo)} {format_float(hi)}")


def patch_keyframes(root: ET.Element) -> None:
    ranges = joint_ranges(root)
    qpos_length = len(ranges)
    actuator_count = len(root.findall("./actuator/*"))
    for key in root.findall("./keyframe/key"):
        for attr, length in (("qpos", qpos_length), ("ctrl", actuator_count)):
            values = key.get(attr, "").split()
            if len(values) < length:
                values.extend(["0"] * (length - len(values)))
            key.set(attr, " ".join(values[:length]))


def set_home_refs(root: ET.Element) -> None:
    for joint in root.iter("joint"):
        pair = parse_pair(joint.get("range"))
        if pair is None:
            continue
        lo, hi = pair
        home = min(max(0.0, lo), hi)
        if home != 0.0:
            joint.set("ref", format_float(home))


def verify_model(path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)

    assert model.nq == 24, f"expected nq=24, got {model.nq}"
    assert model.nu == 24, f"expected nu=24, got {model.nu}"

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

    body_names = {model.body(i).name for i in range(model.nbody)}
    assert "panda_link8" in body_names
    assert any("orcahand_right" in name for name in body_names)

    print(
        f"  verified MJCF: nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} ngeom={model.ngeom}"
    )


def main() -> None:
    if not PANDA_NOHAND_XML.exists():
        raise FileNotFoundError(
            f"Missing vendored Panda MJCF source: {PANDA_NOHAND_XML}"
        )

    print("[1/4] Compiling URDF once to extract the OrcaHand subtree...")
    compiled_urdf_root = compile_urdf_for_orcahand()

    print("[2/4] Loading MuJoCo Menagerie Panda no-hand MJCF...")
    tree = ET.parse(PANDA_NOHAND_XML)
    root = tree.getroot()
    root.set("model", "orcapanda")

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")

    rewrite_panda_asset_paths(root)
    rename_panda_tree(root)
    add_hand_defaults(root)

    print("[3/4] Attaching OrcaHand assets, body tree, and actuators...")
    append_orcahand_assets(root, compiled_urdf_root)
    append_orcahand_body(root, compiled_urdf_root)
    set_home_refs(root)
    patch_actuator_ctrlranges(root)
    patch_keyframes(root)

    print(f"[4/4] Writing {MJCF_OUT}...")
    ET.indent(tree, space="  ")
    tree.write(MJCF_OUT, xml_declaration=False, encoding="unicode")
    verify_model(MJCF_OUT)
    print("Done.")


if __name__ == "__main__":
    main()

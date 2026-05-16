"""Tests for the Panda + OrcaHand embodiment."""

import os
import xml.etree.ElementTree as ET

import numpy as np

import orca_arm


def _mesh_path(filename):
    if filename.startswith("assets/"):
        return os.path.join(os.path.dirname(orca_arm.ORCAPANDA_URDF_PATH), filename)
    return filename


def _filename_handler(fname):
    return _mesh_path(fname)


def test_orcapanda_files_exist():
    assert os.path.isfile(orca_arm.ORCAPANDA_URDF_PATH)
    assert os.path.isfile(orca_arm.ORCAPANDA_MJCF_PATH)


def test_orcapanda_urdf_tree_and_mount():
    root = ET.parse(orca_arm.ORCAPANDA_URDF_PATH).getroot()
    links = root.findall("link")
    joints = root.findall("joint")

    assert root.get("name") == "orcapanda"

    all_links = {link.get("name") for link in links}
    child_links = {
        joint.find("child").get("link")
        for joint in joints
        if joint.find("child") is not None
    }
    assert all_links - child_links == {"world"}

    mount = root.find("./joint[@name='panda_link8_to_orcahand_joint']")
    assert mount is not None
    assert mount.get("type") == "fixed"
    assert mount.find("parent").get("link") == "panda_link8"
    assert mount.find("child").get("link") == "orcahand_right_ForeArmStructure-Model_e18f2368"
    assert mount.find("origin").get("xyz") == "0 0 0.0575"

    dynamic_joints = [joint for joint in joints if joint.get("type") != "fixed"]
    assert len(dynamic_joints) == 24
    assert {f"panda_joint{i}" for i in range(1, 8)} <= {
        joint.get("name") for joint in dynamic_joints
    }
    assert any(joint.get("name", "").startswith("orcahand_right_") for joint in dynamic_joints)


def test_orcapanda_urdf_meshes_resolve():
    root = ET.parse(orca_arm.ORCAPANDA_URDF_PATH).getroot()
    missing = [
        mesh.get("filename")
        for mesh in root.iter("mesh")
        if not os.path.isfile(_mesh_path(mesh.get("filename", "")))
    ]
    assert missing == []


def test_orcapanda_loads_with_yourdfpy():
    import yourdfpy

    robot = yourdfpy.URDF.load(
        orca_arm.ORCAPANDA_URDF_PATH,
        filename_handler=_filename_handler,
    )
    assert robot.robot.name == "orcapanda"
    assert len(robot.actuated_joint_names) == 24


def test_orcapanda_mjcf_loads_and_steps():
    import mujoco

    model = mujoco.MjModel.from_xml_path(orca_arm.ORCAPANDA_MJCF_PATH)
    data = mujoco.MjData(model)

    assert model.nq == 24
    assert model.nu == 24

    qpos0_violations = []
    for joint_id in range(model.njnt):
        if not model.jnt_limited[joint_id]:
            continue
        qpos_addr = model.jnt_qposadr[joint_id]
        qpos0 = model.qpos0[qpos_addr]
        lower, upper = model.jnt_range[joint_id]
        if qpos0 < lower or qpos0 > upper:
            qpos0_violations.append((model.joint(joint_id).name, qpos0, lower, upper))
    assert qpos0_violations == []

    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        assert np.allclose(
            model.actuator_ctrlrange[actuator_id],
            model.jnt_range[joint_id],
        )

    body_names = {model.body(i).name for i in range(model.nbody)}
    assert "panda_link8" in body_names
    assert any("orcahand_right" in name for name in body_names)

    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        qpos_addr = model.jnt_qposadr[joint_id]
        data.ctrl[actuator_id] = model.qpos0[qpos_addr]

    for _ in range(200):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


def test_orcapanda_mjcf_home_keyframe_matches_qpos0():
    import mujoco

    model = mujoco.MjModel.from_xml_path(orca_arm.ORCAPANDA_MJCF_PATH)
    assert model.nkey >= 1
    for key_id in range(model.nkey):
        assert np.allclose(model.key_qpos[key_id], model.qpos0)


def test_orcapanda_mjcf_uses_menagerie_panda_meshes():
    root = ET.parse(orca_arm.ORCAPANDA_MJCF_PATH).getroot()
    mesh_files = {mesh.get("file", "") for mesh in root.findall("./asset/mesh")}
    body_names = {body.get("name", "") for body in root.iter("body")}

    assert "assets/franka_emika_panda/link0_0.obj" in mesh_files
    assert "assets/franka_emika_panda/link7_7.obj" in mesh_files
    assert not any(name.startswith("panda_link") and name.endswith("_sc") for name in body_names)

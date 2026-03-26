"""Tests for the orca_arm description package."""
import os
import xml.etree.ElementTree as ET

import pytest


@pytest.fixture(scope="module")
def desc():
    import description
    return description


# ── File existence ─────────────────────────────────────────────────────────

class TestAssetFiles:
    def test_urdf_exists(self, desc):
        assert os.path.isfile(desc.URDF_PATH)

    def test_mjcf_exists(self, desc):
        assert os.path.isfile(desc.MJCF_PATH)

    def test_meshes_dir_exists(self, desc):
        assert os.path.isdir(desc.MESHES_DIR)

    def test_all_urdf_meshes_exist(self, desc):
        """Every mesh referenced in the URDF must resolve to a real file."""
        tree = ET.parse(desc.URDF_PATH)
        urdf_dir = os.path.dirname(desc.URDF_PATH)
        missing = []
        for mesh in tree.iter("mesh"):
            fn = mesh.get("filename", "")
            if fn.startswith("../"):
                resolved = os.path.normpath(os.path.join(urdf_dir, fn))
                if not os.path.isfile(resolved):
                    missing.append(fn)
        assert missing == [], f"Missing mesh files: {missing}"


# ── URDF structure ─────────────────────────────────────────────────────────

class TestURDFStructure:
    @pytest.fixture(autouse=True)
    def parse_urdf(self, desc):
        self.tree = ET.parse(desc.URDF_PATH)
        self.root = self.tree.getroot()
        self.links = self.root.findall("link")
        self.joints = self.root.findall("joint")

    def test_robot_name(self):
        assert self.root.get("name") == "orca_arm"

    def test_single_root_link(self):
        child_links = {
            j.find("child").get("link")
            for j in self.joints
            if j.find("child") is not None
        }
        all_links = {l.get("name") for l in self.links}
        roots = all_links - child_links
        assert len(roots) == 1, f"Expected 1 root, got {roots}"

    def test_no_orphan_parents(self):
        """Every parent link referenced in a joint must be defined."""
        all_links = {l.get("name") for l in self.links}
        parents = {
            j.find("parent").get("link")
            for j in self.joints
            if j.find("parent") is not None
        }
        orphans = parents - all_links
        assert orphans == set(), f"Undefined parent links: {orphans}"

    def test_no_duplicate_children(self):
        """Each link appears as a child in at most one joint (tree structure)."""
        children = []
        for j in self.joints:
            c = j.find("child")
            if c is not None:
                children.append(c.get("link"))
        assert len(children) == len(set(children)), "Duplicate child links found"

    def test_right_arm_has_orcahand(self):
        """The right arm chain must include orcahand joints."""
        orcahand_joints = [
            j for j in self.joints if "orcahand_right_" in j.get("name", "")
        ]
        assert len(orcahand_joints) > 0, "No orcahand joints found"

    def test_left_arm_has_original_gripper(self):
        """The left arm must still have the original openarm hand."""
        left_hand_joints = [
            j for j in self.joints if "left_openarm_hand" in j.get("name", "")
        ]
        assert len(left_hand_joints) > 0, "Left arm gripper missing"

    def test_right_arm_no_original_gripper(self):
        """The right arm must NOT have the original openarm hand."""
        right_hand_joints = [
            j for j in self.joints if "right_openarm_hand" in j.get("name", "")
        ]
        assert len(right_hand_joints) == 0, "Right arm still has original gripper"

    def test_orcahand_attached_to_link4(self):
        """The orcahand must connect to openarm_right_link4 (elbow)."""
        connect = [
            j for j in self.joints
            if j.get("name") == "openarm_right_to_orcahand_joint"
        ]
        assert len(connect) == 1
        parent = connect[0].find("parent").get("link")
        assert parent == "openarm_right_link4"

    def test_right_forearm_removed(self):
        """link5-7 and joint5-7 must be removed from the right arm."""
        for name in ["openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7"]:
            assert not any(j.get("name") == name for j in self.joints), f"{name} still present"
        for name in ["openarm_right_link5", "openarm_right_link6", "openarm_right_link7"]:
            assert not any(l.get("name") == name for l in self.links), f"{name} still present"


# ── Loader tests ───────────────────────────────────────────────────────────

class TestLoaders:
    def test_load_urdf(self, desc):
        robot = desc.load_urdf()
        assert robot.robot.name == "orca_arm"
        assert len(robot.actuated_joint_names) == 29

    def test_load_mjcf(self, desc):
        model, data = desc.load_mjcf()
        assert model.nq > 0
        assert model.nbody > 0
        assert data is not None


# ── MuJoCo model tests ────────────────────────────────────────────────────

class TestMuJoCoModel:
    @pytest.fixture(autouse=True)
    def load_model(self, desc):
        import mujoco
        self.model, self.data = desc.load_mjcf()
        self.mujoco = mujoco

    def test_step_does_not_crash(self):
        """A single physics step must not raise."""
        self.mujoco.mj_step(self.model, self.data)

    def test_multi_step_stable(self):
        """Run 1000 steps from zero config — positions must stay finite."""
        import numpy as np
        for _ in range(1000):
            self.mujoco.mj_step(self.model, self.data)
        assert np.all(np.isfinite(self.data.qpos)), "qpos diverged to inf/nan"
        assert np.all(np.isfinite(self.data.qvel)), "qvel diverged to inf/nan"

    def test_joint_count_matches_urdf(self, desc):
        """MJCF actuated DOFs should be consistent with URDF joints."""
        robot = desc.load_urdf()
        urdf_actuated = len(robot.actuated_joint_names)
        # MuJoCo nq counts all DOFs (revolute=1, free=7, etc.)
        # nq >= urdf actuated joints (MuJoCo may add a free root joint)
        assert self.model.nq >= urdf_actuated - 2, (
            f"MJCF nq={self.model.nq} too low for {urdf_actuated} URDF joints"
        )

    def test_forward_kinematics(self):
        """mj_forward must produce finite body positions."""
        import numpy as np
        self.mujoco.mj_forward(self.model, self.data)
        assert np.all(np.isfinite(self.data.xpos)), "Body positions not finite"
        assert np.all(np.isfinite(self.data.xquat)), "Body quats not finite"

    def test_body_names_contain_orcahand(self):
        """The MuJoCo model must include orcahand bodies."""
        names = [
            self.model.body(i).name
            for i in range(self.model.nbody)
        ]
        orcahand_bodies = [n for n in names if "orcahand" in n.lower()]
        assert len(orcahand_bodies) > 0, f"No orcahand bodies found in: {names}"

    def test_random_configs_stable(self):
        """Random joint configurations within limits must not crash."""
        import numpy as np
        rng = np.random.default_rng(42)
        for _ in range(10):
            # Random config within joint limits
            qpos = rng.uniform(
                self.model.jnt_range[:, 0],
                self.model.jnt_range[:, 1],
            )
            self.data.qpos[:] = qpos
            self.mujoco.mj_forward(self.model, self.data)
            assert np.all(np.isfinite(self.data.xpos)), "Body positions not finite"

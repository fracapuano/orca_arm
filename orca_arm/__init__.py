"""orca_arm: paths to the orcabot URDF/MJCF assets.

Usage from a downstream project (e.g. a ManiSkill fork):

    import orca_arm
    urdf_path = orca_arm.URDF_PATH   # absolute path to orcabot.urdf
    mjcf_path = orca_arm.MJCF_PATH   # absolute path to orcabot.xml
"""
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent

URDF_PATH = str(_REPO_ROOT / "orcabot.urdf")
MJCF_PATH = str(_REPO_ROOT / "orcabot.xml")

__all__ = ["URDF_PATH", "MJCF_PATH"]

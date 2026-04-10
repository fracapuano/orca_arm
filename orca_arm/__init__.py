"""orca_arm: paths to the orcabot URDF/MJCF assets.

Usage from a downstream project (e.g. a ManiSkill fork):

    import orca_arm
    urdf_path = orca_arm.URDF_PATH   # absolute path to orcabot.urdf
    mjcf_path = orca_arm.MJCF_PATH   # absolute path to orcabot.xml

The URDF and its meshes are bundled inside the installed package, so
both editable (`pip install -e .`) and regular (`pip install git+...`)
installs work without needing the repo or its submodules on disk.
"""
from pathlib import Path

_PACKAGE_DIR = Path(__file__).resolve().parent

URDF_PATH = str(_PACKAGE_DIR / "orcabot.urdf")
MJCF_PATH = str(_PACKAGE_DIR / "orcabot.xml")
ASSETS_DIR = str(_PACKAGE_DIR / "assets")

__all__ = ["URDF_PATH", "MJCF_PATH", "ASSETS_DIR"]

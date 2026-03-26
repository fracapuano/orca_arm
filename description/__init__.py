"""orca_arm_description -- self-contained robot description package.

Provides URDF and MJCF paths plus loader helpers so external code can
interface with the robot without knowing the internal layout.

Usage:
    import orca_arm_description as orca

    # Paths
    orca.URDF_PATH   # -> .../orca_arm_description/urdf/orca_arm.urdf
    orca.MJCF_PATH   # -> .../orca_arm_description/mjcf/orca_arm.xml

    # Loaders
    robot = orca.load_urdf()          # yourdfpy.URDF
    model, data = orca.load_mjcf()    # mujoco.MjModel, mujoco.MjData
"""

import os as _os

# -- Package layout --
PACKAGE_DIR = _os.path.dirname(_os.path.abspath(__file__))
URDF_PATH = _os.path.join(PACKAGE_DIR, "urdf", "orca_arm.urdf")
MJCF_PATH = _os.path.join(PACKAGE_DIR, "mjcf", "orca_arm.xml")
MESHES_DIR = _os.path.join(PACKAGE_DIR, "meshes")


def _mesh_handler(fname):
    """Resolve relative mesh paths (../meshes/...) from the URDF location."""
    if fname.startswith("../"):
        return _os.path.normpath(_os.path.join(PACKAGE_DIR, "urdf", fname))
    return fname


def load_urdf():
    """Load the orca_arm URDF and return a yourdfpy.URDF object."""
    import yourdfpy
    return yourdfpy.URDF.load(URDF_PATH, filename_handler=_mesh_handler)


def load_mjcf():
    """Load the orca_arm MJCF and return (mujoco.MjModel, mujoco.MjData)."""
    import mujoco
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)
    return model, data

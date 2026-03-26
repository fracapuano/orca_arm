#!/usr/bin/env python3
"""
Package orca_arm into a self-contained description repository.

Copies only the referenced meshes and source URDFs, rewrites all mesh paths
to relative paths (from the URDF file), and produces URDF + MJCF assets
that work without any external dependencies.
"""

import os
import shutil
import xml.etree.ElementTree as ET

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(BASE_DIR, "orcabot.urdf")
OUTPUT_DIR = os.path.join(BASE_DIR, "description")

OPENARM_DESC = os.path.join(BASE_DIR, "openarm_description_repo")
ORCAHAND_DESC = os.path.join(BASE_DIR, "orcahand_repo")

# ── Clean output directory ─────────────────────────────────────────────────
if os.path.exists(OUTPUT_DIR):
    shutil.rmtree(OUTPUT_DIR)

# ── Create directory structure ─────────────────────────────────────────────
dirs = [
    "urdf",
    "mjcf",
    "urdf/source/openarm",
    "urdf/source/orcahand_v2",
    "meshes/openarm/arm/v10/visual",
    "meshes/openarm/arm/v10/collision",
    "meshes/openarm/body/v10/visual",
    "meshes/openarm/body/v10/collision",
    "meshes/openarm/ee/openarm_hand/visual",
    "meshes/openarm/ee/openarm_hand/collision",
    "meshes/orcahand/right",
]
for d in dirs:
    os.makedirs(os.path.join(OUTPUT_DIR, d), exist_ok=True)

# ── Step 1: Copy mesh files ───────────────────────────────────────────────
print("[1/5] Copying mesh files...")

tree = ET.parse(URDF_PATH)
root = tree.getroot()

# Map: absolute source path -> relative path within orca_arm_description
path_map = {}

for mesh in root.iter("mesh"):
    fn = mesh.get("filename", "")
    if not fn.startswith("file://"):
        continue
    abs_path = fn[7:]

    if "openarm_description_repo/meshes/" in abs_path:
        rel_from_repo = abs_path.split("openarm_description_repo/meshes/")[1]
        new_rel = f"meshes/openarm/{rel_from_repo}"
    elif "orcahand_repo/v2/models/assets/right/" in abs_path:
        filename = os.path.basename(abs_path)
        new_rel = f"meshes/orcahand/right/{filename}"
    else:
        print(f"  WARNING: Unknown mesh path: {abs_path}")
        continue

    path_map[abs_path] = new_rel

copied = 0
for src_path, rel_path in path_map.items():
    dst_path = os.path.join(OUTPUT_DIR, rel_path)
    os.makedirs(os.path.dirname(dst_path), exist_ok=True)
    if os.path.exists(src_path) and not os.path.exists(dst_path):
        shutil.copy2(src_path, dst_path)
        copied += 1
print(f"  Copied {copied} unique mesh files.")

# ── Step 2: Rewrite URDF with relative mesh paths ─────────────────────────
print("[2/5] Writing URDF with relative mesh paths...")

# Rename robot
root.set("name", "orca_arm")

# Mesh paths are relative from the URDF file (in urdf/) to meshes/ (sibling dir)
# So: ../meshes/openarm/... and ../meshes/orcahand/...
for mesh in root.iter("mesh"):
    fn = mesh.get("filename", "")
    if not fn.startswith("file://"):
        continue
    abs_path = fn[7:]
    if abs_path in path_map:
        mesh.set("filename", f"../{path_map[abs_path]}")

ET.indent(tree, space="  ")
urdf_out = os.path.join(OUTPUT_DIR, "urdf", "orca_arm.urdf")
tree.write(urdf_out, xml_declaration=True, encoding="unicode")
print(f"  Written: urdf/orca_arm.urdf")

# ── Step 3: Generate MJCF from URDF ───────────────────────────────────────
print("[3/5] Generating MJCF from URDF...")

try:
    import mujoco
    import re
    import tempfile

    # MuJoCo strips directory prefixes from URDF <mesh filename="..."> and
    # resolves them relative to a single meshdir. So we:
    # 1. Create a temp URDF with bare filenames (no paths)
    # 2. Symlink all meshes into one flat temp directory
    # 3. Load with MuJoCo pointing meshdir at that flat dir
    # 4. Save MJCF and rewrite paths to use the real package layout

    # Collect all unique mesh basenames and their source paths
    orig_tree = ET.parse(URDF_PATH)
    orig_root = orig_tree.getroot()

    mesh_sources = {}  # basename -> absolute source path
    for mesh in orig_root.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith("file://"):
            abs_path = fn[7:]
            basename = os.path.basename(abs_path)
            # Handle collisions: some openarm and orcahand files may share names
            # Prefix with source to disambiguate
            if "openarm" in abs_path:
                safe_name = f"openarm_{basename}"
            else:
                safe_name = f"orcahand_{basename}"
            mesh_sources[safe_name] = abs_path
            mesh.set("filename", safe_name)

    with tempfile.TemporaryDirectory() as tmpdir:
        # Write temp URDF with bare mesh names.
        # Add <mujoco> tag inside URDF for compiler flags (MuJoCo supports this).
        mj_tag = ET.SubElement(orig_root, "mujoco")
        compiler_tag = ET.SubElement(mj_tag, "compiler")
        compiler_tag.set("balanceinertia", "true")
        compiler_tag.set("angle", "radian")

        tmp_urdf = os.path.join(tmpdir, "robot.urdf")
        orig_tree.write(tmp_urdf, xml_declaration=True, encoding="unicode")

        # Symlink all meshes into the temp dir
        for safe_name, src_path in mesh_sources.items():
            dst = os.path.join(tmpdir, safe_name)
            if os.path.exists(src_path) and not os.path.exists(dst):
                os.symlink(src_path, dst)

        # Load with MuJoCo
        model = mujoco.MjModel.from_xml_path(tmp_urdf)
        print(f"  MuJoCo model: nq={model.nq}, nv={model.nv}, nbody={model.nbody}, njnt={model.njnt}")

        # Save as MJCF
        mjcf_out = os.path.join(OUTPUT_DIR, "mjcf", "orca_arm.xml")
        mujoco.mj_saveLastXML(mjcf_out, model)

    # Now fix the MJCF to use the real package mesh paths.
    # The saved MJCF has meshdir pointing to the tmpdir and mesh files
    # with the safe_name prefixed basenames. Replace with proper relative paths.
    with open(mjcf_out, "r") as f:
        mjcf_content = f.read()

    # Fix compiler: set meshdir relative from mjcf/ to meshes/
    # and keep balanceinertia
    if 'meshdir=' in mjcf_content:
        mjcf_content = re.sub(r'meshdir="[^"]*"', 'meshdir="../meshes"', mjcf_content)
    elif '<compiler' in mjcf_content:
        mjcf_content = mjcf_content.replace(
            '<compiler', '<compiler meshdir="../meshes" balanceinertia="true"', 1
        )
    else:
        mjcf_content = mjcf_content.replace(
            '<mujoco', '<mujoco>\n  <compiler meshdir="../meshes" balanceinertia="true" angle="radian"/>\n<', 1
        )

    # Replace prefixed mesh filenames with relative paths under meshes/
    for safe_name, src_path in mesh_sources.items():
        if safe_name in mjcf_content:
            # Map to the package-relative path
            if "openarm_description_repo/meshes/" in src_path:
                rel = "openarm/" + src_path.split("openarm_description_repo/meshes/")[1]
            elif "orcahand_repo/v2/models/assets/right/" in src_path:
                rel = "orcahand/right/" + os.path.basename(src_path)
            else:
                continue
            mjcf_content = mjcf_content.replace(f'file="{safe_name}"', f'file="{rel}"')
            mjcf_content = mjcf_content.replace(f'"{safe_name}"', f'"{rel}"')

    # Clean up model name
    mjcf_content = mjcf_content.replace('model="orcabot"', 'model="orca_arm"')
    mjcf_content = mjcf_content.replace('model="MuJoCo Model"', 'model="orca_arm"')

    # Add scene elements: lighting, floor, and actuators.
    # Insert after <compiler> line and before <asset>.
    scene_elements = """
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.1 0.1 0.1"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <statistic center="0 0 0.4" extent="1.2"/>
"""

    # Insert scene elements after compiler line
    mjcf_content = mjcf_content.replace(
        '\n\n  <asset>',
        scene_elements + '\n  <asset>'
    )

    # Add floor to worldbody (just after <worldbody> opening)
    floor_geom = """
    <light pos="0 0 2.5" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3" castshadow="true"/>
    <light pos="1 1 2" dir="-0.5 -0.5 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.8 0.8 0.8 1" pos="0 0 0"/>
"""
    mjcf_content = mjcf_content.replace(
        '<worldbody>\n',
        '<worldbody>\n' + floor_geom
    )

    # Add position actuators for all joints
    # Parse the current MJCF to get joint names and ranges
    import xml.etree.ElementTree as ET_mjcf
    tmp_root = ET_mjcf.fromstring(mjcf_content)
    actuator_lines = ['  <actuator>']
    for body in tmp_root.iter('body'):
        for jnt in body.findall('joint'):
            jname = jnt.get('name', '')
            jrange = jnt.get('range', '')
            # Higher kp for arm joints, lower for hand
            kp = "100" if "openarm" in jname and any(f"joint{i}" in jname for i in [1,2,3,4]) else \
                 "50" if "openarm" in jname else "5"
            actuator_lines.append(
                f'    <position name="act_{jname}" joint="{jname}" kp="{kp}" ctrlrange="{jrange}"/>'
            )
    actuator_lines.append('  </actuator>')
    actuator_block = '\n'.join(actuator_lines)

    # Insert actuators before </mujoco>
    mjcf_content = mjcf_content.replace('</mujoco>', actuator_block + '\n</mujoco>')

    with open(mjcf_out, "w") as f:
        f.write(mjcf_content)

    # Verify the MJCF loads from its final location
    model2 = mujoco.MjModel.from_xml_path(mjcf_out)
    print(f"  MJCF verified: mjcf/orca_arm.xml (nq={model2.nq}, nu={model2.nu} actuators)")

except Exception as e:
    print(f"  MJCF generation failed: {e}")
    import traceback
    traceback.print_exc()
    print("  (URDF is still valid — MJCF can be generated manually)")

# ── Step 4: Copy source URDFs for reference ────────────────────────────────
print("[4/5] Copying source URDFs...")

openarm_src = os.path.join(OUTPUT_DIR, "urdf", "source", "openarm")
for subdir in ["urdf/robot", "urdf/arm", "urdf/ee", "urdf/body",
               "config/arm/v10", "config/hand/openarm_hand", "config/body/v10"]:
    src = os.path.join(OPENARM_DESC, subdir)
    if os.path.exists(src):
        dst = os.path.join(openarm_src, subdir)
        shutil.copytree(src, dst, dirs_exist_ok=True)

orcahand_src = os.path.join(OUTPUT_DIR, "urdf", "source", "orcahand_v2")
shutil.copy2(
    os.path.join(ORCAHAND_DESC, "v2", "models", "urdf", "orcahand_right.urdf"),
    os.path.join(orcahand_src, "orcahand_right.urdf"),
)
print("  Copied openarm xacro/config and orcahand v2 right URDF.")

# ── Step 5: Write __init__.py module ───────────────────────────────────────
print("[5/5] Writing __init__.py module...")

with open(os.path.join(OUTPUT_DIR, "__init__.py"), "w") as f:
    f.write('''"""orca_arm_description -- self-contained robot description package.

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
''')

# ── Summary ────────────────────────────────────────────────────────────────
print(f"\n{'='*60}")
print(f"Package created: {OUTPUT_DIR}")
print(f"{'='*60}")

total = 0
for dirpath, dirnames, filenames in os.walk(OUTPUT_DIR):
    total += len(filenames)

print(f"  Total files: {total}")
print(f"  Mesh files: {len(set(path_map.values()))}")
print()
print("  Simulation assets:")
print("    urdf/orca_arm.urdf")
print("    mjcf/orca_arm.xml")
print()
print("  Source references:")
print("    urdf/source/openarm/")
print("    urdf/source/orcahand_v2/")
print()
print("  Python module: __init__.py")
print()
print("  Usage from external code:")
print("    import orca_arm_description as orca")
print("    robot = orca.load_urdf()")
print("    model, data = orca.load_mjcf()")

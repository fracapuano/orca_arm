# ROS 2 URDF description package export for the orca_arm bimanual robot.
#
#   build/orca_arm_description/
#   ├── package.xml
#   ├── urdf/orca_arm.urdf      (mesh URIs rewritten to package://)
#   └── meshes/*.stl *.dae
#
# Usage:
#   make urdf-export                          # build/orca_arm_description/
#   make urdf-export EXPORT_DIR=/tmp/orca     # custom location
#   make clean-urdf-export

EXPORT_DIR ?= build/orca_arm_description
PKG_NAME   := orca_arm_description

URDF_SRC   := orca_arm/orcabot.urdf
ASSETS_SRC := orca_arm/assets

.PHONY: urdf-export clean-urdf-export

urdf-export:
	@test -f "$(URDF_SRC)" || { echo "Missing $(URDF_SRC)"; exit 1; }
	@test -d "$(ASSETS_SRC)" || { echo "Missing $(ASSETS_SRC)"; exit 1; }
	rm -rf "$(EXPORT_DIR)"
	mkdir -p "$(EXPORT_DIR)/urdf" "$(EXPORT_DIR)/meshes"
	cp -R "$(ASSETS_SRC)/." "$(EXPORT_DIR)/meshes/"
	sed 's|filename="assets/|filename="package://$(PKG_NAME)/meshes/|g' \
		"$(URDF_SRC)" > "$(EXPORT_DIR)/urdf/orca_arm.urdf"
	@printf '%s\n' \
		'<?xml version="1.0"?>' \
		'<package format="3">' \
		'  <name>$(PKG_NAME)</name>' \
		'  <version>0.0.1</version>' \
		'  <description>OrcaArm bimanual robot URDF description.</description>' \
		'  <maintainer email="wim@orcahand.com">OrcaHand</maintainer>' \
		'  <license>Apache-2.0</license>' \
		'  <buildtool_depend>ament_cmake</buildtool_depend>' \
		'  <export>' \
		'    <build_type>ament_cmake</build_type>' \
		'  </export>' \
		'</package>' \
		> "$(EXPORT_DIR)/package.xml"
	@N=$$(find "$(EXPORT_DIR)/meshes" -type f | wc -l | tr -d ' '); \
	echo "Wrote $(EXPORT_DIR)/{package.xml,urdf/orca_arm.urdf} and $$N mesh files under $(EXPORT_DIR)/meshes/"

clean-urdf-export:
	rm -rf "$(EXPORT_DIR)"

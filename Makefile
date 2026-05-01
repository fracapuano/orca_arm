# Self-contained URDF export for the orca_arm bimanual robot.
#
# Bundles the URDF and its meshes into a single directory that any
# generic URDF viewer (urdf-viz, foxglove, urdfpy, RViz, ...) can open
# standalone. The URDF references meshes via filenames relative to the
# URDF file, so the export is just a copy — no XML rewriting needed.
#
# Usage:
#   make urdf-export                          # build/orca_arm_urdf/
#   make urdf-export EXPORT_DIR=/tmp/orca     # custom location
#   make clean-urdf-export

EXPORT_DIR ?= build/orca_arm_urdf

URDF_SRC   := orca_arm/orcabot.urdf
ASSETS_SRC := orca_arm/assets

.PHONY: urdf-export clean-urdf-export

urdf-export:
	@test -f "$(URDF_SRC)" || { echo "Missing $(URDF_SRC)"; exit 1; }
	@test -d "$(ASSETS_SRC)" || { echo "Missing $(ASSETS_SRC)"; exit 1; }
	rm -rf "$(EXPORT_DIR)"
	mkdir -p "$(EXPORT_DIR)"
	cp "$(URDF_SRC)" "$(EXPORT_DIR)/orca_arm.urdf"
	cp -R "$(ASSETS_SRC)" "$(EXPORT_DIR)/assets"
	@N=$$(find "$(EXPORT_DIR)/assets" -type f | wc -l | tr -d ' '); \
	echo "Wrote $(EXPORT_DIR)/orca_arm.urdf and $$N mesh files under $(EXPORT_DIR)/assets/"

clean-urdf-export:
	rm -rf "$(EXPORT_DIR)"

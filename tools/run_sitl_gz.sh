#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/px4_sitl_default"
ROOTFS_DIR="${BUILD_DIR}/rootfs"

if [ ! -d "${BUILD_DIR}" ]; then
	echo "Build dir not found: ${BUILD_DIR}"
	echo "Run: make px4_sitl_default"
	exit 1
fi

# Load Gazebo environment (sets PX4_GZ_* and GZ_SIM_* paths)
if [ -f "${ROOTFS_DIR}/gz_env.sh" ]; then
	# shellcheck disable=SC1090
	. "${ROOTFS_DIR}/gz_env.sh"
else
	export PX4_GZ_MODELS="${ROOT_DIR}/Tools/simulation/gz/models"
	export PX4_GZ_WORLDS="${ROOT_DIR}/Tools/simulation/gz/worlds"
	export PX4_GZ_PLUGINS="${BUILD_DIR}/src/modules/simulation/gz_plugins"
	export PX4_GZ_SERVER_CONFIG="${ROOT_DIR}/src/modules/simulation/gz_bridge/server.config"
fi

export PX4_SIMULATOR="${PX4_SIMULATOR:-gz}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"

exec "${BUILD_DIR}/bin/px4" -i 0 -d "${BUILD_DIR}/etc" -s etc/init.d-posix/rcS

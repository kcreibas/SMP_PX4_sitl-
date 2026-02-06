#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/px4_sitl_default"

PTY_PX4="${PTY_PX4:-/tmp/smp_px4}"
PTY_GCS="${PTY_GCS:-/tmp/smp_gcs}"
PTY_MON="${PTY_MON:-/tmp/smp_mon}"
PTY_MON_PX4="${PTY_MON_PX4:-/tmp/smp_mon_px4}"
PTY_MON_GCS="${PTY_MON_GCS:-/tmp/smp_mon_gcs}"
BAUD="${PX4_SMP_BAUD:-115200}"
SMP_SIM="${SMP_SIM:-none}" # none|jmavsim|gz
SMP_ECHO="${SMP_ECHO:-0}"
SMP_ACK_MSGID="${SMP_ACK_MSGID:-}"
SMP_ACK_PAYLOAD="${SMP_ACK_PAYLOAD:-}"
SMP_TAP="${SMP_TAP:-1}"
SMP_KILL="${SMP_KILL:-1}"
SMP_PARAM="${SMP_PARAM:-}"
SMP_GZCLIENT="${SMP_GZCLIENT:-1}"
SMP_LOCAL_ACK="${SMP_LOCAL_ACK:-0}"

cleanup() {
	if [ -n "${MUX_PID:-}" ] && kill -0 "$MUX_PID" 2>/dev/null; then
		kill "$MUX_PID" || true
	fi
	if [ -n "${STUB_DIR:-}" ] && [ -d "$STUB_DIR" ]; then
		rm -rf "$STUB_DIR"
	fi
	if [ -n "${RC_TMP:-}" ] && [ -f "$RC_TMP" ]; then
		rm -f "$RC_TMP"
	fi
}

trap cleanup EXIT

if [ ! -d "${BUILD_DIR}" ]; then
	echo "Build dir not found: ${BUILD_DIR}"
	echo "Run: make px4_sitl_default"
	exit 1
fi

echo "[1/2] Starting SMP PTY mux:"
echo "  PX4 -> ${PTY_PX4}"
echo "  GCS -> ${PTY_GCS}"
echo "  MON -> ${PTY_MON}"
echo "  MON PX4 -> ${PTY_MON_PX4}"
echo "  MON GCS -> ${PTY_MON_GCS}"
echo "  TAP -> ${SMP_TAP}"
echo "  KILL -> ${SMP_KILL}"
echo "  PARAM -> ${SMP_PARAM}"
echo "  LOCAL_ACK -> ${SMP_LOCAL_ACK}"
if [ "${SMP_SIM}" = "gz" ]; then
	echo "  GZCLIENT -> ${SMP_GZCLIENT}"
fi
if [ "${SMP_KILL}" = "1" ]; then
	if command -v pkill >/dev/null 2>&1; then
		pkill -f "Tools/smp/smp_mux.py" || true
	fi
	rm -f "${PTY_PX4}" "${PTY_GCS}" "${PTY_MON}" "${PTY_MON_PX4}" "${PTY_MON_GCS}"
fi
MUX_ARGS=(--px4 "${PTY_PX4}" --gcs "${PTY_GCS}" --mon "${PTY_MON}" --mon-px4 "${PTY_MON_PX4}" --mon-gcs "${PTY_MON_GCS}" --verbose)
if [ "${SMP_TAP}" = "1" ]; then
	MUX_ARGS+=(--tap-gcs)
fi
python3 "${ROOT_DIR}/Tools/smp/smp_mux.py" "${MUX_ARGS[@]}" &
MUX_PID=$!
sleep 0.3

if [ "${SMP_SIM}" = "none" ]; then
	STUB_DIR="$(mktemp -d /tmp/px4_smp_stub.XXXXXX)"
	cat > "${STUB_DIR}/px4-rc.simulator" <<'EOF'
#!/bin/sh
echo "INFO  [init] simulator disabled (SMP-only)"
EOF
	cat > "${STUB_DIR}/px4-rc.mavlink" <<'EOF'
#!/bin/sh
echo "INFO  [init] mavlink disabled (SMP-only)"
EOF
	chmod +x "${STUB_DIR}/px4-rc.simulator" "${STUB_DIR}/px4-rc.mavlink"
	export PATH="${STUB_DIR}:${PATH}"
fi

export PX4_SMP_DEV="${PTY_PX4}"
export PX4_SMP_BAUD="${BAUD}"
if [ "${SMP_ECHO}" = "1" ]; then
	export PX4_SMP_ECHO=1
fi
if [ "${SMP_LOCAL_ACK}" = "1" ]; then
	export PX4_SMP_LOCAL_ACK=1
fi
if [ -n "${SMP_ACK_MSGID}" ]; then
	export PX4_SMP_ACK_MSGID="${SMP_ACK_MSGID}"
fi
if [ -n "${SMP_ACK_PAYLOAD}" ]; then
	export PX4_SMP_ACK_PAYLOAD="${SMP_ACK_PAYLOAD}"
fi

if [ "${SMP_SIM}" = "jmavsim" ]; then
	export PX4_SIMULATOR="jmavsim"
	export PX4_SIM_MODEL="jmavsim_iris"
elif [ "${SMP_SIM}" = "gz" ]; then
	export PX4_SIMULATOR="gz"
	export PX4_SIM_MODEL="gz_x500"
	GZ_ENV="${BUILD_DIR}/rootfs/gz_env.sh"
	if [ -f "${GZ_ENV}" ]; then
		# shellcheck disable=SC1090
		. "${GZ_ENV}"
	else
		export PX4_GZ_WORLDS="${ROOT_DIR}/Tools/simulation/gz/worlds"
		export PX4_GZ_MODELS="${ROOT_DIR}/Tools/simulation/gz/models"
		export PX4_GZ_PLUGINS="${BUILD_DIR}/src/modules/simulation/gz_plugins"
		export PX4_GZ_SERVER_CONFIG="${ROOT_DIR}/src/modules/simulation/gz_bridge/server.config"
	fi
	export PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
	if [ "${SMP_GZCLIENT}" = "1" ]; then
		if command -v gzclient >/dev/null 2>&1; then
			(gzclient >/dev/null 2>&1 &) || true
		elif command -v gz >/dev/null 2>&1; then
			(gz gui >/dev/null 2>&1 &) || true
		else
			echo "WARN: gzclient/gz not found. Install Gazebo GUI packages (e.g. gz-harmonic)."
		fi
	fi
else
	export PX4_SIMULATOR="none"
	export PX4_SIM_MODEL="none_iris"
	export PX4_PARAM_SIM_GZ_EN="0"
fi

RC_SCRIPT="etc/init.d-posix/rcS"
if [ -n "${SMP_PARAM}" ]; then
	RC_TMP="$(mktemp /tmp/px4_smp_rcS.XXXXXX)"
	cat "${RC_SCRIPT}" > "${RC_TMP}"
	echo "" >> "${RC_TMP}"
	echo "# SMP_PARAM overrides" >> "${RC_TMP}"
	for kv in ${SMP_PARAM}; do
		name="${kv%%=*}"
		val="${kv#*=}"
		echo "param set ${name} ${val}" >> "${RC_TMP}"
	done
	chmod +x "${RC_TMP}"
	RC_SCRIPT="${RC_TMP}"
fi

echo "[2/2] Starting PX4 SITL (SMP only, sim: ${SMP_SIM})"
echo "  SMP DEV: ${PX4_SMP_DEV} (baud ${PX4_SMP_BAUD})"
echo "  RX  : python3 tools/smp_rx.py --port ${PTY_MON} --baud ${BAUD}"
echo "  TX  : python3 tools/smp_tx.py --port ${PTY_GCS} --cmd 400"
echo ""
echo "PX4 is running in the foreground. Open another terminal for RX/TX."

exec "${BUILD_DIR}/bin/px4" -i 0 -d "${BUILD_DIR}/etc" -s "${RC_SCRIPT}"

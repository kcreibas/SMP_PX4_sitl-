#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/px4_sitl_default"

PTY_PX4="${PTY_PX4:-/tmp/smp_px4}"
PTY_GCS="${PTY_GCS:-/tmp/smp_gcs}"
PTY_MON="${PTY_MON:-/tmp/smp_mon}"
BAUD="${PX4_SMP_BAUD:-115200}"

cleanup() {
	if [ -n "${MUX_PID:-}" ] && kill -0 "$MUX_PID" 2>/dev/null; then
		kill "$MUX_PID" || true
	fi
	if [ -n "${STUB_DIR:-}" ] && [ -d "$STUB_DIR" ]; then
		rm -rf "$STUB_DIR"
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
python3 "${ROOT_DIR}/Tools/smp/smp_mux.py" --px4 "${PTY_PX4}" --gcs "${PTY_GCS}" --mon "${PTY_MON}" --verbose &
MUX_PID=$!
sleep 0.5

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

export PX4_SMP_DEV="${PTY_PX4}"
export PX4_SMP_BAUD="${BAUD}"
export PX4_SIMULATOR="none"
export PX4_SIM_MODEL="none"
export PX4_PARAM_SIM_GZ_EN="0"
export PATH="${STUB_DIR}:${PATH}"

echo "[2/2] Starting PX4 SITL (SMP only, MAVLink disabled)"
echo "  SMP DEV: ${PX4_SMP_DEV} (baud ${PX4_SMP_BAUD})"
echo "  Monitor: python3 Tools/smp/smp_monitor.py --port ${PTY_MON} --baud ${BAUD}"
echo "  Send CMD: python3 tools/smp_send.py --port ${PTY_GCS} --cmd 400"

"${BUILD_DIR}/bin/px4" -i 0 -d "${BUILD_DIR}/etc" -s etc/init.d-posix/rcS

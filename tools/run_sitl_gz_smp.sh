#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PTY_PX4="${PTY_PX4:-/tmp/smp_px4}"
PTY_GCS="${PTY_GCS:-/tmp/smp_gcs}"
PTY_MON="${PTY_MON:-/tmp/smp_mon}"
BAUD="${PX4_SMP_BAUD:-115200}"

cleanup() {
	if [ -n "${MUX_PID:-}" ] && kill -0 "$MUX_PID" 2>/dev/null; then
		kill "$MUX_PID" || true
	fi
}

trap cleanup EXIT

echo "[1/2] Starting SMP PTY mux:"
echo "  PX4 -> ${PTY_PX4}"
echo "  GCS -> ${PTY_GCS}"
echo "  MON -> ${PTY_MON}"
python3 "${ROOT_DIR}/Tools/smp/smp_mux.py" --px4 "${PTY_PX4}" --gcs "${PTY_GCS}" --mon "${PTY_MON}" --verbose &
MUX_PID=$!
sleep 0.5

echo "[2/2] Starting PX4 SITL (gz)"
export PX4_SMP_DEV="${PTY_PX4}"
export PX4_SMP_BAUD="${BAUD}"

exec "${ROOT_DIR}/tools/run_sitl_gz.sh"

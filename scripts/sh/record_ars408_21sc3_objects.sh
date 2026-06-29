#!/usr/bin/env bash
set -euo pipefail

INTERFACE="${1:-can0}"
DURATION="${2:-10}"
OUTPUT_DIR="${3:-$HOME/radar_logs}"
LABEL="${4:-ars408_21sc3}"
WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "$WORKSPACE/install/setup.bash"
  set -u
fi

mkdir -p "$OUTPUT_DIR"
OUTPUT="$OUTPUT_DIR/${LABEL}_objects.csv"

echo "=== Set objects mode ==="
ros2 run pe_ars408_ros ars408_set_mode.py \
  --interface "$INTERFACE" \
  --mode objects \
  --quality true \
  --ext-info true
sleep 1

echo "=== Record objects mode for ${DURATION}s -> ${OUTPUT} ==="
ros2 run pe_ars408_ros ars408_can_logger.py \
  --interface "$INTERFACE" \
  --duration "$DURATION" \
  --output "$OUTPUT"

echo "Done: $OUTPUT"
echo "Summary: ${OUTPUT_DIR}/${LABEL}_objects_summary.csv"

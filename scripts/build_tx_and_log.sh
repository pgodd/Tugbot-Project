#!/usr/bin/env bash
set -euo pipefail

# Build wrapper: ensures .pio exists and captures verbose TX build output
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$PROJECT_ROOT/.pio"
LOG_FILE="$LOG_DIR/tugbot_tx_build.log"

mkdir -p "$LOG_DIR"
echo "Running PlatformIO build for env 'tugbot_tx'..."
echo "Logging to: $LOG_FILE"

platformio run -e tugbot_tx -v 2>&1 | tee "$LOG_FILE"

echo
echo "Build finished. Last 200 lines of log:"
tail -n 200 "$LOG_FILE" || true

exit 0

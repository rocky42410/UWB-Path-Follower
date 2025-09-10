# File: tests/smoke.sh
#!/usr/bin/env bash
set -euo pipefail

BINARY="../build/uwb_path_controller"
LOG_FILE="test_run.log"
TIMEOUT=15

if [ ! -f "$BINARY" ]; then
  echo "Binary not found at $BINARY"
  echo "Build first: mkdir -p ../build && cd ../build && cmake .. && make -j"
  exit 1
fi

echo "Test 1: --help"
$BINARY --help >/dev/null

echo "Test 2: Missing interface handling"
if $BINARY 2>&1 | grep -qi "interface required"; then
  echo "✓ missing interface rejected"
else
  echo "✗ missing interface not rejected"
  exit 1
fi

# The following run will attempt to talk to the robot SDK; adjust iface for your rig.
IFACE="${IFACE_OVERRIDE:-eth0}"

echo "Test 3: Short OPEN_LOOP run on $IFACE"
set +e
timeout $TIMEOUT $BINARY --interface "$IFACE" --mode OPEN_LOOP > "$LOG_FILE" 2>&1
RC=$?
set -e

if [ -f "../uwb_path_log.csv" ]; then
  L=$(wc -l < ../uwb_path_log.csv)
  echo "Log lines: $L"
else
  echo "No log file found (this may be expected without a robot connection)."
fi

echo "Smoke tests finished (exit $RC)."
exit 0

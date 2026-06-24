#!/usr/bin/env bash
# tune_mfc.sh — build ANTON_MFC (nps), run a timed sim, then analyze the CSV.
#
# Usage:
#   ./tune_mfc.sh [--duration SEC] [--no-build] [--csv PATH]
#
# Defaults:
#   duration: 30s
#   no-build: false (build before running)
#   csv:      auto-detected from /tmp/mfc_sim_*.csv (latest)
#
# The sim runs in the container (via sim.sh) and saves a CSV to /tmp/ inside
# the container, which is shared with the host via the bind-mount.  After the
# sim exits, analyze_mfc.py is run on the latest CSV to print stability metrics.
#
# Examples:
#   ./tune_mfc.sh               # build + 30 s run + analysis
#   ./tune_mfc.sh --duration 60 # 60 s run
#   ./tune_mfc.sh --no-build    # skip rebuild, reuse last binary

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DURATION=30
NO_BUILD=0
CSV_OVERRIDE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --no-build) NO_BUILD=1; shift ;;
    --csv)      CSV_OVERRIDE="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

# ── Build ────────────────────────────────────────────────────────────────────
if [[ $NO_BUILD -eq 0 ]]; then
  echo "=== Building ANTON_MFC (nps) ==="
  ./pprz.sh build ANTON_MFC nps
fi

# ── Run sim ──────────────────────────────────────────────────────────────────
echo ""
echo "=== Running sim for ${DURATION}s ==="
echo "    (ctrl-C to abort early; CSV will still be analyzed)"

# sim.sh launches simsitl + sim_anton.py in a container.
# Kill it after DURATION seconds.
TIMEOUT_CMD="timeout --signal=SIGINT ${DURATION}s"

# sim.sh exits with non-zero on SIGINT; ignore that.
${TIMEOUT_CMD} ./sim.sh ANTON_MFC || true

# ── Find CSV ─────────────────────────────────────────────────────────────────
if [[ -n "$CSV_OVERRIDE" ]]; then
  CSV="$CSV_OVERRIDE"
else
  CSV=$(ls -t "$SCRIPT_DIR/sim_logs"/mfc_sim_*.csv 2>/dev/null | head -1 || echo "")
fi

if [[ -z "$CSV" ]]; then
  echo ""
  echo "No CSV found in sim_logs/ — sim may not have run long enough or crashed early."
  echo "Check: ls $SCRIPT_DIR/sim_logs/"
  exit 1
fi

echo ""
echo "=== Analyzing: $CSV ==="
python3 "$SCRIPT_DIR/analyze_mfc.py" "$CSV"

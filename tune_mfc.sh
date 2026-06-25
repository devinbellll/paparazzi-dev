#!/usr/bin/env bash
# tune_mfc.sh — grade an MFC run with analyze_mfc.py, from any of three feeds:
#
#   (default)  build + timed NPS sim + analyse the sim CSV   (sim tuning loop)
#   --scope F  analyse an already-captured NPS scope JSON     (sim OR flight)
#   --flight L analyse a downloaded SD flight log: convert .data -> scope JSON
#              (tools/sdlog2scope.py) then run the SAME analyser
#
# The point: ONE analyser grades a sim run and a real flight identically, via the
# unified NPS_SCOPE JSON schema (see Knowledge/Plans/MFC Flight-Test Enablement.md).
#
# Usage:
#   ./tune_mfc.sh [--duration SEC] [--no-build] [--csv PATH] [--aircraft AC]
#   ./tune_mfc.sh --scope  CAPTURE.json
#   ./tune_mfc.sh --flight FLIGHT.data        # decoded SD log (sd2log output)
#
# Notes:
#   * Sim capture of the scope UDP stream is unreliable headless in the sandbox,
#     so the default sim path still grades sim_anton.py's CSV. To grade a sim run
#     through the scope schema instead, record the scope to a file (PlotJuggler,
#     or a UDP sink on the scope port) and pass it with --scope.
#   * --flight needs the log already decoded to .data (run `sd2log FLIGHT.TLM`,
#     which needs the Paparazzi ground segment, then pass the resulting .data).

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DURATION=30
NO_BUILD=0
CSV_OVERRIDE=""
AIRCRAFT="ANTON_MFC"
SCOPE_FILE=""
FLIGHT_LOG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --no-build) NO_BUILD=1; shift ;;
    --csv)      CSV_OVERRIDE="$2"; shift 2 ;;
    --aircraft) AIRCRAFT="$2"; shift 2 ;;
    --scope)    SCOPE_FILE="$2"; shift 2 ;;
    --flight)   FLIGHT_LOG="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

# ── Feed (b): pre-captured scope JSON (sim or flight) ────────────────────────
if [[ -n "$SCOPE_FILE" ]]; then
  echo "=== Analysing scope capture: $SCOPE_FILE ==="
  python3 "$SCRIPT_DIR/analyze_mfc.py" "$SCOPE_FILE"
  exit 0
fi

# ── Feed (c): SD flight log -> scope JSON -> analyse ─────────────────────────
if [[ -n "$FLIGHT_LOG" ]]; then
  JSON="${FLIGHT_LOG%.data}.scope.json"
  echo "=== Converting flight log -> scope JSON: $FLIGHT_LOG -> $JSON ==="
  python3 "$SCRIPT_DIR/tools/sdlog2scope.py" "$FLIGHT_LOG" -o "$JSON"
  echo ""
  echo "=== Analysing: $JSON ==="
  python3 "$SCRIPT_DIR/analyze_mfc.py" "$JSON"
  exit 0
fi

# ── Feed (a): build + timed sim + analyse the sim CSV ────────────────────────
if [[ $NO_BUILD -eq 0 ]]; then
  echo "=== Building ${AIRCRAFT} (nps) ==="
  ./pprz.sh build "${AIRCRAFT}" nps
fi

echo ""
echo "=== Running sim for ${DURATION}s ==="
echo "    (ctrl-C to abort early; CSV will still be analyzed)"

# sim.sh launches simsitl + sim_anton.py in a container.  Kill after DURATION s.
TIMEOUT_CMD="timeout --signal=SIGINT ${DURATION}s"
# sim.sh exits non-zero on SIGINT; ignore that.
${TIMEOUT_CMD} ./sim.sh "${AIRCRAFT}" || true

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

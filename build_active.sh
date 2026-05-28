#!/usr/bin/env bash
# Build whatever C/C++ configuration is currently selected in VS Code.
#
# Invoked from tasks.json with the active config name, e.g.:
#   ./build_active.sh "ANTON_MFC (ap)"          # full, correct build
#   ./build_active.sh "ANTON_MFC (ap)" fast     # incremental airborne-only build
#
# Config names must be of the form "AIRCRAFT (target)", matching the names in
# c_cpp_properties.json. This is what makes the single C/C++ picker drive both
# IntelliSense and the build.

set -euo pipefail

WS=/workspace
PPRZ=${WS}/paparazzi
CONF_XML="${PPRZ}/conf/airframes/ENAC/conf_enac.xml"

NAME="${1:?Usage: $0 \"AIRCRAFT (target)\" [fast]}"
MODE="${2:-full}"

# ── Parse "AIRCRAFT (target)" ────────────────────────────────────────────────
if [[ "$NAME" != *"("*")"* ]]; then
  echo "ERROR: config name '$NAME' is not of the form 'AIRCRAFT (target)'." >&2
  exit 2
fi
AIRCRAFT="${NAME%% (*}"        # everything before " ("
TARGET="${NAME##*(}"          # everything after the last "("
TARGET="${TARGET%)}"          # strip trailing ")"

echo "==> Active config : $NAME"
echo "==> Aircraft      : $AIRCRAFT"
echo "==> Target        : $TARGET"
echo "==> Mode          : $MODE"
echo ""

# ── Fast path: airborne-only incremental compile ─────────────────────────────
# Skips the host-tools scan and the ac_h codegen. Make is already incremental
# at the object level, so this only recompiles changed .c files.
# VALID ONLY IF codegen is current (i.e. you have NOT changed the airframe XML
# since the last full build / codegen). If you changed the XML, run the full
# build or the codegen task first.
if [[ "$MODE" == "fast" ]]; then
  LTO_ARG=()
  [[ "$TARGET" == "ap" ]] && LTO_ARG=(USE_LTO=no)
  echo "==> Fast incremental compile (airborne only)..."
  exec make -C "${PPRZ}/sw/airborne" \
      AIRCRAFT="$AIRCRAFT" \
      TARGET="$TARGET" \
      "${LTO_ARG[@]}" \
      -j"$(nproc)" \
      all
fi

# ── Full path: correct end-to-end build via build_fw.sh ──────────────────────
exec "${WS}/build_fw.sh" "$AIRCRAFT" "$CONF_XML" "$TARGET"

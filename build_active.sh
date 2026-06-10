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

  # The fast path invokes sw/airborne/Makefile directly, bypassing Makefile.ac
  # which normally computes and exports the PPRZ version vars. Without them,
  # sw/airborne/Makefile emits -DPPRZ_VER_MAJOR= (empty), which breaks the
  # PPRZ_VERSION_INT macro in autopilot.c whenever that TU is recompiled.
  # Recreate the vars here, mirroring Makefile.ac exactly.
  GIT_SHA1=$(cd "$PPRZ" && git log -1 --pretty=format:%H 2>/dev/null || echo UNKNOWN)
  GIT_DESC=$(cd "$PPRZ" && ./paparazzi_version)
  PPRZ_VER=$(echo "$GIT_DESC" | sed 's/[^0-9.]*\([0-9.]*\).*/\1/')
  PPRZ_VER_MAJOR=$(echo "$GIT_DESC" | sed 's/v\([0-9]*\).*/\1/')
  PPRZ_VER_MINOR=$(echo "$GIT_DESC" | sed 's/v[0-9]*.\([0-9]*\).*/\1/')
  PPRZ_VER_PATCH=$(echo "$GIT_DESC" | sed 's/v[0-9]*.[0-9]*.\([0-9]*\).*/\1/')
  [[ $(echo "$PPRZ_VER_PATCH" | wc -w) -eq 1 ]] || PPRZ_VER_PATCH=0

  echo "==> Fast incremental compile (airborne only)..."
  exec make -C "${PPRZ}/sw/airborne" \
      AIRCRAFT="$AIRCRAFT" \
      TARGET="$TARGET" \
      GIT_SHA1="$GIT_SHA1" \
      GIT_DESC="$GIT_DESC" \
      PPRZ_VER="$PPRZ_VER" \
      PPRZ_VER_MAJOR="$PPRZ_VER_MAJOR" \
      PPRZ_VER_MINOR="$PPRZ_VER_MINOR" \
      PPRZ_VER_PATCH="$PPRZ_VER_PATCH" \
      "${LTO_ARG[@]}" \
      -j"$(nproc)" \
      all
fi

# ── Full path: correct end-to-end build via build_fw.sh ──────────────────────
exec "${WS}/build_fw.sh" "$AIRCRAFT" "$CONF_XML" "$TARGET"

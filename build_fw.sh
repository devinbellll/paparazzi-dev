#!/usr/bin/env bash
# Paparazzi firmware build script
# Usage: ./build_fw.sh [-c] AIRCRAFT CONF_XML [TARGET]
#
# Options:
#   -c   Clean the aircraft build first (required when switching LTO on/off)
#
# Example:
#   ./build_fw.sh ANTON conf/airframes/ENAC/conf_enac.xml
#   ./build_fw.sh -c ANTON conf/airframes/ENAC/conf_enac.xml ap

set -euo pipefail

PPRZ=/workspace/paparazzi
CLEAN=false

# ── Args ──────────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "-c" ]]; then
  CLEAN=true
  shift
fi

AIRCRAFT="${1:?Usage: $0 [-c] AIRCRAFT CONF_XML [TARGET]}"
CONF_XML="${2:?Usage: $0 [-c] AIRCRAFT CONF_XML [TARGET]}"
TARGET="${3:-ap}"

echo "==> Aircraft : $AIRCRAFT"
echo "==> Conf     : $CONF_XML"
echo "==> Target   : $TARGET"
echo ""

# ── Host tools ────────────────────────────────────────────────────────────────
# Run serially (-j1) to avoid parallel git submodule sync lock conflicts in
# the dronecan submodule. Matches the documented install step: `make -j1`.
echo "==> Building host tools..."
make -j1 -C "$PPRZ"

# ── Firmware ──────────────────────────────────────────────────────────────────
if [[ "$CLEAN" == true ]]; then
  echo "==> Cleaning previous aircraft build..."
  make -C "$PPRZ" -f Makefile.ac AIRCRAFT="$AIRCRAFT" clean_ac
fi

# USE_LTO=no works around a gcc-arm-none-eabi 13.2 LTO ICE triggered by
# Rosetta 2 emulation (linux/amd64 container on Apple Silicon).
echo "==> Compiling firmware..."
make -C "$PPRZ" -f Makefile.ac \
  AIRCRAFT="$AIRCRAFT" \
  CONF_XML="$CONF_XML" \
  USE_LTO=no \
  "${TARGET}.compile"

echo ""
echo "==> Done: $PPRZ/var/aircrafts/$AIRCRAFT/$TARGET/obj/${TARGET}.elf"

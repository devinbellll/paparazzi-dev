#!/usr/bin/env bash
# Paparazzi firmware build script
# Usage: ./build_fw.sh [-c] AIRCRAFT CONF_XML [TARGET]
#
# Options:
#   -c   Clean the aircraft build first (required when switching LTO on/off)
#
# Example:
#   ./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml
#   ./build_fw.sh -c ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml ap
#
# EXECUTION MODEL
#   If the ARM cross-compiler is on PATH (we are inside the toolchain container,
#   or on a host that has it), the build runs natively here.
#   Otherwise (the lean Claude sbx sandbox) the script re-invokes ITSELF inside
#   an ephemeral arm64 toolchain container (see pprz_docker.sh / Dockerfile.build),
#   with the repo bind-mounted at /workspace. Build outputs land back in the
#   workspace tree, visible to both the sandbox and host VSCode.

set -euo pipefail

# Preserve the original argv so we can forward it verbatim into the container.
ORIG_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Dispatch into the toolchain container unless we're on a Linux host that has
# the full toolchain (i.e. inside the build container). macOS always dispatches,
# even if arm-none-eabi-gcc was installed there for clangd — the OCaml ground
# segment / codegen only exist in the container.
if [[ "$(uname -s)" != "Linux" ]] || ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  # shellcheck source=pprz_docker.sh
  source "$SCRIPT_DIR/pprz_docker.sh"
  echo "==> Dispatching build into container '$(pprz_image_name)'..."
  pprz_run -- ./build_fw.sh ${ORIG_ARGS[@]+"${ORIG_ARGS[@]}"}
  exit $?
fi

# ── Native build (inside the toolchain container, or a toolchained host) ───────
PPRZ="${PAPARAZZI_HOME:-/workspace/paparazzi}"
CLEAN=false

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
# The generator stamps USE_GENERATED_AUTOPILOT=TRUE only for the current
# target's block in Makefile.ac.  An AP build followed by an NPS build (or
# vice versa) would reuse the stale file and compile the wrong autopilot
# source.  Always delete it so the generator regenerates it fresh for THIS
# target.
MAKEFILE_AC="$PPRZ/var/aircrafts/$AIRCRAFT/Makefile.ac"
if [[ -f "$MAKEFILE_AC" ]]; then
  rm -f "$MAKEFILE_AC"
fi

if [[ "$CLEAN" == true ]]; then
  echo "==> Cleaning previous aircraft build..."
  make -C "$PPRZ" -f Makefile.ac AIRCRAFT="$AIRCRAFT" clean_ac
fi

# USE_LTO=no works around a gcc-arm-none-eabi 13.2 LTO ICE seen under Rosetta and
# also keeps incremental dev rebuilds fast. (arm64-native no longer hits the ICE,
# but LTO is left off for fast iteration; flip to USE_LTO=yes for release builds.)
echo "==> Compiling firmware..."
make -C "$PPRZ" -f Makefile.ac \
  AIRCRAFT="$AIRCRAFT" \
  CONF_XML="$CONF_XML" \
  USE_LTO=no \
  "${TARGET}.compile"

echo ""
echo "==> Done: $PPRZ/var/aircrafts/$AIRCRAFT/$TARGET/obj/${TARGET}.elf"

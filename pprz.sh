#!/usr/bin/env bash
# ── pprz.sh — unified Paparazzi build/IDE tool (ephemeral container dispatch) ──
#
# Consolidates the former build_active.sh / gen_compile_db.sh / gen_build_log.sh
# / gen_vscode.sh into one parametric front door. Each subcommand runs inside an
# ephemeral arm64 toolchain container (repo bind-mounted at /workspace); artifacts
# that VSCode on the Mac reads (compile_commands.json, build.log) are rewritten
# from the container's /workspace prefix to the real host repo path.
#
# Subcommands (config name form is "AIRCRAFT (target)", matching c_cpp_properties):
#   build  "ANTON_MFC (ap)" [fast]   Build the config (full, or fast airborne-only).
#   db     "ANTON_MFC (ap)"          Generate compile_commands.json (clangd) + host rewrite.
#   log    "ANTON_MFC (ap)"          Capture build.log (Makefile-Tools) + host rewrite.
#   codegen "ANTON_MFC (ap)"         Regenerate airframe.h / modules.h headers only.
#
# Set PPRZ_HOST_CC to a Mac arm-none-eabi-gcc path (e.g. from
# `brew install --cask gcc-arm-embedded`) to also rewrite the compiler in the DB
# so clangd resolves bare-metal system headers. Optional — project headers and
# generated #defines resolve without it.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONF_XML_DEFAULT="paparazzi/conf/airframes/ENAC/conf_enac.xml"

usage() {
  sed -n '2,28p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

# Parse a "AIRCRAFT (target)" config name into AIRCRAFT / TARGET globals.
parse_config() {
  local name="$1"
  if [[ "$name" != *"("*")"* ]]; then
    echo "ERROR: '$name' is not of the form 'AIRCRAFT (target)'." >&2
    exit 2
  fi
  AIRCRAFT="${name%% (*}"
  TARGET="${name##*(}"; TARGET="${TARGET%)}"
}

# ── Host-side path rewrite of build-emitted artifacts ─────────────────────────
# Container emits /workspace-rooted paths; VSCode on the Mac reads at the real
# repo path. Rewrite in place. HOST_ROOT defaults to where this repo lives.
host_rewrite() {
  local file="$1"
  local host_root="${PPRZ_HOST_ROOT:-$SCRIPT_DIR}"
  [[ -f "$file" ]] || { echo "!! expected artifact missing: $file" >&2; return 1; }
  sed -i "s#/workspace#${host_root}#g" "$file"
  if [[ -n "${PPRZ_HOST_CC:-}" ]]; then
    # Point clangd at the Mac cross-compiler so it can query system includes.
    sed -i "s#\"/usr/bin/arm-none-eabi-gcc\"#\"${PPRZ_HOST_CC}\"#g; s#\"arm-none-eabi-gcc\"#\"${PPRZ_HOST_CC}\"#g" "$file"
  fi
  echo "==> Rewrote container paths -> ${host_root} in $(basename "$file")"
}

# ─────────────────────────────────────────────────────────────────────────────
# IN-CONTAINER implementations (toolchain present). Everything here is rooted at
# /workspace, exactly as the old scripts assumed.
# ─────────────────────────────────────────────────────────────────────────────
PPRZ=/workspace/paparazzi

_in_build() {
  parse_config "$1"; local mode="${2:-full}"
  local conf="${PPRZ}/${CONF_XML_DEFAULT#paparazzi/}"

  if [[ "$mode" == "fast" ]]; then
    # Airborne-only incremental compile. VALID ONLY if codegen is current (XML
    # unchanged since the last full build). Recreates the PPRZ version vars that
    # Makefile.ac normally exports, so PPRZ_VERSION_INT doesn't break.
    local lto=(); [[ "$TARGET" == "ap" ]] && lto=(USE_LTO=no)
    local sha desc ver maj min pat
    sha=$(cd "$PPRZ" && git log -1 --pretty=format:%H 2>/dev/null || echo UNKNOWN)
    desc=$(cd "$PPRZ" && ./paparazzi_version)
    ver=$(echo "$desc"  | sed 's/[^0-9.]*\([0-9.]*\).*/\1/')
    maj=$(echo "$desc"  | sed 's/v\([0-9]*\).*/\1/')
    min=$(echo "$desc"  | sed 's/v[0-9]*.\([0-9]*\).*/\1/')
    pat=$(echo "$desc"  | sed 's/v[0-9]*.[0-9]*.\([0-9]*\).*/\1/')
    [[ $(echo "$pat" | wc -w) -eq 1 ]] || pat=0
    echo "==> Fast incremental compile (airborne only): $AIRCRAFT / $TARGET"
    exec make -C "${PPRZ}/sw/airborne" \
      AIRCRAFT="$AIRCRAFT" TARGET="$TARGET" \
      GIT_SHA1="$sha" GIT_DESC="$desc" \
      PPRZ_VER="$ver" PPRZ_VER_MAJOR="$maj" PPRZ_VER_MINOR="$min" PPRZ_VER_PATCH="$pat" \
      "${lto[@]}" -j"$(nproc)" all
  fi

  echo "==> Full build: $AIRCRAFT / $TARGET"
  exec /workspace/build_fw.sh "$AIRCRAFT" "$conf" "$TARGET"
}

_in_db() {
  parse_config "$1"
  local conf="${PPRZ}/${CONF_XML_DEFAULT#paparazzi/}"
  local db="${PPRZ}/var/aircrafts/${AIRCRAFT}/${TARGET}/compile_commands.json"
  mkdir -p "$(dirname "$db")"
  echo "==> compile_commands.json via bear (clean build): $AIRCRAFT / $TARGET"
  bear --output "$db" -- /workspace/build_fw.sh -c "$AIRCRAFT" "$conf" "$TARGET"
  # Also expose at repo root so clangd auto-discovers the active config's DB.
  cp "$db" /workspace/compile_commands.json
  echo "==> Wrote $db (+ /workspace/compile_commands.json)"
}

_in_log() {
  parse_config "$1"
  local conf="${PPRZ}/${CONF_XML_DEFAULT#paparazzi/}"
  local log="${PPRZ}/var/aircrafts/${AIRCRAFT}/${TARGET}/build.log"
  mkdir -p "$(dirname "$log")"
  local lto=(); [[ "$TARGET" == "ap" ]] && lto=(USE_LTO=no)
  echo "==> build.log capture (Q='' verbose): $AIRCRAFT / $TARGET"
  make -C "$PPRZ" -f Makefile.ac \
    AIRCRAFT="$AIRCRAFT" CONF_XML="$conf" Q='' "${lto[@]}" \
    "${TARGET}.compile" 2>&1 | tee "$log"
}

_in_codegen() {
  parse_config "$1"
  local conf="${PPRZ}/${CONF_XML_DEFAULT#paparazzi/}"
  echo "==> Codegen (headers only): $AIRCRAFT / $TARGET"
  make -j1 -C "$PPRZ"
  make -C "$PPRZ" -f Makefile.ac AIRCRAFT="$AIRCRAFT" CONF_XML="$conf" "${TARGET}.ac_h"
}

# ─────────────────────────────────────────────────────────────────────────────
# DISPATCH: when no local toolchain (Mac host / sbx sandbox), re-invoke this
# script inside the container, then post-process artifacts on the host.
# ─────────────────────────────────────────────────────────────────────────────
SUB="${1:-}"; shift || true
[[ -z "$SUB" || "$SUB" == "-h" || "$SUB" == "--help" ]] && usage 0

if [[ "$(uname -s)" == "Linux" ]] && command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  # Inside the container (Linux + toolchain) — run the real work.
  case "$SUB" in
    build)   _in_build   "$@";;
    db)      _in_db      "$@";;
    log)     _in_log     "$@";;
    codegen) _in_codegen "$@";;
    *) echo "Unknown subcommand: $SUB" >&2; usage 1;;
  esac
  exit $?
fi

# Host side: dispatch into the container, then rewrite emitted paths.
# shellcheck source=pprz_docker.sh
source "$SCRIPT_DIR/pprz_docker.sh"
HOST_ROOT="${WORKSPACE_DIR:-$SCRIPT_DIR}"

echo "==> Dispatching '$SUB' into container '$(pprz_image_name)'..."
pprz_run -- ./pprz.sh "$SUB" "$@"

# Post-process artifacts that VSCode reads (path rewrite to the host root).
case "$SUB" in
  db)
    parse_config "${1:?config name required}"
    host_rewrite "$HOST_ROOT/paparazzi/var/aircrafts/$AIRCRAFT/$TARGET/compile_commands.json"
    host_rewrite "$HOST_ROOT/compile_commands.json"
    ;;
  log)
    parse_config "${1:?config name required}"
    host_rewrite "$HOST_ROOT/paparazzi/var/aircrafts/$AIRCRAFT/$TARGET/build.log"
    ;;
esac

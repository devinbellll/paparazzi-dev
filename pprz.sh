#!/usr/bin/env bash
# ── pprz.sh — single Paparazzi build / IDE tool (ephemeral arm64 container) ────
#
# One front door for every build-system flow. Each command runs inside a
# throwaway arm64 toolchain container (repo bind-mounted at /workspace) when no
# local toolchain is present (the Mac host or the sbx sandbox); artifacts land
# back in the shared tree.
#
# Usage:  ./pprz.sh <command> AIRCRAFT [TARGET]        (TARGET defaults to ap)
#
#   The VSCode tasks pass the active C/C++ config name as a single arg
#   ("AIRCRAFT (target)"); that form is accepted too.
#
# Commands:
#   build    AIRCRAFT [TARGET]   Incremental build — the single build button.
#                                Re-codegens only if the XML changed; recompiles
#                                only the C files that changed (make-driven).
#   clean    AIRCRAFT [TARGET]   Wipe var/aircrafts/<AIRCRAFT> (clean_ac).
#   rebuild  AIRCRAFT [TARGET]   clean, then build.
#   db       AIRCRAFT [TARGET]   Regenerate compile_commands.json for clangd
#                                (clean verbose build parsed by compiledb).
#   codegen  AIRCRAFT [TARGET]   Regenerate airframe.h / modules.h only.
#   bootstrap                    (Re)build the OCaml ground segment + generators.
#
# Env:
#   CONF          Fleet XML, relative to the paparazzi/ dir
#                 (default conf/airframes/ENAC/conf_enac.xml).
#   PPRZ_HOST_CC  Mac arm-none-eabi-gcc path — rewritten into the DB so clangd
#                 resolves bare-metal system headers (optional).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Fleet XML, relative to the paparazzi/ dir. gen_aircraft.out resolves -conf
# relative to make's cwd, and the build runs `make -C paparazzi`, so this is the
# canonical form (NOT the repo-root form "paparazzi/conf/...").
CONF_DEFAULT="conf/airframes/ENAC/conf_enac.xml"

PPRZ=/workspace/paparazzi   # in-container repo path

usage() { sed -n '2,33p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

# Accept "AIRCRAFT (target)" (one arg, from the VSCode picker) OR "AIRCRAFT TARGET".
parse_ac_target() {
  if [[ "${1:-}" == *"("*")"* ]]; then
    AIRCRAFT="${1%% (*}"; TARGET="${1##*(}"; TARGET="${TARGET%)}"
  else
    AIRCRAFT="${1:-}"; TARGET="${2:-ap}"
  fi
  [[ -n "$AIRCRAFT" ]] || { echo "ERROR: AIRCRAFT required (e.g. ./pprz.sh build ANTON_MFC nps)" >&2; exit 2; }
}

# ─────────────────────────────────────────────────────────────────────────────
# IN-CONTAINER implementations (Linux + toolchain). Rooted at /workspace.
# ─────────────────────────────────────────────────────────────────────────────

# Build the OCaml ground segment + code generators once. The per-build commands
# call this as a cheap guard; it is a near-instant no-op once present, so a full
# `make -C paparazzi` no longer runs on every single build (the old slow path).
_ensure_core() {
  if [[ -x "$PPRZ/sw/tools/generators/gen_aircraft.out" && -d "$PPRZ/var/include" ]]; then
    return 0
  fi
  echo "==> Ground segment / generators missing — building them (one-time)..."
  make -j1 -C "$PPRZ"
}

# Force a clean codegen of the shared per-aircraft Makefile.ac when (and only
# when) the target changed since the last build. ap and nps share
# var/aircrafts/<AC>/Makefile.ac (the generator stamps the autopilot per target);
# switching targets must regenerate it, but a same-target rebuild must NOT — that
# would bump its mtime and force a full recompile, killing incrementality.
_guard_target_switch() {
  local sentinel="$PPRZ/var/aircrafts/$AIRCRAFT/.pprz_last_target"
  if [[ -f "$sentinel" && "$(cat "$sentinel" 2>/dev/null)" != "$TARGET" ]]; then
    echo "==> Target changed ($(cat "$sentinel") -> $TARGET): refreshing codegen"
    rm -f "$PPRZ/var/aircrafts/$AIRCRAFT/Makefile.ac"
  fi
}

# ap is only ever built for hardware (release), so optimize for the flight binary:
# LTO does cross-file inlining/dead-code elimination -> smaller, faster firmware.
# (Was USE_LTO=no for fast incremental dev rebuilds + an old Rosetta LTO-ICE
# workaround; arm64-native no longer hits the ICE, and we never iterate on ap.)
_lto() { [[ "$TARGET" == "ap" ]] && echo "USE_LTO=yes"; }

_in_build() {
  parse_ac_target "$@"
  _ensure_core
  _guard_target_switch
  local conf="${CONF:-$CONF_DEFAULT}"
  echo "==> Build (incremental): $AIRCRAFT / $TARGET"
  # %.compile -> %.ac_h (codegen-if-XML-changed) -> sw/airborne incremental make.
  make -C "$PPRZ" -f Makefile.ac \
    AIRCRAFT="$AIRCRAFT" CONF_XML="$conf" $(_lto) "${TARGET}.compile"
  mkdir -p "$PPRZ/var/aircrafts/$AIRCRAFT"
  echo "$TARGET" > "$PPRZ/var/aircrafts/$AIRCRAFT/.pprz_last_target"
  echo "==> Done: $PPRZ/var/aircrafts/$AIRCRAFT/$TARGET/obj/$TARGET.elf"
}

_in_clean() {
  parse_ac_target "$@"
  echo "==> Clean: $AIRCRAFT (removing var/aircrafts/$AIRCRAFT)"
  make -C "$PPRZ" -f Makefile.ac AIRCRAFT="$AIRCRAFT" clean_ac
}

_in_rebuild() { _in_clean "$@"; _in_build "$@"; }

_in_codegen() {
  parse_ac_target "$@"
  _ensure_core
  local conf="${CONF:-$CONF_DEFAULT}"
  echo "==> Codegen (headers only): $AIRCRAFT / $TARGET"
  make -C "$PPRZ" -f Makefile.ac AIRCRAFT="$AIRCRAFT" CONF_XML="$conf" "${TARGET}.ac_h"
}

_in_db() {
  parse_ac_target "$@"
  _ensure_core
  local conf="${CONF:-$CONF_DEFAULT}"
  local outdir="$PPRZ/var/aircrafts/$AIRCRAFT/$TARGET"
  local db="$outdir/compile_commands.json"
  local log="$outdir/build_verbose.log"
  echo "==> compile_commands.json via compiledb (clean verbose build): $AIRCRAFT / $TARGET"
  # A full DB needs every TU to actually compile once, so clean first, then
  # capture a verbose build (USE_VERBOSE_COMPILE=yes Q='') and feed the log to
  # compiledb --parse. No bear, no gRPC wrapper. --print-directory keeps per-TU
  # include resolution correct; --build-dir seeds the cwd before the first
  # "Entering directory". A final LINK failure is tolerated on purpose: every
  # per-file compile command — all clangd needs — is already in the log, and ap
  # firmware may legitimately not link mid-development.
  make -C "$PPRZ" -f Makefile.ac AIRCRAFT="$AIRCRAFT" clean_ac
  mkdir -p "$outdir"
  if ! make --print-directory -C "$PPRZ" -f Makefile.ac \
        AIRCRAFT="$AIRCRAFT" CONF_XML="$conf" USE_VERBOSE_COMPILE=yes Q='' $(_lto) \
        "${TARGET}.compile" > "$log" 2>&1; then
    echo "!! verbose build exited non-zero (often just a final link error) —" \
         "the DB is still captured from the per-file compile steps. Log: $log"
  fi
  compiledb --parse "$log" --output "$db" --overwrite --build-dir "$PPRZ"
  cp "$db" /workspace/compile_commands.json
  local n; n="$(python3 -c "import json,sys;print(len(json.load(open('$db'))))" 2>/dev/null || echo '?')"
  echo "==> Wrote $db ($n entries) (+ /workspace/compile_commands.json)"
}

_in_bootstrap() {
  echo "==> Building ground segment + generators (make -j1 -C paparazzi)..."
  make -j1 -C "$PPRZ"
}

# ── Host-side path rewrite of the IDE artifact ───────────────────────────────
# The container emits /workspace-rooted paths; VSCode on the Mac reads at the
# real repo path. Rewrite in place.
host_rewrite() {
  local file="$1"
  local host_root="${PPRZ_HOST_ROOT:-$SCRIPT_DIR}"
  [[ -f "$file" ]] || { echo "!! expected artifact missing: $file" >&2; return 1; }
  perl -pi -e "s|/workspace|${host_root}|g" "$file"
  if [[ -n "${PPRZ_HOST_CC:-}" ]]; then
    perl -pi -e "s|\"/usr/bin/arm-none-eabi-gcc\"|\"${PPRZ_HOST_CC}\"|g; s|\"arm-none-eabi-gcc\"|\"${PPRZ_HOST_CC}\"|g" "$file"
  fi
  echo "==> Rewrote container paths -> ${host_root} in $(basename "$file")"
}

# ─────────────────────────────────────────────────────────────────────────────
# DISPATCH
# ─────────────────────────────────────────────────────────────────────────────
SUB="${1:-}"; shift || true
[[ -z "$SUB" || "$SUB" == "-h" || "$SUB" == "--help" ]] && usage 0

if [[ "$(uname -s)" == "Linux" ]] && command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  # Inside the container (Linux + toolchain) — run the real work.
  case "$SUB" in
    build)     _in_build     "$@";;
    clean)     _in_clean     "$@";;
    rebuild)   _in_rebuild   "$@";;
    db)        _in_db        "$@";;
    codegen)   _in_codegen   "$@";;
    bootstrap) _in_bootstrap "$@";;
    *) echo "Unknown command: $SUB" >&2; usage 1;;
  esac
  exit $?
fi

# Host side: dispatch into the container, then rewrite the IDE artifact.
# shellcheck source=pprz_docker.sh
source "$SCRIPT_DIR/pprz_docker.sh"
HOST_ROOT="${WORKSPACE_DIR:-$SCRIPT_DIR}"

echo "==> Dispatching '$SUB' into container '$(pprz_image_name)'..."
pprz_run -- ./pprz.sh "$SUB" "$@"

if [[ "$SUB" == "db" ]]; then
  parse_ac_target "$@"
  host_rewrite "$HOST_ROOT/paparazzi/var/aircrafts/$AIRCRAFT/$TARGET/compile_commands.json" || true
  host_rewrite "$HOST_ROOT/compile_commands.json" || true
fi

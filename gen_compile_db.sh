#!/usr/bin/env bash
# Generate a per-config compile_commands.json for VS Code IntelliSense.
#
# Uses `bear` to intercept the real compiler exec() calls during a clean
# firmware build. This captures the FULLY RESOLVED -D / -I flags per source
# file -- the only reliable way to get accurate preprocessor + call-trace
# IntelliSense out of Paparazzi's recursive, code-generated Makefile chain.
# (A `make -n` dry-run cannot do this: the airborne srcs list does not exist
# until the build runs for real.)
#
# Run this ONCE per config, and again whenever you change the airframe XML,
# add/remove source files, or change build-affecting defines.
#
# Usage:  ./gen_compile_db.sh AIRCRAFT [CONF_XML] [TARGET]
#   CONF_XML defaults to paparazzi/conf/airframes/ENAC/conf_enac.xml
#   TARGET   defaults to ap
#
# Requires bear 3.x:  sudo apt-get install bear   (or build from source)

set -euo pipefail

WS=/workspace
PPRZ=${WS}/paparazzi
AIRCRAFT="${1:?Usage: $0 AIRCRAFT [CONF_XML] [TARGET]   |   $0 \"AIRCRAFT (target)\"}"

# Accept the VS Code config-name form "AIRCRAFT (target)" as a single arg,
# so this can be driven by ${command:cpptools.activeConfigName} from tasks.json.
if [[ "$AIRCRAFT" == *"("*")"* ]]; then
  NAME="$AIRCRAFT"
  AIRCRAFT="${NAME%% (*}"
  TARGET="${NAME##*(}"; TARGET="${TARGET%)}"
  CONF_XML="${PPRZ}/conf/airframes/ENAC/conf_enac.xml"
else
  CONF_XML="${2:-${PPRZ}/conf/airframes/ENAC/conf_enac.xml}"
  TARGET="${3:-ap}"
fi

DB="${PPRZ}/var/aircrafts/${AIRCRAFT}/${TARGET}/compile_commands.json"

if ! command -v bear >/dev/null 2>&1; then
  echo "ERROR: 'bear' not found. Install it (e.g. apt-get install bear)." >&2
  exit 1
fi

echo "==> Generating compile DB for $AIRCRAFT / $TARGET"
echo "==> Output: $DB"
mkdir -p "$(dirname "$DB")"

# `-c` forces a clean aircraft build so EVERY translation unit is recompiled
# and therefore captured by bear. build_fw.sh already handles host tools,
# codegen, and USE_LTO=no for ARM targets.
bear --output "$DB" -- "${WS}/build_fw.sh" -c "$AIRCRAFT" "$CONF_XML" "$TARGET"

echo "==> Done. Select the matching C/C++ configuration in VS Code,"
echo "    then run 'C/C++: Reset IntelliSense Database' if needed."

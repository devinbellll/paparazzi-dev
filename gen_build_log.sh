#!/usr/bin/env bash
# Capture a verbose build log for Makefile Tools buildLog IntelliSense.
#
# Makefile Tools reads this log instead of doing a `make -n` dry-run, which
# fails for Paparazzi because the source list isn't resolved until the build
# actually runs. The log contains the real, fully-resolved gcc command lines
# that Makefile Tools parses for -D / -I flags per translation unit.
#
# Run once per config, and again when you change the airframe XML, add/remove
# source files, or change build-affecting defines. Normal .c/.h edits don't
# require a re-capture.
#
# After running: Ctrl+Shift+P → "Makefile: Configure" to ingest the new log.
# (Or set makefile.configureOnOpen: true to do it automatically on next open.)
#
# Usage:  ./gen_build_log.sh AIRCRAFT [CONF_XML] [TARGET]
#         ./gen_build_log.sh "AIRCRAFT (target)"   ← VS Code config-name form

set -euo pipefail

WS=/workspace
PPRZ=${WS}/paparazzi
CONF_XML_DEFAULT="${PPRZ}/conf/airframes/ENAC/conf_enac.xml"

AIRCRAFT="${1:?Usage: $0 AIRCRAFT [CONF_XML] [TARGET]   |   $0 \"AIRCRAFT (target)\"}"

# Accept the VS Code config-name form "AIRCRAFT (target)" as a single arg,
# matching the form used by ${command:cpptools.activeConfigName} in tasks.json.
if [[ "$AIRCRAFT" == *"("*")"* ]]; then
    NAME="$AIRCRAFT"
    AIRCRAFT="${NAME%% (*}"
    TARGET="${NAME##*(}"; TARGET="${TARGET%)}"
    CONF_XML="$CONF_XML_DEFAULT"
else
    CONF_XML="${2:-${CONF_XML_DEFAULT}}"
    TARGET="${3:-ap}"
fi

LOG="${PPRZ}/var/aircrafts/${AIRCRAFT}/${TARGET}/build.log"

echo "==> Capturing build log for $AIRCRAFT / $TARGET"
echo "==> Log: $LOG"
mkdir -p "$(dirname "$LOG")"

LTO_ARG=()
[[ "$TARGET" == "ap" ]] && LTO_ARG=(USE_LTO=no)

# Q='' suppresses Paparazzi's pretty-printer so gcc invocations appear in full.
# The log is what Makefile Tools reads for IntelliSense -- it never runs make -n.
make -C "$PPRZ" -f Makefile.ac \
    AIRCRAFT="$AIRCRAFT" \
    CONF_XML="$CONF_XML" \
    Q='' \
    "${LTO_ARG[@]}" \
    "${TARGET}.compile" 2>&1 | tee "$LOG"

echo ""
echo "==> Done. Run 'Makefile: Configure' in VS Code to ingest the new log."

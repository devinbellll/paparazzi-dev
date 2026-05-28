#!/usr/bin/env bash
# Codegen prebuild step for VS Code / Makefile Tools.
# Run this once after changing an airframe XML to regenerate the headers
# that Makefile Tools and c_cpp_properties.json depend on.
#
# Usage:  ./gen_vscode.sh AIRCRAFT [CONF_XML] [TARGET]
#   CONF_XML defaults to paparazzi/conf/airframes/ENAC/conf_enac.xml
#   TARGET   defaults to ap

set -euo pipefail

PPRZ=/workspace/paparazzi
AIRCRAFT="${1:?Usage: $0 AIRCRAFT [CONF_XML] [TARGET]}"
CONF_XML="${2:-${PPRZ}/conf/airframes/ENAC/conf_enac.xml}"
TARGET="${3:-ap}"

echo "==> Codegen: $AIRCRAFT / $TARGET"

make -j1 -C "$PPRZ"

make -C "$PPRZ" -f Makefile.ac \
    AIRCRAFT="$AIRCRAFT" \
    CONF_XML="$CONF_XML" \
    "${TARGET}.ac_h"

echo "==> Done: var/aircrafts/$AIRCRAFT/$TARGET/generated/"

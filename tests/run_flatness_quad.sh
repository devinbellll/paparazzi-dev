#!/usr/bin/env bash
# Analytic property checks on the flatness spine (flatness_quad.c).
#
# Unlike run_mimo_golden.sh this does NOT use tests/stubs: flatness_quad.c
# needs the real pprz quaternion/rotation algebra, so it compiles against
# sw/airborne + sw/include and links pprz_algebra_float.c.
#
# These are hand-derived properties, not a recorded reference. A closed-loop
# trace from Generic_Quad is still owed -- see the task note
# verify-flatness-quad-against-sim.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCRIPT_DIR/pprz_docker.sh"

pprz_run -- bash -c '
  set -e
  cd /workspace
  gcc -O2 -Wall -Wextra -std=c11 \
      -I paparazzi/sw/include -I paparazzi/sw/airborne \
      tests/flatness_quad_test.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/flatness_quad.c \
      paparazzi/sw/airborne/math/pprz_algebra_float.c \
      -lm -o /tmp/flatness_quad_test
  /tmp/flatness_quad_test
'

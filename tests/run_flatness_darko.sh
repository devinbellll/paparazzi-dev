#!/usr/bin/env bash
# Analytic property checks on the Darko flatness spine (flatness_darko.c).
#
# Same convention as run_flatness_quad.sh: NOT tests/stubs, because the spine
# needs the real pprz quaternion/rotation algebra. flatness_quad.c is linked in
# because the Darko spine REUSES flatness_quad_att_error() rather than
# duplicating it.
#
# There are no golden traces and none are coming: generating them needs MATLAB,
# which this environment does not have. These are hand-derived properties.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCRIPT_DIR/pprz_docker.sh"

pprz_run -- bash -c '
  set -e
  cd /workspace
  gcc -O2 -Wall -Wextra -std=c11 \
      -I paparazzi/sw/include -I paparazzi/sw/airborne \
      tests/flatness_darko_test.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/flatness_darko.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/flatness_quad.c \
      paparazzi/sw/airborne/math/pprz_algebra_float.c \
      -lm -o /tmp/flatness_darko_test
  /tmp/flatness_darko_test
'

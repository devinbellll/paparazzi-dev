#!/usr/bin/env bash
# Analytic property checks on the Darko FINDI controller law
# (oneloop_findi_darko_law.h) and its composition with the stage-1 spine.
#
# Same convention as run_flatness_darko.sh: NOT tests/stubs, because the law
# and the spine both need the real pprz quaternion/rotation algebra.
# flatness_quad.c is linked in for flatness_quad_att_error(), which the Darko
# stack REUSES rather than duplicating.
#
# There are no golden traces and none are coming: generating them needs MATLAB,
# which this environment does not have. These are hand-derived properties, and
# offline agreement is not evidence the loop flies -- see the header of
# tests/oneloop_findi_darko_test.c.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCRIPT_DIR/pprz_docker.sh"

pprz_run -- bash -c '
  set -e
  cd /workspace
  gcc -O2 -Wall -Wextra -std=c11 \
      -I paparazzi/sw/include -I paparazzi/sw/airborne \
      tests/oneloop_findi_darko_test.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/flatness_darko.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/flatness_quad.c \
      paparazzi/sw/airborne/math/pprz_algebra_float.c \
      -lm -o /tmp/oneloop_findi_darko_test
  /tmp/oneloop_findi_darko_test
'

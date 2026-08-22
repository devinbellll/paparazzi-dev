#!/usr/bin/env bash
# Analytic property checks on the Darko FMFC controller law
# (oneloop_fmfc_darko_law.h): the two HEOL/MFC brackets, their constant alpha
# matrices, their nominal inputs, and the composition with the stage-1 spine.
#
# INCLUDE PATH NOTE. This links the REAL pprz algebra (the spine needs
# quaternions and rotation matrices) and stubs ONLY the clock, from
# tests/stubs_darko/. tests/stubs/ cannot be used here: it also stubs
# math/pprz_algebra_float.h down to a single float_vect_zero(), which would
# shadow the real header the spine depends on.
#
# mfc_core.c carries the shared recursion primitives that mfc_core_mimo.c
# calls, so both are linked -- one estimator implementation, not two.
#
# There are no golden traces and none are coming: generating them needs MATLAB,
# which this environment does not have -- and the Simulink reference was itself
# untuned and not flying, so there would be no reference behaviour to compare
# against even with it. These are hand-derived STRUCTURAL properties, and
# offline agreement is not evidence the loop flies. See the header of
# tests/oneloop_fmfc_darko_test.c.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCRIPT_DIR/pprz_docker.sh"

pprz_run -- bash -c '
  set -e
  cd /workspace
  gcc -O2 -Wall -Wextra -std=c11 \
      -I tests/stubs_darko -I paparazzi/sw/include -I paparazzi/sw/airborne \
      tests/oneloop_fmfc_darko_test.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/flatness_darko.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/flatness_quad.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core_mimo.c \
      paparazzi/sw/airborne/math/pprz_algebra_float.c \
      -lm -o /tmp/oneloop_fmfc_darko_test
  /tmp/oneloop_fmfc_darko_test
'

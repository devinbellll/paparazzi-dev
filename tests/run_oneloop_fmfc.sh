#!/usr/bin/env bash
# Analytic property checks on the flatness-MFC brackets (oneloop_fmfc_law.h)
# and on their composition into the two increments oneloop_fmfc.c replaces.
#
# There are no golden traces and none are coming: generating them needs MATLAB,
# which this environment does not have. These are hand-derived properties,
# decided up front. Offline agreement is not evidence the loop flies -- see the
# header of tests/oneloop_fmfc_test.c.
#
# INCLUDE ORDER MATTERS. tests/stubs/ cannot be used here: it also shadows
# math/pprz_algebra_float.h, and these checks need the REAL pprz algebra for
# float_mat_inv_4d. tests/stubs_clock/ replaces the clock and nothing else, so
# it goes first and sw/airborne supplies everything else.
#
# The 2-vector regression (group C) links the REAL heol_mimo.c against the REAL
# mfc_core_mimo.c. Neither is modified by this module; the check below asserts
# that as well as exercising it.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# pprz_docker.sh bind-mounts $WORKSPACE_DIR at /workspace. When this session is
# launched from the vault root that variable points at the VAULT, not at this
# repo, and every path below resolves one level too high. Pin it to the repo.
export WORKSPACE_DIR="$SCRIPT_DIR"
source "$SCRIPT_DIR/pprz_docker.sh"

# The 2-vector bit-identity claim is only meaningful if the shared core really
# is untouched. Assert it against the base branch rather than trusting it.
cd "$SCRIPT_DIR/paparazzi"
SHARED="sw/airborne/firmwares/rotorcraft/stabilization/mfc_core_mimo.c \
        sw/airborne/firmwares/rotorcraft/stabilization/mfc_core_mimo.h \
        sw/airborne/firmwares/rotorcraft/stabilization/heol_mimo.c \
        sw/airborne/firmwares/rotorcraft/stabilization/heol_mimo.h \
        sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c \
        sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.h"
if ! git diff --quiet cd0ba036f -- $SHARED; then
  echo "FAIL: the shared MFC core / heol_mimo has been modified:" >&2
  git diff --stat cd0ba036f -- $SHARED >&2
  exit 1
fi
echo "shared mfc_core_mimo / heol_mimo / mfc_core: unmodified vs cd0ba036f  PASS"
cd "$SCRIPT_DIR"

pprz_run -- bash -c '
  set -e
  cd /workspace
  gcc -O2 -Wall -Wextra -std=c11 \
      -I tests/stubs_clock -I paparazzi/sw/include -I paparazzi/sw/airborne \
      tests/oneloop_fmfc_test.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core_mimo.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/heol_mimo.c \
      paparazzi/sw/airborne/math/pprz_algebra_float.c \
      -lm -o /tmp/oneloop_fmfc_test
  /tmp/oneloop_fmfc_test
'

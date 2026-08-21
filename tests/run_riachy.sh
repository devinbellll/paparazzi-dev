#!/usr/bin/env bash
# Property checks on Riachy's trick in mfc_core.c (use_riachy).
#
# Same convention as run_mimo_golden.sh: stubs FIRST on the include path so
# mcu_periph/sys_time.h resolves to the harness clock. Runs in the
# paparazzi-build container (host gcc is not assumed).
#
# No golden traces: MFC_SISO's reference uses the sliding-window estimator and
# integrates the raw measurement, neither of which this core does. See the
# header comment in riachy_test.c.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCRIPT_DIR/pprz_docker.sh"

pprz_run -- bash -c '
  set -e
  cd /workspace
  gcc -O2 -Wall -Wextra -std=c11 \
      -I tests/stubs -I paparazzi/sw/airborne \
      tests/riachy_test.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c \
      -lm -o /tmp/riachy_test
  /tmp/riachy_test
'

#!/usr/bin/env bash
# Replay the sim repo's reference traces through the real firmware sources.
# Runs in the paparazzi-build container (host gcc is not assumed), from the
# repo root so the tests/golden/ paths resolve.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SCRIPT_DIR/pprz_docker.sh"

# Stubs FIRST on the include path so mcu_periph/sys_time.h and
# math/pprz_algebra_float.h resolve to the harness versions, not the real ones.
pprz_run -- bash -c '
  set -e
  cd /workspace
  gcc -O2 -Wall -Wextra -std=c11 \
      -I tests/stubs -I paparazzi/sw/airborne \
      tests/mimo_golden_test.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core_mimo.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/heol_mimo.c \
      paparazzi/sw/airborne/firmwares/rotorcraft/guidance/heol_input_sensitivity.c \
      -lm -o /tmp/mimo_golden_test
  /tmp/mimo_golden_test
'

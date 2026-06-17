#!/usr/bin/env bash
# ── Run the ANTON NPS / SITL simulation in an ephemeral container ─────────────
#
# Usage: ./sim.sh [--no-build] [sim_anton.py flags...]
#   ./sim.sh --mfc --render          # build ANTON_MFC nps, run with TUI dashboard
#   ./sim.sh --z                     # ANTON_MFC_THRUST
#   ./sim.sh --mfc --gdb             # wait for a debugger on :1234 (see below)
#   ./sim.sh --no-build --mfc        # skip the rebuild, just run
#
# The sim + the OCaml ground segment (server, link) + the IVY bus all run INSIDE
# one container; only the visualization streams out to the Mac host:
#   PlotJuggler  host.docker.internal:9870   (default scope)
#   FlightGear   host.docker.internal:5501   (--fg)
# Container uses --network host so its egress matches the sandbox's.
#
# Debugging (--gdb): the sim waits for gdbserver on :1234. On the HOST run
#   sbx ports "$SANDBOX_VM_ID" --publish 1234:1234
# then attach from VSCode (.vscode/launch.json -> "Attach: NPS sim").
#
# Visualization egress to the Mac may need the relevant UDP ports allowed in the
# sandbox network policy / published — verify at runtime.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=pprz_docker.sh
source "$SCRIPT_DIR/pprz_docker.sh"

CONF="${CONF:-conf/airframes/ENAC/conf_enac.xml}"

# ── Parse our own flags; forward the rest to sim_anton.py ─────────────────────
DO_BUILD=true
SIM_ARGS=()
for a in "$@"; do
  case "$a" in
    --no-build) DO_BUILD=false ;;
    *) SIM_ARGS+=("$a") ;;
  esac
done

# Pick the aircraft the way sim_anton.py does (must match for the nps binary).
AC_NAME="ANTON"
for a in ${SIM_ARGS[@]+"${SIM_ARGS[@]}"}; do
  [[ "$a" == "--mfc" ]] && AC_NAME="ANTON_MFC"
  [[ "$a" == "--z"   ]] && AC_NAME="ANTON_MFC_THRUST"
done

# ── Build the NPS target (incremental) ────────────────────────────────────────
if [[ "$DO_BUILD" == true ]]; then
  echo "==> Building NPS target for $AC_NAME..."
  "$SCRIPT_DIR/build_fw.sh" "$AC_NAME" "$CONF" nps
fi

# ── Networking differs by daemon ──────────────────────────────────────────────
# sbx sandbox (Claude): nested DinD — share the sandbox netns so IVY + egress to
#   the Mac behave as they do from the sandbox; gdbserver :1234 lands on the
#   sandbox, publish to the Mac with `sbx ports "$SANDBOX_VM_ID" --publish 1234:1234`.
# Mac Docker Desktop (VSCode task): host networking doesn't reach macOS; use the
#   default bridge (host.docker.internal works natively) and map the gdb port.
NET_OPTS=()
if [[ -n "${IS_SANDBOX:-}" ]]; then
  # The sim shares the sandbox netns (--network host). DO NOT use
  # `--add-host host.docker.internal:host-gateway` here: in the nested DinD that
  # resolves to the inner bridge gateway (172.18.0.1), a dead-end that never
  # reaches the Mac, so PlotJuggler/FlightGear get nothing.
  # Instead point host.docker.internal at the Mac's real IPv4. sbx injects
  # host.docker.internal -> 169.254.1.1 (IPv4) / fe80::1 (IPv6); we need the
  # IPv4 because sim_anton.py uses gethostbyname() and the NPS C scope/FG
  # emitters use inet_addr() (dotted-decimal only — no IPv6). Resolve it from
  # the sandbox so we don't hardcode the alias.
  MAC_HOST_IP="$(python3 -c 'import socket;print([f[4][0] for f in socket.getaddrinfo("host.docker.internal",0,socket.AF_INET)][0])' 2>/dev/null || echo 169.254.1.1)"
  echo "==> Mac host (PlotJuggler/FlightGear) IPv4: $MAC_HOST_IP"
  NET_OPTS=(--network host --add-host "host.docker.internal:${MAC_HOST_IP}")
else
  NET_OPTS=(--add-host host.docker.internal:host-gateway)
  for a in ${SIM_ARGS[@]+"${SIM_ARGS[@]}"}; do [[ "$a" == "--gdb" ]] && NET_OPTS+=(-p 1234:1234); done
fi

# ── Run the sim ───────────────────────────────────────────────────────────────
# Allocate a TTY only when we actually have one (interactive run). Headless /
# background / CI runs have no TTY, where `docker run -t` errors out
# ("the input device is not a TTY"); `-i` alone is fine there.
TTY_OPTS=(-i)
[[ -t 0 && -t 1 ]] && TTY_OPTS=(-it)

echo "==> Launching NPS sim ($AC_NAME) in container '$(pprz_image_name)'..."
pprz_run "${NET_OPTS[@]}" "${TTY_OPTS[@]}" -- python3 sim_anton.py ${SIM_ARGS[@]+"${SIM_ARGS[@]}"}

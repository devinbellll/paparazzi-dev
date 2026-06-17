#!/usr/bin/env bash
# ── Build the Paparazzi toolchain image (arm64-native) ────────────────────────
#
# Usage: ./build_image.sh [--amd64]
#
#   (default)  Build arm64-native image  -> paparazzi-build:latest
#   --amd64    Build amd64 image for emulation (parity / PPA-arm64 fallback).
#              Requires qemu binfmt; this script registers it via tonistiigi/binfmt
#              (needs a privileged container — may be blocked in the sandbox).
#
# NETWORK PREREQUISITE
#   The paparazzi PPA needs three domains that are blocked by the sandbox's
#   default-deny policy. Allow them on the HOST before building:
#       sbx policy allow network api.launchpad.net,keyserver.ubuntu.com,ppa.launchpadcontent.net
#   (ports.ubuntu.com / launchpad.net are already reachable. For an --amd64 build
#   also ensure archive.ubuntu.com,security.ubuntu.com are allowed.)
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

IMAGE="${PPRZ_IMAGE:-paparazzi-build:latest}"

# Pin the platform EXPLICITLY so a host DOCKER_DEFAULT_PLATFORM=linux/amd64 (left
# over from the old Rosetta workflow) can't sneak an amd64 build in — that pulls
# an amd64 OCaml ground segment and dies under Rosetta with
# "failed to open elf at /lib64/ld-linux-x86-64.so.2".
PLATFORM="--platform linux/arm64"

if [[ "${1:-}" == "--amd64" ]]; then
  PLATFORM="--platform linux/amd64"
  echo "==> Registering qemu binfmt for amd64 emulation..."
  docker run --privileged --rm tonistiigi/binfmt --install amd64
fi

echo "==> Building image '$IMAGE' ($PLATFORM)..."
# shellcheck disable=SC2086
docker build $PLATFORM -f Dockerfile.build -t "$IMAGE" .

echo ""
echo "==> Done: $IMAGE"
echo "    Test it:  ./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml"

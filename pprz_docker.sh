#!/usr/bin/env bash
# ── Ephemeral Paparazzi build/sim container dispatcher ────────────────────────
#
# Sourced by pprz.sh and sim.sh. Runs any command inside a throwaway
# arm64 container built from Dockerfile.build, with the repo bind-mounted at
# /workspace so all generated code/artifacts land back in the workspace tree
# (visible to both sandbox Claude and host VSCode).
#
# Public functions:
#   pprz_image_name           -> echoes the image tag
#   pprz_ensure_image         -> builds the image if it is missing
#   pprz_run [docker-opts] -- <cmd...>   -> run <cmd> in an ephemeral container
#
# WORKSPACE_DIR is provided by the sbx sandbox env; fall back to the repo root
# (the directory containing this script) when running elsewhere.
# ─────────────────────────────────────────────────────────────────────────────

PPRZ_IMAGE="${PPRZ_IMAGE:-paparazzi-build:latest}"

# Repo root = directory containing this script (resolve symlinks).
_PPRZ_SELF="${BASH_SOURCE[0]}"
PPRZ_REPO_ROOT="$(cd "$(dirname "$_PPRZ_SELF")" && pwd)"

# Host/sandbox path of the repo. Prefer the sandbox-provided WORKSPACE_DIR so
# bind mounts and IDE path-rewrites use the canonical host path.
PPRZ_HOST_ROOT="${WORKSPACE_DIR:-$PPRZ_REPO_ROOT}"

pprz_image_name() { echo "$PPRZ_IMAGE"; }

pprz_image_exists() {
  docker image inspect "$PPRZ_IMAGE" >/dev/null 2>&1
}

pprz_ensure_image() {
  if pprz_image_exists; then
    return 0
  fi
  echo "==> Image '$PPRZ_IMAGE' not found — building it (one-time)..." >&2
  "$PPRZ_REPO_ROOT/build_image.sh"
}

# Pick the --platform value to RUN the container with. Must match the image's
# own architecture, otherwise `docker run --platform <other>` treats the local
# image as absent and tries to PULL it ("Unable to find image ... locally").
# Honors an explicit PPRZ_PLATFORM override; else inspects the built image.
pprz_platform() {
  if [[ -n "${PPRZ_PLATFORM:-}" ]]; then
    echo "$PPRZ_PLATFORM"
    return 0
  fi
  local arch
  arch="$(docker image inspect --format '{{.Architecture}}' "$PPRZ_IMAGE" 2>/dev/null || true)"
  case "$arch" in
    amd64) echo "linux/amd64" ;;
    arm64) echo "linux/arm64" ;;
    *)     echo "linux/arm64" ;;   # sensible default before the image exists
  esac
}

# pprz_run [extra docker run opts...] -- <command...>
# Everything before the literal `--` is passed to `docker run`; everything after
# is the command executed inside the container.
pprz_run() {
  pprz_ensure_image

  local docker_opts=()
  while [[ $# -gt 0 && "$1" != "--" ]]; do
    docker_opts+=("$1")
    shift
  done
  [[ "${1:-}" == "--" ]] && shift   # drop the separator

  docker run --rm \
    --platform "$(pprz_platform)" \
    --user 1000:1000 \
    -v "$PPRZ_HOST_ROOT":/workspace \
    -e HOME=/tmp \
    -e PAPARAZZI_HOME=/workspace/paparazzi \
    -e PAPARAZZI_SRC=/workspace/paparazzi \
    -e CAML_LD_LIBRARY_PATH=/workspace/paparazzi/var/lib/ocaml/pprzlink:/workspace/paparazzi/sw/lib/ocaml \
    -e PPRZ_HOST_ROOT="$PPRZ_HOST_ROOT" \
    -e PPRZ_HOST_CC="${PPRZ_HOST_CC:-}" \
    -e CONF="${CONF:-}" \
    -w /workspace \
    ${docker_opts[@]+"${docker_opts[@]}"} \
    "$PPRZ_IMAGE" \
    "$@"
}

# ENAC Paparazzi Firmware Workspace

Headless firmware build environment for ENAC UAV Lab aircraft. Cross-compiles ARM Cortex-M firmware using the Paparazzi autopilot framework inside a containerized linux/amd64 toolchain.

---

## Workspace layout

```
/workspace/
├── paparazzi/              # Upstream Paparazzi build system (github.com/paparazzi/paparazzi)
│   └── conf/airframes/ENAC/
│       ├── conf_enac.xml   # Active fleet configuration (aircraft registry)
│       ├── fixed-wing/     # Fixed-wing airframe definitions
│       ├── hybrid/         # Hybrid airframe definitions
│       ├── quadrotor/      # Rotorcraft airframe definitions
│       └── rover/
├── enac_paparazzi/         # ENAC's fork (github.com/enacuavlab/paparazzi)
├── pprz.sh                 # The build tool (build / clean / rebuild / db / codegen / bootstrap)
├── sim.sh                  # NPS sim runner
├── build_image.sh          # Build the paparazzi-build toolchain image
└── Dockerfile.build        # arm64 toolchain image
```

---

## Building firmware

### Quick build

```bash
./pprz.sh build AIRCRAFT [TARGET]      # TARGET defaults to ap (autopilot)
```

Aircraft and target are **separate args** — there is no conf path to pass (the
fleet XML is fixed to `conf/airframes/ENAC/conf_enac.xml`). Examples:

```bash
./pprz.sh build ANTON              # ANTON autopilot firmware (ap)
./pprz.sh build CYFOAM ap          # explicit target
./pprz.sh build ANTON_MFC nps      # nps (sim) target
```

Output ELF lands at: `paparazzi/var/aircrafts/<AIRCRAFT>/<TARGET>/obj/<TARGET>.elf`

`build` is incremental and CMake-like: codegen reruns only when the airframe/conf
XML changed, and only the C files that changed recompile. Other commands:
`clean`, `rebuild`, `db` (regenerate `compile_commands.json` for clangd),
`codegen` (headers only), `bootstrap` (rebuild the OCaml ground segment).

### How it works

`pprz.sh` runs natively when a cross-compiler is on PATH (inside the toolchain
container), otherwise it dispatches itself into an ephemeral arm64 container
(repo bind-mounted at `/workspace`). Internally `build` runs
`make -f Makefile.ac AIRCRAFT=... CONF_XML=conf/... USE_LTO=no <target>.compile`
from the `paparazzi/` dir. The OCaml ground segment is built once (when the
generators / `var/include` are missing), not on every build. `USE_LTO=no` (ap
target) keeps incremental dev rebuilds fast; flip to `USE_LTO=yes` for release.

---

## Aircraft fleet

Defined in [paparazzi/conf/airframes/ENAC/conf_enac.xml](paparazzi/conf/airframes/ENAC/conf_enac.xml):

| Aircraft    | ac_id | Type       | Airframe file                              |
|-------------|-------|------------|--------------------------------------------|
| CYFOAM      | 6     | Hybrid     | airframes/ENAC/hybrid/cyfoam.xml           |
| ZAGI        | 12    | Fixed-wing | airframes/ENAC/fixed-wing/zagi_test.xml    |
| RoBoBee     | 2     | Quadrotor  | airframes/ENAC/quadrotor/robobee.xml       |
| ANTON       | 217   | Quadrotor  | airframes/ENAC/quadrotor/anton_indi_aruco.xml |
| CobraV2     | 11    | Quadrotor  | airframes/ENAC/quadrotor/cobraV2.xml       |
| MAYA        | 1     | Quadrotor  | airframes/ENAC/quadrotor/maya_outdoor.xml  |
| FALCON_V2   | 3     | Hybrid     | airframes/ENAC/hybrid/falcon_v2.xml        |
| PANACHE_1   | 4     | Fixed-wing | airframes/ENAC/fixed-wing/panache_1.xml    |
| CROW        | 5     | Quadrotor  | airframes/ENAC/quadrotor/crow_indoor.xml   |
| GOOSE       | 7     | Quadrotor  | airframes/ENAC/quadrotor/goose.xml         |

Airframe XML paths are relative to `paparazzi/conf/`.

---

## Container setup

The build runs inside a linux/amd64 container. The image is built in two stages.

### Prerequisites

- Docker with `--platform linux/amd64` support (Docker Desktop on Apple Silicon uses Rosetta 2 automatically)

### First-time build

```bash
cd /path/to/this/workspace   # parent of .devcontainer/

# Stage 1: build and tag the base image
docker build --platform linux/amd64 \
  -f .devcontainer/Dockerfile \
  -t claude-code-base \
  .devcontainer/

# Stage 2: build the Paparazzi toolchain layer
docker build --platform linux/amd64 \
  -f Dockerfile.paparazzi \
  -t claude-code-paparazzi \
  .

# Start the container
docker compose -f .devcontainer/docker-compose.yml up -d
```

Use the Makefile shortcut to run both build stages in order:

```bash
make build   # builds both stages
make up      # builds and starts
```

### VS Code / Cursor

Command Palette → **"Reopen in Container"** handles the compose step automatically. Run Stage 1 (`docker build ... -t claude-code-base`) once manually before opening the devcontainer.

### Rebuilding after Dockerfile changes

Re-run Stage 1 whenever `.devcontainer/Dockerfile` changes — Compose cannot build `claude-code-base` on its own.

---

## Gotchas

**Apple Silicon Macs** — The Paparazzi PPA has no arm64 packages. Docker Desktop emulates amd64 via Rosetta 2. Builds are slower but produce correct ARM Cortex-M firmware. The `platform: linux/amd64` in `docker-compose.yml` handles this.

**No USB flashing from inside the container on macOS** — Docker Desktop does not support USB passthrough. Build firmware in the container; flash from the host using `arm-none-eabi-` tools installed via Homebrew or DFU over USB.

**`claude-config` volume** — Holds the Claude auth token. Survives `docker compose down` but is deleted by `docker compose down -v`. Omit `-v` for routine restarts.

**`~/.gitconfig` mount** — The compose file mounts your host `~/.gitconfig` read-only so commits have the right author. Create a minimal one if your host doesn't have it.

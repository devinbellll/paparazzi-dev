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
├── build_fw.sh             # Firmware build wrapper script
├── Dockerfile.paparazzi    # Container image (extends claude-code-base)
└── .devcontainer/          # VS Code / Cursor devcontainer config
```

---

## Building firmware

### Quick build

```bash
./build_fw.sh AIRCRAFT CONF_XML [TARGET]
```

The default target is `ap` (autopilot). Examples:

```bash
# Build ANTON autopilot firmware
./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml

# Build CYFOAM with explicit target
./build_fw.sh CYFOAM paparazzi/conf/airframes/ENAC/conf_enac.xml ap

# Build PANACHE_1 fixed-wing
./build_fw.sh PANACHE_1 paparazzi/conf/airframes/ENAC/conf_enac.xml
```

Output ELF lands at: `paparazzi/var/aircrafts/<AIRCRAFT>/ap/obj/ap.elf`

### What the script does

1. Builds Paparazzi host tools (`make -j1 -C paparazzi/`) — serial to avoid dronecan submodule lock conflicts
2. Runs `make -f Makefile.ac AIRCRAFT=... CONF_XML=... USE_LTO=no <target>.compile`

`USE_LTO=no` is a permanent workaround for a gcc-arm-none-eabi 13.2 LTO ICE triggered by Rosetta 2 emulation (affects all builds in this container, including on Intel where it's a no-op).

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

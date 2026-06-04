# FlightGear 3D Visualization for NPS

How to get a live 3D view of ANTON (or any NPS aircraft) in FlightGear running on your Mac while the sim runs in the devcontainer.

Reference files: `sim_anton.py`, `paparazzi/conf/simulator/flightgear/bebop-set.xml`

---

## What it is

FlightGear acts as a **pure 3D viewer** — it renders an aircraft model that moves with the real physics from JSBSim running inside NPS. It receives position and attitude over UDP, does no physics of its own. The Bebop quadrotor model (`bebop.ac`) is the closest available match for ANTON.

---

## How to run

**1. Start the sim with FlightGear output:**
```bash
python3 sim_anton.py --fg
```

**2. In FlightGear → Additional Settings, paste:**
```
--fdm=null
--native-fdm=socket,in,60,,5501,udp
--aircraft=bebop
--aircraft-dir="/Users/you/.../paparazzi/conf/simulator/flightgear"
--disable-ai-models
--disable-real-weather-fetch
--disable-terrasync
--timeofday=noon
--lat=43.56 --lon=1.48 --altitude=300 --heading=0
```

Then click **Fly**. The Bebop model will appear and move with the sim.

> **Tip:** match `--lat`/`--lon` to the ANTON flight plan home position so FG initialises at the right location.

---

## How it works

```
┌─────────────────────────────┐      UDP (NET_FDM)        ┌──────────────────┐
│  devcontainer               │  ───────────────────────▶ │  Mac host        │
│                             │  192.168.65.254:5501       │                  │
│  JSBSim ──▶ NPS ──▶ simsitl │                           │  FlightGear      │
│  (physics)   (firmware)     │                           │  bebop.ac model  │
│                             │                           │  (visual only)   │
└─────────────────────────────┘                           └──────────────────┘
```

NPS serialises position + attitude into a `FGNetFDM` binary struct and sends it at ~30 Hz. FlightGear reads that struct and moves the aircraft model accordingly.

---

## Gotchas (hard-won)

These are non-obvious and will bite you again if you forget them.

### 1. `--fg_fdm` is required
NPS has two FG send modes:
- **GUI mode** (default) — sends `FGNetGUI`, version 8. FlightGear rejects it silently.
- **FDM mode** (`--fg_fdm`) — sends `FGNetFDM`, version 24. This is what `--native-fdm` expects.

`sim_anton.py --fg` already passes `--fg_fdm` automatically.

### 2. Pass the resolved IP, not the hostname
`nps_flightgear_init` uses `inet_addr()` which only accepts dotted-decimal IPs — it cannot resolve `host.docker.internal`. `sim_anton.py` resolves the hostname to `192.168.65.254` with `socket.gethostbyname()` before passing it to simsitl.

### 3. The container firewall blocks the Mac host IP
The devcontainer firewall allows outbound to the Docker bridge gateway (`172.24.0.1`) but not to the Mac host at `192.168.65.254`. `init-firewall.sh` was patched to auto-detect and allow `host.docker.internal` at startup.

### 4. Port must match on both sides
NPS sends to `FG_PORT = 5501`. FlightGear must listen on the same port: `--native-fdm=socket,in,60,,5501,udp`. A mismatch means packets arrive but nothing listens (Wireshark shows "Port unreachable" ICMP).

### 5. XML comments cannot contain `--`
`bebop-set.xml` is a FlightGear aircraft set file. XML comments with `--` inside them (e.g. CLI flags) break XML parsers and make everything appear commented-out in editors.

---

## Verifying the pipeline

| Check | Command |
|-------|---------|
| Packets leaving container | `watch -n0.5 'ss -nup \| grep 5501'` (run while sim is up) |
| Packets arriving on Mac | `nc -ul 5501 \| xxd \| head` (if nothing else is listening on 5501) |
| FG is listening | `nc -ul 5501` returns "Address already in use" → FG has the port |
| Correct FDM version | First 4 bytes of packet should decode to `24` via `ntohl` |

---

## Aircraft models available

Located in `paparazzi/conf/simulator/flightgear/`:

| Model | Description | Best for |
|-------|-------------|----------|
| `bebop` | Parrot Bebop quadrotor | ANTON, any quad |
| `hexa` | Generic hexacopter | 6-motor airframes |
| `easystar` | Fixed-wing glider | Fixed-wing NPS sims |
| `mikrokopter` | Older quad style | — |

To use a different model: add a `<model>-set.xml` following `bebop-set.xml` as a template, then change `--aircraft=<model>` in FG Additional Settings.

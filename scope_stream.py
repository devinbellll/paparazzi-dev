#!/usr/bin/env python3
"""
scope_stream.py — stream live signals to PlotJuggler as a Simulink-style scope.

Why this exists
---------------
The devcontainer is headless (no display server), so native-GUI plotters
(matplotlib / Qt / pyqtgraph, incl. paparazzi/sw/logalizer/rt_plotter.py) need
X11/VNC and lag over Rosetta. Instead we run PlotJuggler natively on the Mac
host and push a UDP/JSON snapshot of the signals to it — the same outbound-UDP
pattern sim_anton.py already uses to feed FlightGear (host.docker.internal:5501).
The container is the *sender*; PlotJuggler binds the UDP port on the Mac, which
has a real display. No port forwarding into the container is needed, and the
devcontainer firewall already allows all outbound UDP to the host.

Wire protocol
-------------
One JSON object per UDP datagram == one PlotJuggler sample. Nested dicts become
a '/'-separated tree in PlotJuggler (e.g. {"truth": {"phi": ...}} -> truth/phi).
Include a float-seconds timestamp field "t" and select it in PlotJuggler via
"use field as timestamp".

PlotJuggler host setup (on the Mac)
-----------------------------------
1. Install PlotJuggler:
     brew install plotjuggler
   or grab the macOS build from the facontidavide/PlotJuggler GitHub releases.
2. Streaming -> Start -> "UDP Server"
     Port:             9870
     Message Protocol: JSON
     [x] use field as timestamp -> t
3. Drag series from the tree into plot panels. PlotJuggler owns the rolling
   time window — nothing to configure on this side.

Quick self-test (validate transport + PlotJuggler config without the sim)
-------------------------------------------------------------------------
    python3 scope_stream.py --demo --host host.docker.internal
Two synthetic sine groups (wave/*) should appear in the tree and scroll live.

Library usage
-------------
    from scope_stream import ScopeStream
    scope = ScopeStream("host.docker.internal", 9870)
    scope.send({"t": time.time(), "truth": {"phi": phi, "theta": theta}})
"""

import json
import socket
import sys
import time


class ScopeStream:
    """Fire-and-forget UDP/JSON emitter for PlotJuggler. Never raises on send."""

    def __init__(self, host, port=9870, enabled=True):
        self.addr = (host, port)
        self.enabled = enabled
        self._warned = False
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if enabled else None

    def send(self, snapshot: dict):
        """JSON-encode one sample and push it. Silently drops on any error so a
        missing/closed PlotJuggler never disturbs the sim."""
        if not self.enabled or self._sock is None:
            return
        try:
            self._sock.sendto(json.dumps(snapshot).encode("utf-8"), self.addr)
        except OSError as e:
            if not self._warned:
                print(f"[scope] send failed ({e}); continuing silently", file=sys.stderr)
                self._warned = True

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None


# ── self-test ────────────────────────────────────────────────────────────────
def _demo(host, port, rate_hz=50.0):
    import math

    scope = ScopeStream(host, port)
    print(f"[scope] streaming synthetic sines to {host}:{port} at {rate_hz:.0f} Hz "
          f"(Ctrl-C to stop)")
    t0 = time.time()
    dt = 1.0 / rate_hz
    try:
        while True:
            t = time.time()
            phase = t - t0
            scope.send({
                "t": t,
                "wave": {
                    "sine_1hz":   math.sin(2 * math.pi * 1.0 * phase),
                    "sine_2hz":   math.sin(2 * math.pi * 2.0 * phase),
                    "cosine_1hz": math.cos(2 * math.pi * 1.0 * phase),
                },
                "ramp": {
                    "saw":      (phase % 1.0),
                    "triangle": abs((phase % 2.0) - 1.0),
                },
            })
            time.sleep(dt)
    except KeyboardInterrupt:
        scope.close()
        print("\n[scope] demo stopped")


def _arg(flag, default):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


if __name__ == "__main__":
    if "--demo" in sys.argv:
        _demo(_arg("--host", "host.docker.internal"), int(_arg("--port", "9870")))
    else:
        print(__doc__)
        print("Run with --demo [--host H] [--port P] to stream a synthetic self-test.")

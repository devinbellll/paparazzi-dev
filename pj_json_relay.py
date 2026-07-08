#!/usr/bin/env python3
"""
UDP JSON relay/sanitizer between server.ml's telemetry stream and PlotJuggler.

Why this exists
----------------
server.ml's UDP JSON emitter (paparazzi/sw/ext/pprzlink/lib/v2.0/ocaml/pprzLink.ml,
json_of_message) has two bugs that produce invalid JSON PlotJuggler cannot parse,
crashing its UDP Server plugin:

  1. String fields are interpolated into the JSON with NO escaping:
         String s -> sprintf "\"%s\": \"%s\"" field_name s
     Any backslash, quote, or control byte inside the field breaks the JSON.
     Real telemetry (lossy RF link) corrupts raw bytes far more often than the
     sim's loopback link, which is why this is "way worse" with real hardware.
     Example: `"PPRZ\\p` (unescaped backslash mid-string).

  2. Float fields are formatted with plain `%f`, and OCaml's `%f` prints NaN/Inf
     as the bare identifiers `nan`/`inf`/`-inf`, which are not valid JSON
     literals (JSON only knows numbers, true/false/null). Example:
     `"innov_vel": nan` (nlohmann's parser reports it as an unrecognized literal;
     it stops reading at "na", which is what shows up in the error text).

Both are upstream pprzlink bugs (sw/ext/pprzlink is vendored). Rather than patch
vendored code, this relay sits between server.ml and PlotJuggler, repairs (or
drops) malformed datagrams, and re-emits clean JSON. It also replaces the
`socat` hop that was already needed to get packets from the VM interface to
localhost, so it's a drop-in for that command.

Normalization (shared PlotJuggler schema)
-----------------------------------------
Besides repairing JSON, the relay rewrites every packet into a single schema
shared by the two telemetry sources, so ONE PlotJuggler layout works for any
aircraft, sim or real flight:

  ivy server  (server.ml, UDP/JSON 9870):  {"NAME (id)":  {"GUIDANCE_MFC": {...}}, "timestamp": t}
  nps_scope   (nps_scope.c, in-process):   {"NAME (sim)": {"TRUTH": {...}, "MFC_STAB/sp_phi": v, ...}, "timestamp": t}

both become:

  {"uav": {"MFC_GUIDANCE": {...}}, "timestamp": t}

i.e. the aircraft-specific root key is replaced by the fixed root "uav", and
MFC-related branches are renamed so they sort together in the curve tree:
STAB_MFC -> MFC_STAB, GUIDANCE_MFC -> MFC_GUIDANCE,
GUIDANCE_MFC_ACC2ATT / ACC2ATT -> MFC_ACC2ATT. Pass --raw to disable.

Usage (replaces the socat invocation):
    ./pj_json_relay.py --listen-addr 172.16.113.1 --listen-port 9870 \
                        --forward-addr 127.0.0.1  --forward-port 9870

Defaults match server.ml's own defaults (127.0.0.1:9870) so
`./pj_json_relay.py --listen-addr <vm-iface-ip>` is normally all you need.
"""
import argparse
import json
import socket
import sys
import time

ESCAPE_CHARS = set('"\\/bfnrtu')
# JSON's required single-character escapes for control bytes (json.dumps uses
# these too); anything else < 0x20 falls back to \u00XX.
SHORT_ESCAPES = {
    '\b': '\\b', '\f': '\\f', '\n': '\\n', '\r': '\\r', '\t': '\\t',
}
BARE_LITERALS = {
    "nan": "null", "-nan": "null",
    "inf": "null", "-inf": "null",
    "infinity": "null", "-infinity": "null",
}

# ── shared-schema normalization ──────────────────────────────────────────────
# Fixed root key replacing the aircraft-specific "NAME (id)" / "NAME (sim)"
# roots, so PlotJuggler layouts are aircraft- and feed-agnostic.
NORMALIZED_ROOT = "uav"
# First-path-segment renames applied inside the root. Covers both the ivy
# message names (messages.xml) and the legacy nps_scope registration names.
BRANCH_MAP = {
    "STAB_MFC": "MFC_STAB",
    "GUIDANCE_MFC": "MFC_GUIDANCE",
    "GUIDANCE_MFC_ACC2ATT": "MFC_ACC2ATT",
    "ACC2ATT": "MFC_ACC2ATT",
}


def normalize_obj(obj):
    """Rewrite a parsed telemetry packet into the shared schema (see module doc).

    Any top-level key whose value is a dict is treated as an aircraft root and
    merged under NORMALIZED_ROOT; scalar top-level keys ("timestamp") pass
    through. Branch keys (which may contain '/' separators, e.g. the flat
    "MFC_STAB/sp_phi" names nps_scope emits) get their first path segment
    renamed via BRANCH_MAP.
    """
    if not isinstance(obj, dict):
        return obj
    tree = {}
    passthrough = {}
    for key, val in obj.items():
        if not isinstance(val, dict):
            passthrough[key] = val
            continue
        for branch, sub in val.items():
            head, sep, rest = branch.partition("/")
            tree[BRANCH_MAP.get(head, head) + sep + rest] = sub
    if not tree:
        return obj
    out = {NORMALIZED_ROOT: tree}
    out.update(passthrough)
    return out


def repair_json_text(text: str) -> str:
    """Best-effort repair of the two known server.ml JSON bugs.

    Single pass over the text tracking whether we're inside a JSON string:
      - inside a string: stray backslashes (not starting a valid escape) get
        doubled, raw control bytes get escaped.
      - outside a string: bare nan/-nan/inf/-inf/Infinity/-Infinity tokens
        (server.ml's %f output for non-finite floats) get replaced with null.
    """
    out = []
    in_string = False
    i = 0
    n = len(text)
    word_start = None  # start index (in `out`) of a bare word outside a string

    def flush_word(end_idx):
        nonlocal word_start
        if word_start is None:
            return
        word = "".join(out[word_start:end_idx])
        if word.lower() in BARE_LITERALS:
            out[word_start:end_idx] = [BARE_LITERALS[word.lower()]]
        word_start = None

    while i < n:
        c = text[i]
        if in_string:
            if c == '\\':
                nxt = text[i + 1] if i + 1 < n else ''
                if nxt in ESCAPE_CHARS:
                    out.append(c)
                    out.append(nxt)
                    i += 2
                    continue
                # stray backslash -> escape it so the result is valid JSON
                out.append('\\\\')
                i += 1
                continue
            if c == '"':
                in_string = False
                out.append(c)
                i += 1
                continue
            if ord(c) < 0x20:
                out.append(SHORT_ESCAPES.get(c, "\\u%04x" % ord(c)))
                i += 1
                continue
            out.append(c)
            i += 1
            continue
        else:
            if c == '"':
                flush_word(len(out))
                in_string = True
                out.append(c)
                i += 1
                continue
            if c.isalpha() or c == '-':
                if word_start is None:
                    word_start = len(out)
                out.append(c)
                i += 1
                continue
            flush_word(len(out))
            out.append(c)
            i += 1
            continue
    flush_word(len(out))
    return "".join(out)


def _sanitize_ex(data: bytes, normalize: bool):
    """Return (clean_bytes_or_None, was_valid_json)."""
    try:
        obj = json.loads(data)
        was_clean = True
    except (json.JSONDecodeError, UnicodeDecodeError):
        was_clean = False
        text = data.decode("utf-8", errors="replace")
        repaired = repair_json_text(text)
        try:
            obj = json.loads(repaired)
        except json.JSONDecodeError:
            return None, False

    if normalize:
        obj = normalize_obj(obj)
    elif was_clean:
        return data, True  # raw mode, already valid: pass through untouched
    return json.dumps(obj).encode("utf-8"), was_clean


def sanitize(data: bytes, normalize: bool = True):
    """Return clean JSON bytes to forward, or None if the packet is unsalvageable.

    With normalize=True (the default) the packet is also rewritten into the
    shared aircraft-agnostic schema (see normalize_obj)."""
    return _sanitize_ex(data, normalize)[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen-addr", default="127.0.0.1", help="address to bind and receive server.ml's stream on")
    ap.add_argument("--listen-port", type=int, default=9870)
    ap.add_argument("--forward-addr", default="127.0.0.1", help="PlotJuggler UDP Server address")
    ap.add_argument("--forward-port", type=int, default=9870)
    ap.add_argument("--raw", action="store_true",
                     help="disable schema normalization: forward packets with their original "
                          "aircraft-specific root and message names instead of rewriting them "
                          "to the shared '%s' schema" % NORMALIZED_ROOT)
    ap.add_argument("-q", "--quiet", action="store_true", help="only print drop/repair counters, not every event")
    ap.add_argument("-v", "--verbose", action="store_true",
                     help="log every packet received and forwarded (source addr, size, payload preview), "
                          "plus a heartbeat every 2s while idle so a stuck/misdirected feed is obvious")
    ap.add_argument("--reuse-addr", action="store_true",
                     help="set SO_REUSEADDR before binding. Off by default: SO_REUSEADDR lets a second "
                          "process silently bind the same UDP port and steal packets with no error, which "
                          "looks identical to 'no packets are arriving'. Leave this off so a stale socat/"
                          "relay process still holding the port makes bind() fail loudly instead.")
    args = ap.parse_args()

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.reuse_addr:
        recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        recv_sock.bind((args.listen_addr, args.listen_port))
    except OSError as e:
        print(f"pj_json_relay: bind({args.listen_addr}:{args.listen_port}) failed: {e}", file=sys.stderr)
        if e.errno == 98:  # EADDRINUSE
            print("  Another process already owns this port -- that process is receiving the packets "
                  "instead of this one. Find it with:", file=sys.stderr)
            print(f"    sudo lsof -nP -iUDP:{args.listen_port}", file=sys.stderr)
            print("  Likely a leftover `socat` or a previous pj_json_relay.py instance. Kill it, then "
                  "rerun.", file=sys.stderr)
        sys.exit(1)
    # Without a timeout, recvfrom() blocks forever if nothing ever arrives and
    # the periodic [stats] line (below) never fires -- a silently-idle relay
    # looks identical to a working one. Wake up periodically so we always say
    # something, even "0 packets received".
    recv_sock.settimeout(2.0)

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    forward_to = (args.forward_addr, args.forward_port)

    print(f"pj_json_relay: listening on {args.listen_addr}:{args.listen_port}, "
          f"forwarding to {args.forward_addr}:{args.forward_port} "
          f"({'raw passthrough' if args.raw else 'normalized to /' + NORMALIZED_ROOT + ' schema'})",
          file=sys.stderr)

    n_ok = n_repaired = n_dropped = 0
    last_report = time.monotonic()
    last_packet_at = None

    try:
        while True:
            try:
                data, from_addr = recv_sock.recvfrom(65535)
            except socket.timeout:
                now = time.monotonic()
                if args.verbose:
                    idle_for = now - last_packet_at if last_packet_at else now - last_report
                    print(f"[idle] no packets in the last {idle_for:.1f}s "
                          f"(listening on {args.listen_addr}:{args.listen_port})", file=sys.stderr)
                if now - last_report > 10:
                    print(f"[stats] ok={n_ok} repaired={n_repaired} dropped={n_dropped}", file=sys.stderr)
                    last_report = now
                continue

            last_packet_at = time.monotonic()
            if args.verbose:
                print(f"[recv] {from_addr[0]}:{from_addr[1]} {len(data)}B: {data[:200]!r}", file=sys.stderr)

            clean, was_clean = _sanitize_ex(data, normalize=not args.raw)
            if clean is None:
                n_dropped += 1
                if not args.quiet:
                    preview = data[:120]
                    print(f"[drop] unparseable packet ({len(data)}B): {preview!r}", file=sys.stderr)
            else:
                if not was_clean:
                    n_repaired += 1
                    if not args.quiet:
                        print(f"[repair] fixed malformed packet ({len(data)}B)", file=sys.stderr)
                else:
                    n_ok += 1
                try:
                    send_sock.sendto(clean, forward_to)
                except OSError as e:
                    # e.g. ECONNREFUSED if nothing is listening on forward_to yet
                    print(f"[send-error] could not forward to {forward_to}: {e}", file=sys.stderr)
                else:
                    if args.verbose:
                        print(f"[send] -> {args.forward_addr}:{args.forward_port} {len(clean)}B", file=sys.stderr)

            now = time.monotonic()
            if now - last_report > 10:
                print(f"[stats] ok={n_ok} repaired={n_repaired} dropped={n_dropped}", file=sys.stderr)
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[stats] ok={n_ok} repaired={n_repaired} dropped={n_dropped}", file=sys.stderr)


if __name__ == "__main__":
    main()

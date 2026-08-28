"""Write the per-run provenance sidecar.

``<stem>.csv`` carries the numbers; ``<stem>.meta.json`` carries everything
about the run that the numbers cannot tell you -- which aircraft, which
firmware, what rate, how it was converted.

FIELD OWNERSHIP. Every key here is measured from the data or read off the log
header, or supplied explicitly by the operator. Nothing is defaulted to a
plausible value. A key that cannot be established is ABSENT, because a note
downstream is only trustworthy if an absent fact stays absent.

That is why ``firmware_commit`` is not guessed from the working tree: the SD
card was written by whatever was flashed at the time, which the checkout today
does not know. Pass --commit if you know it; otherwise the run honestly does
not record one.
"""

import datetime
import json
import os

SCHEMA = 1


def rate_of(times):
    """Sample rate measured from the timestamps, never assumed.

    Load-bearing: some committed flight exports are low-rate ground-station
    exports wearing the filename a full-rate conversion would produce. The
    only way to tell them apart is to measure.
    """
    if len(times) < 2:
        return None
    span = times[-1] - times[0]
    if span <= 0:
        return None
    return (len(times) - 1) / span


def build(source, original, times=None, **facts):
    meta = {
        "schema": SCHEMA,
        "source": source,
        "original_file": os.path.basename(original),
        "converted_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "converter": "mfcdata",
    }
    if times:
        meta["n_samples"] = len(times)
        meta["t_start"] = times[0]
        meta["t_end"] = times[-1]
        meta["duration_s"] = times[-1] - times[0]
        r = rate_of(times)
        if r is not None:
            meta["rate_hz"] = round(r, 3)

    # Only facts that were actually established. None means "not known", and
    # not known means the key does not appear.
    for k, v in facts.items():
        if v is not None and v != "":
            meta[k] = v
    return meta


def write(csv_path, meta):
    path = os.path.splitext(csv_path)[0] + ".meta.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, sort_keys=False)
        f.write("\n")
    return path

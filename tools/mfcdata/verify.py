"""Semantic verification: are the role bindings actually RIGHT?

``check`` compares the contract against messages.xml. It catches renamed
messages, dropped fields and unbound additions -- schema drift.

It cannot catch a binding that is wrong but valid, and that is the failure
mode that actually bit: ``ref_cmd`` was bound to ``sp_<axis>`` when the
firmware forms its error against ``sp_traj_<axis>``. Both are real fields, so
every name resolved and every plot rendered. The error traces were simply
wrong -- worst on yaw, where the derived error was out by 194% of the logged
range while looking entirely plausible.

The firmware logs ``err_<axis>`` as well as the signals it is computed from.
That redundancy is the only independent evidence available offline, so this
module spends it: derive ``y - ref_cmd`` and require it to match the logged
error. A mismatch means a binding or a sign convention is wrong.

Run it on a converted CSV before trusting any figure made from one.
"""

import csv

from . import contract

# Fraction of the logged error's own range. Generous, because the point is to
# catch a WRONG SIGNAL, which is off by order-one, not to police rounding.
TOL = 0.01


def _read(path, wanted):
    """column -> [float], reading only the columns asked for."""
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r, None)
        if not header:
            return {}
        idx = {c: header.index(c) for c in wanted if c in header}
        out = {c: [] for c in idx}
        for row in r:
            for c, i in idx.items():
                try:
                    out[c].append(float(row[i]))
                except (ValueError, IndexError):
                    out[c].append(float("nan"))
        return out


def branch(path, spec, source_name, branch_name):
    """Verify one branch. Returns a list of (axis, ok, detail)."""
    cols = contract.columns(spec, source_name, branch_name)
    if not {"y", "ref_cmd", "err"} <= set(cols):
        return [(None, None, "branch binds no err -- nothing to verify against")]

    wanted = cols["y"] + cols["ref_cmd"] + cols["err"]
    data = _read(path, wanted)

    src = contract.source(spec, source_name)
    axes = src["branches"][branch_name].get("axes", [])
    results = []

    for k, axis in enumerate(axes):
        yk, rk, ek = cols["y"][k], cols["ref_cmd"][k], cols["err"][k]
        if not all(c in data and data[c] for c in (yk, rk, ek)):
            results.append((axis, None, "columns absent from this file"))
            continue

        y, r, e = data[yk], data[rk], data[ek]
        n = min(len(y), len(r), len(e))

        # Skip rows where any of the three is absent. A full-rate conversion
        # emits one row per record, so a row carrying one message leaves the
        # others blank -- and a single incomplete row would otherwise NaN the
        # whole comparison and report a binding error that is not there.
        rows = [i for i in range(n)
                if not (y[i] != y[i] or r[i] != r[i] or e[i] != e[i])]
        if not rows:
            results.append((axis, None, "no rows with all three columns populated"))
            continue

        diff = max(abs((y[i] - r[i]) - e[i]) for i in rows)
        scale = max(max(abs(e[i]) for i in rows), 1e-12)
        frac = diff / scale

        ok = frac <= TOL
        skipped = n - len(rows)
        detail = "max |derived - logged| = {:.4g} ({:.1f}% of logged range)".format(
            diff, 100 * frac)
        if skipped:
            detail += "  [{} of {} rows incomplete, skipped]".format(skipped, n)
        results.append((axis, ok, detail))
    return results


def run(path, source_name="flight", spec=None, verbose=True):
    """Verify every firmware branch present in one converted CSV."""
    spec = spec or contract.load()
    src = contract.source(spec, source_name)

    n_fail = 0
    report = ["verify  {}".format(path), "source  {}".format(source_name)]

    for branch_name in src["branches"]:
        report.append("")
        report.append("[{}]".format(branch_name))
        for axis, ok, detail in branch(path, spec, source_name, branch_name):
            if ok is None:
                report.append("  --   {}  {}".format(axis or "", detail))
            elif ok:
                report.append("  OK   {}  {}".format(axis, detail))
            else:
                n_fail += 1
                report.append("  FAIL {}  {}".format(axis, detail))

    if n_fail:
        report += [
            "",
            "{} axis binding(s) disagree with the firmware's own logged error.".format(n_fail),
            "A role binding or a sign convention is wrong. Do not trust a figure",
            "made from this file until it is resolved -- it will look plausible.",
        ]

    if verbose:
        print("\n".join(report))
    return n_fail, report

"""Load the vendored signal role contract and resolve it to column names.

The contract (``contract/signals.json`` at the repo root) is the shared
vocabulary between this repo and the MATLAB side. It is JSON rather than YAML
because it is machine-checked from both runtimes and both read JSON with no
dependencies -- this repo is stdlib-only and MATLAB has no dependable YAML
reader.

Nothing here knows what a role MEANS. This module resolves role -> column
names; interpreting them is the plotting layer's job, which lives in MATLAB.
"""

import json
import os

COLUMN_ROOT = "uav"


def contract_path(start=None):
    """Locate the vendored contract by walking up from ``start``.

    Walks rather than hardcoding so the package works whether it is invoked
    from the repo root, from tools/, or from an effort folder.
    """
    d = os.path.abspath(start or os.path.dirname(__file__))
    while True:
        p = os.path.join(d, "contract", "signals.json")
        if os.path.isfile(p):
            return p
        parent = os.path.dirname(d)
        if parent == d:
            raise FileNotFoundError(
                "No contract/signals.json found walking up from "
                f"{start or __file__}. The contract is vendored into each "
                "producing repo; if it is missing, this repo is not set up "
                "to emit runs."
            )
        d = parent


def load(path=None):
    with open(path or contract_path(), encoding="utf-8") as f:
        return json.load(f)


def source(spec, name):
    """Resolve one source's bindings, following ``same_as`` indirection.

    flight and sitl are the same firmware emitting the same messages, so the
    contract records flight as ``same_as: sitl`` rather than duplicating the
    table. Fields set alongside ``same_as`` (notably ``time_column``) override.
    """
    src = dict(spec["sources"][name])
    base = src.pop("same_as", None)
    if base:
        merged = dict(spec["sources"][base])
        merged.update(src)
        src = merged
    return src


def columns(spec, source_name, branch):
    """role -> [column names], for one branch of one source.

    A role bound to a template string expands over the branch's axes; a role
    bound to a list is taken literally (u_total is per-actuator, not per-axis).
    """
    src = source(spec, source_name)
    b = src["branches"][branch]
    axes = b.get("axes", [])
    out = {}
    for role, binding in b["roles"].items():
        if isinstance(binding, list):
            names = list(binding)
        elif "<axis>" in binding:
            names = [binding.replace("<axis>", a) for a in axes]
        else:
            names = [binding]
        out[role] = [column(branch, n) for n in names]
    return out


def column(branch, field):
    """Qualify one field name into a full column path.

    A binding that ALREADY STARTS WITH "/" is a fully qualified column name
    and is returned verbatim. The "/uav/<branch>/" prefix below is built from
    the BRANCH, so every role in a branch would otherwise share one first
    path segment -- and a 6-DOF binding has to reach TRUTH, SP and EST at
    once. No branch name can express that, so such a branch spells its
    columns out in full. Must stay identical to the same escape hatch in
    qsim/+qsim/contract.m.
    """
    if field.startswith("/"):
        return field
    return "/{}/{}/{}".format(COLUMN_ROOT, branch, field)


def firmware_branches(spec):
    """Branches that come from firmware telemetry, with their message names.

    Only branches carrying a ``message`` key -- that is what ties a contract
    branch back to messages.xml, and it is what the conformance check walks.
    """
    src = source(spec, "sitl")
    return {
        name: b["message"]
        for name, b in src["branches"].items()
        if "message" in b
    }


def bound_fields(spec, branch):
    """Every field name the contract binds for one branch, axes expanded."""
    src = source(spec, "sitl")
    b = src["branches"][branch]
    axes = b.get("axes", [])
    fields = []
    for binding in b["roles"].values():
        if isinstance(binding, list):
            fields.extend(binding)
        elif "<axis>" in binding:
            fields.extend(binding.replace("<axis>", a) for a in axes)
        else:
            fields.append(binding)
    return fields


def unbound_fields(spec, branch):
    """Fields the contract deliberately does NOT bind, axes expanded.

    Declared rather than inferred: the check must be able to tell "we decided
    not to bind this" from "we forgot", and only the contract knows which.
    """
    src = source(spec, "sitl")
    b = src["branches"][branch]
    axes = b.get("axes", [])
    out = []
    for f in b.get("unbound", []):
        if "<axis>" in f:
            out.extend(f.replace("<axis>", a) for a in axes)
        else:
            out.append(f)
    return out

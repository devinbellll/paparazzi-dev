"""Conformance check: the contract against this repo's own upstream truth.

The contract is vendored as a copy into each producing repo. Copy-to-copy
drift is caught by diffing the copies. This catches the other kind, which the
diff cannot see: the copies agreeing with each other and all of them being
wrong about the code.

Here, upstream truth is ``messages.xml`` (the telemetry field definitions) and
``BRANCH_MAP`` in pj_json_relay (the message -> column-branch renames). If the
contract names a field the firmware does not emit, or a message that no longer
exists, this fails.

It also reports fields the firmware emits that the contract neither binds nor
lists as deliberately unbound -- not a failure, but the thing you want to know
about after a firmware change added a signal.
"""

import os
import sys
import xml.etree.ElementTree as ET

from . import contract

# Candidate locations for the generated messages.xml, best first. var/ is
# written by the build and is the one that matches the running firmware;
# conf/ is the checked-in source.
MESSAGES_CANDIDATES = [
    os.path.join("paparazzi", "var", "messages.xml"),
    os.path.join("paparazzi", "conf", "tools", "messages.xml"),
]


def find_messages(repo_root):
    for rel in MESSAGES_CANDIDATES:
        p = os.path.join(repo_root, rel)
        if os.path.isfile(p):
            return p
    return None


def message_fields(messages_path):
    """message name -> set of field names, from messages.xml."""
    root = ET.parse(messages_path).getroot()
    out = {}
    for m in root.iter("message"):
        name = m.get("name")
        if name:
            out.setdefault(name, set()).update(
                f.get("name") for f in m.findall("field")
            )
    return out


def branch_map():
    """BRANCH_MAP from pj_json_relay, which is where the renames actually live.

    Imported rather than duplicated. A second copy of this table is exactly
    the class of drift this module exists to catch.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    if repo not in sys.path:
        sys.path.insert(0, repo)
    from pj_json_relay import BRANCH_MAP  # noqa: E402
    return BRANCH_MAP


def run(repo_root=None, spec=None, verbose=True):
    """Check the contract against messages.xml. Returns (n_errors, report)."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = repo_root or os.path.dirname(os.path.dirname(here))
    spec = spec or contract.load(contract.contract_path(repo_root))

    errors, warnings, notes = [], [], []

    messages_path = find_messages(repo_root)
    if messages_path is None:
        # Not an error: messages.xml is build output and a fresh clone has
        # none. Saying so beats failing a check nobody can satisfy yet -- but
        # only if it is actually said. This path returned before the verbose
        # block below, so the CLI exited 0 in total silence and a skipped
        # check was indistinguishable from a clean one.
        report = ["SKIP  no messages.xml found -- build the ground segment "
                  "first, then this check has something to verify against."]
        if verbose:
            print(report[0])
        return 0, report

    fields = message_fields(messages_path)
    bmap = branch_map()

    notes.append("contract  v{} ({})".format(spec.get("version"), spec.get("updated")))
    notes.append("messages  {}".format(os.path.relpath(messages_path, repo_root)))

    for branch, message in contract.firmware_branches(spec).items():
        # The contract names the message; BRANCH_MAP must rename it to the
        # branch the contract uses for columns, or the two disagree about
        # what a column is called.
        mapped = bmap.get(message, message)
        if mapped != branch:
            errors.append(
                "BRANCH  contract branch {!r} claims message {!r}, but "
                "BRANCH_MAP sends that message to {!r}. Columns will be "
                "named /uav/{}/... and nothing will resolve."
                .format(branch, message, mapped, mapped)
            )

        if message not in fields:
            errors.append(
                "MESSAGE {!r} (contract branch {!r}) is not in messages.xml. "
                "Either the firmware dropped it or the contract is stale."
                .format(message, branch)
            )
            continue

        have = fields[message]
        want = contract.bound_fields(spec, branch)
        unbound = set(contract.unbound_fields(spec, branch))

        missing = [f for f in want if f not in have]
        for f in missing:
            errors.append(
                "FIELD   {}/{} is bound by the contract but not emitted by "
                "the firmware.".format(branch, f)
            )

        extra = sorted(have - set(want) - unbound)
        for f in extra:
            warnings.append(
                "UNBOUND {}/{} is emitted but neither bound nor declared "
                "unbound. Bind it, or add it to `unbound` to say the "
                "omission is deliberate.".format(branch, f)
            )

        if not missing:
            notes.append("OK      {} <- {}  ({} fields bound)"
                         .format(branch, message, len(want)))

    report = notes + warnings + errors
    if verbose:
        for line in report:
            print(line)
        print("\n{} error(s), {} warning(s)".format(len(errors), len(warnings)))
    return len(errors), report

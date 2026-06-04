#!/usr/bin/env bash
# Stop hook — token-discipline harness.
#
# Advisory reminder to capture a session retro in the Knowledge vault. This is
# intentionally NON-blocking: it prints context and exits 0. A Stop hook that
# returns a blocking decision can trap the session in a loop, so we never do.
set -uo pipefail

today="$(date +%F)"

cat <<EOF
Before wrapping up: if this session made non-trivial changes, write a retro to
Knowledge/Sessions/${today}-<topic>.md covering what changed, what you learned,
and what's next. (Skip for trivial sessions.)
EOF

exit 0

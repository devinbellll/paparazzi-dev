#!/usr/bin/env bash
# SessionStart hook — token-discipline harness.
#
# Keeps each repo's OpenWolf file index (.wolf/anatomy.md) fresh. Runs for the
# envelope and every registered satellite submodule. `scan --check` exits
# non-zero only when the index is stale, so the actual rescan runs lazily.
#
# Guarded by `command -v openwolf` and always exits 0 — a missing OpenWolf or a
# repo that was never `openwolf init`-ed must not block the session.
set -uo pipefail

command -v openwolf >/dev/null 2>&1 || exit 0

refresh() {
  local dir="$1"
  [[ -d "$dir/.wolf" ]] || return 0   # only refresh repos that opted in via `openwolf init`
  ( cd "$dir" && openwolf scan --check >/dev/null 2>&1 && openwolf scan >/dev/null 2>&1 ) || true
}

# Envelope root.
refresh "."

# Each satellite submodule path from .gitmodules.
if [[ -f .gitmodules ]]; then
  while read -r _ path; do
    [[ -n "$path" ]] && refresh "$path"
  done < <(git config --file .gitmodules --get-regexp '\.path$' 2>/dev/null || true)
fi

exit 0

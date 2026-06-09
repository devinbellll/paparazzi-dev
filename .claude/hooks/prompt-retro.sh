#!/usr/bin/env bash
# Stop hook — session retro guard.
# Blocks stop if memory.md is newer than the latest session retro (or no retro exists today).
# Loop-safe: only blocks when memory.md has been written since the last retro.
set -uo pipefail

today="$(date +%F)"
session_dir="$CLAUDE_PROJECT_DIR/Knowledge/Sessions"
memory_file="$CLAUDE_PROJECT_DIR/.wolf/memory.md"

# Find the most recent session file written today (if any)
latest_session="$(ls -t "$session_dir/${today}-"*.md 2>/dev/null | head -1)"

if [ -n "$latest_session" ]; then
  # A retro exists — only block if memory.md has been updated since it was written
  if find "$memory_file" -newer "$latest_session" -type f 2>/dev/null | grep -q .; then
    printf '{"decision":"block","reason":"memory.md has been updated since the last retro. Update Knowledge/Sessions/%s-<topic>.md with what changed since, then you can stop."}\n' "$today"
    exit 0
  fi
  exit 0
fi

# No retro yet today — block if memory.md was touched today (non-trivial session)
mem_date="$(stat -c %y "$memory_file" 2>/dev/null | cut -d' ' -f1)"
if [ "$mem_date" = "$today" ]; then
  printf '{"decision":"block","reason":"Before stopping: write a session retro to Knowledge/Sessions/%s-<topic>.md — what changed, what you learned, what is next. Pull content from .wolf/memory.md and .wolf/cerebrum.md."}\n' "$today"
  exit 0
fi

exit 0

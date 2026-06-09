#!/usr/bin/env bash
# Stop hook — session retro guard.
# Blocks stop if memory.md is newer than the latest session retro (or no retro exists today).
# Loop-safe: when memory.md is newer, checks whether the new entries are only session-file
# bookkeeping (post-write hook logging the retro write itself). If so, touches the retro
# to reset the mtime and exits cleanly — no false-positive block.
set -uo pipefail

today="$(date +%F)"
session_dir="$CLAUDE_PROJECT_DIR/Knowledge/Sessions"
memory_file="$CLAUDE_PROJECT_DIR/.wolf/memory.md"

# Find the most recent session file written today (if any)
latest_session="$(ls -t "$session_dir/${today}-"*.md 2>/dev/null | head -1)"

if [ -n "$latest_session" ]; then
  if find "$memory_file" -newer "$latest_session" -type f 2>/dev/null | grep -q .; then
    # memory.md is newer — check if the new entries are only retro-write bookkeeping.
    # Find the line number of the last Knowledge/Sessions/<today> entry in memory.md,
    # then inspect everything after it for substantive (non-bookkeeping) work.
    last_retro_line="$(grep -n "Knowledge/Sessions/${today}" "$memory_file" 2>/dev/null | tail -1 | cut -d: -f1)"
    if [ -n "$last_retro_line" ]; then
      new_work="$(tail -n +"$((last_retro_line + 1))" "$memory_file" | grep "^|" | grep -v "Session end")"
      if [ -z "$new_work" ]; then
        # Only post-write bookkeeping since the retro — reset mtime and allow stop
        touch "$latest_session" 2>/dev/null || true
        exit 0
      fi
    fi
    printf '{"decision":"block","reason":"memory.md has been updated since the last retro. Update Knowledge/Sessions/%s with what changed since, then you can stop."}\n' "$(basename "$latest_session")"
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

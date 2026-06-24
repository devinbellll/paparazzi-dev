#!/usr/bin/env bash
# .claude/hooks/prompt-retro.sh
# Stop hook — session retro guard.
#
# Fires only when the session was substantive: ≥3 new entries in
# .wolf/memory.md since the last retro written today.
#
# Dead-end escape: if RETRO_DEAD_END=1 is set in the environment,
# writes a minimal dead-end retro and exits cleanly instead of blocking.
# Usage: RETRO_DEAD_END=1 RETRO_TOPIC=<slug> RETRO_NOTE="<one line>" <stop command>
#
# Loop-safe: bookkeeping entries written by the retro hook itself
# (references to Knowledge/Sessions/) are excluded from the count.

set -uo pipefail

today="$(date +%F)"
session_dir="$CLAUDE_PROJECT_DIR/Knowledge/Sessions"
memory_file="$CLAUDE_PROJECT_DIR/.wolf/memory.md"
THRESHOLD=3

# ── Bail out early if memory file doesn't exist ──────────────────────────────
if [ ! -f "$memory_file" ]; then
  exit 0
fi

# ── 1. Find the most recent retro written today (if any) ─────────────────────
latest_session="$(ls -t "$session_dir/${today}-"*.md 2>/dev/null | head -1)"

# ── 2. Count substantive memory entries since the last retro ─────────────────
count_new_entries() {
  local since_line=0

  if [ -n "$latest_session" ]; then
    local retro_basename
    retro_basename="$(basename "$latest_session")"
    local last_retro_line
    last_retro_line="$(grep -n "$retro_basename" "$memory_file" 2>/dev/null \
      | tail -1 | cut -d: -f1)"
    [ -n "$last_retro_line" ] && since_line="$last_retro_line"
  fi

  if [ "$since_line" -gt 0 ]; then
    tail -n +"$((since_line + 1))" "$memory_file"
  else
    cat "$memory_file"
  fi \
    | grep "^|" \
    | grep -v "Knowledge/Sessions\|session retro\|Session end" \
    | wc -l
}

new_entries="$(count_new_entries)"

# ── 3. Below threshold — nothing to do ───────────────────────────────────────
if [ "$new_entries" -lt "$THRESHOLD" ]; then
  exit 0
fi

# ── 4. Dead-end escape ───────────────────────────────────────────────────────
if [ "${RETRO_DEAD_END:-}" = "1" ]; then
  topic="${RETRO_TOPIC:-dead-end}"
  note="${RETRO_NOTE:-Tried something that didn't work out. No changes committed.}"
  retro_path="$session_dir/${today}-${topic}.md"
  session_n=$(( $(ls "$session_dir/${today}-"*.md 2>/dev/null | wc -l) + 1 ))

  cat > "$retro_path" <<EOF
---
date: ${today}
effort:
session_n: ${session_n}
status: raw
---

## Dead end
${note}
EOF

  echo "Dead-end retro written: $(basename "$retro_path")" >&2
  exit 0
fi

# ── 5. Block and prompt ───────────────────────────────────────────────────────
session_n=$(( $(ls "$session_dir/${today}-"*.md 2>/dev/null | wc -l) + 1 ))

printf '{"decision":"block","reason":"Session had %d memory entries — write a retro before stopping.\n\nWrite to: Knowledge/Sessions/%s-<topic>.md  (session %d today)\n\nSections:\n  ## What happened\n  ## Outcomes  (✅ verified / ⚠️ unverified / ❌ dead end)\n  ## Open threads\n  ## Promotable to Knowledge/\n\nDead-end session with nothing to record?\nRe-run stop with: RETRO_DEAD_END=1 RETRO_TOPIC=<slug> RETRO_NOTE='\''<one line>'\''"}' \
  "$new_entries" "$today" "$session_n"
printf '\n'

exit 0

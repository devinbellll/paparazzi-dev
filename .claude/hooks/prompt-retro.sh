#!/usr/bin/env bash
# .claude/hooks/prompt-retro.sh

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
latest_session="$(ls -t "$session_dir/${today}-"*.md 2>/dev/null | head -1 || true)"

# ── 2. Count substantive memory entries since the last retro ─────────────────
count_new_entries() {
  local since_line=0

  # Search for the last memory row that references any today's session retro.
  # We match "Knowledge/Sessions/YYYY-MM-DD-" rather than a specific filename
  # because Claude writes the row before the filename is finalised.
  local last_retro_line
  last_retro_line="$(grep -n "Knowledge/Sessions/${today}-" "$memory_file" 2>/dev/null \
    | tail -1 | cut -d: -f1)"
  [ -n "$last_retro_line" ] && since_line="$last_retro_line"

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
  note="${RETRO_NOTE:-Tried something that did not work out. No changes committed.}"

  session_n=$(( $(ls "$session_dir/${today}-"*.md 2>/dev/null | wc -l) + 1 ))
  retro_path="$session_dir/${today}-${topic}.md"

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

  # Append a sentinel memory row so the counter resets on the next stop.
  printf "| %s | Session retro (dead-end) | Knowledge/Sessions/%s-%s.md | written | - |\n" \
    "$(date +%H:%M)" "$today" "$topic" >> "$memory_file"

  echo "Dead-end retro written: $(basename "$retro_path")" >&2
  exit 0
fi

# ── 5. Block and prompt ───────────────────────────────────────────────────────
session_n=$(( $(ls "$session_dir/${today}-"*.md 2>/dev/null | wc -l) + 1 ))

cat <<EOF
{"decision":"block","reason":"Session had ${new_entries} memory entries — write a retro before stopping.

Write to: Knowledge/Sessions/${today}-<topic>.md  (session ${session_n} today)

Sections:
  ## What happened
  ## Outcomes  (✅ verified / ⚠️ unverified / ❌ dead end)
  ## Open threads
  ## Promotable to Knowledge/

After writing the retro, append ONE row to .wolf/memory.md:
  | HH:MM | Session retro | Knowledge/Sessions/${today}-<topic>.md | written | ~NNN |
(This resets the counter so the next stop is not blocked.)

Dead-end session with nothing to record?
Re-run stop with: RETRO_DEAD_END=1 RETRO_TOPIC=<slug> RETRO_NOTE=<one line>"}
EOF

exit 0

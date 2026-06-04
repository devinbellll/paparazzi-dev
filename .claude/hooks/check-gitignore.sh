#!/usr/bin/env bash
# PreToolUse hook (Edit|Write|MultiEdit) — token-discipline harness.
#
# Blocks edits to a .gitignore that lives INSIDE a satellite repo. Satellites
# are independent repos with their own remotes; silently rewriting their
# .gitignore can hide source from Claude Code's navigation or leak local config
# upstream. The envelope-root .gitignore is fine to edit (closs manages it).
#
# Contract: read the hook payload as JSON on stdin, decide via the PreToolUse
# permissionDecision field. Fail open (allow) on any parse error so a missing
# jq or malformed payload never traps the session.
set -euo pipefail

payload="$(cat)"

command -v jq >/dev/null 2>&1 || exit 0

file_path="$(printf '%s' "$payload" | jq -r '.tool_input.file_path // empty' 2>/dev/null || true)"
[[ -n "$file_path" ]] || exit 0

# Normalise to a workspace-relative path (strip the /workspace/ mount prefix).
rel="${file_path#/workspace/}"
rel="${rel#./}"

# Only care about files literally named .gitignore.
[[ "$(basename "$rel")" == ".gitignore" ]] || exit 0

# Envelope-root .gitignore has no leading directory segment — allow it.
# Anything deeper (e.g. "satellite/.gitignore") is inside a satellite — deny.
if [[ "$rel" == */.gitignore ]]; then
  satellite="${rel%%/*}"
  reason="Refusing to edit ${satellite}/.gitignore: satellite repos own their own .gitignore (token-discipline Hard Rule). Ask the user before changing it."
  jq -n --arg r "$reason" '{
    hookSpecificOutput: {
      hookEventName: "PreToolUse",
      permissionDecision: "deny",
      permissionDecisionReason: $r
    }
  }'
  exit 0
fi

exit 0

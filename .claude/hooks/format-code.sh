#!/usr/bin/env bash
# PostToolUse hook (Edit|Write|MultiEdit) — token-discipline harness.
#
# Best-effort format of the single file Claude just edited. Dispatches by
# extension; every formatter is guarded by `command -v` so a stack without that
# tool is a silent no-op. This must NEVER fail the session — always exit 0.
set -uo pipefail

payload="$(cat)"

command -v jq >/dev/null 2>&1 || exit 0

file_path="$(printf '%s' "$payload" | jq -r '.tool_input.file_path // empty' 2>/dev/null || true)"
[[ -n "$file_path" && -f "$file_path" ]] || exit 0

case "$file_path" in
  *.js|*.jsx|*.ts|*.tsx|*.json|*.md|*.css|*.scss|*.html|*.yaml|*.yml)
    command -v prettier >/dev/null 2>&1 && prettier --write "$file_path" >/dev/null 2>&1 || true
    ;;
  *.py)
    command -v black >/dev/null 2>&1 && black --quiet "$file_path" >/dev/null 2>&1 || true
    ;;
  *.go)
    command -v gofmt >/dev/null 2>&1 && gofmt -w "$file_path" >/dev/null 2>&1 || true
    ;;
  *.rs)
    command -v rustfmt >/dev/null 2>&1 && rustfmt "$file_path" >/dev/null 2>&1 || true
    ;;
esac

exit 0

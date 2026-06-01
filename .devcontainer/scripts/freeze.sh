#!/usr/bin/env bash
# ── freeze.sh ─────────────────────────────────────────────────────────────────
# Snapshots the parts of your container state that live outside git:
#   - Installed apt packages (dpkg manifest)
#   - Global npm packages
#   - Global pip packages
#   - ~/.claude contents (settings, not auth tokens)
#   - Any dotfiles in $HOME worth keeping
#
# Output is a timestamped folder you can commit, archive, or just keep around.
# This is NOT a full container image export — it's a human-readable record of
# state so you can reconstruct the environment if needed.
#
# Usage (inside the container):
#   bash .devcontainer/scripts/freeze.sh
#
# Usage (from host):
#   docker compose -f .devcontainer/docker-compose.yml exec claude \
#     bash /workspace/.devcontainer/scripts/freeze.sh
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
FREEZE_DIR="/workspace/.devcontainer/snapshots/freeze_${TIMESTAMP}"

bold()   { echo -e "\033[1m$*\033[0m"; }
green()  { echo -e "\033[0;32m$*\033[0m"; }
yellow() { echo -e "\033[0;33m$*\033[0m"; }

bold "╔══════════════════════════════════════════╗"
bold "║         Devcontainer Freeze              ║"
bold "╚══════════════════════════════════════════╝"
echo ""
echo "  Output: $FREEZE_DIR"
echo ""

mkdir -p "$FREEZE_DIR"

# ── apt packages ──────────────────────────────────────────────────────────────
bold "→ Snapshotting apt packages..."
dpkg --get-selections > "$FREEZE_DIR/apt-packages.txt"
# Also save just the manually installed ones (cleaner for reconstruction)
apt-mark showmanual > "$FREEZE_DIR/apt-packages-manual.txt" 2>/dev/null || true
green "  ✓ apt-packages.txt (full)"
green "  ✓ apt-packages-manual.txt (manually installed only)"

# ── npm global packages ───────────────────────────────────────────────────────
bold "→ Snapshotting npm global packages..."
npm list -g --depth=0 --json 2>/dev/null > "$FREEZE_DIR/npm-global.json" || true
# Human-readable version too
npm list -g --depth=0 2>/dev/null > "$FREEZE_DIR/npm-global.txt" || true
green "  ✓ npm-global.json"
green "  ✓ npm-global.txt"

# ── pip packages ──────────────────────────────────────────────────────────────
if command -v pip3 &>/dev/null; then
  bold "→ Snapshotting pip packages..."
  pip3 freeze > "$FREEZE_DIR/requirements-freeze.txt" 2>/dev/null || true
  green "  ✓ requirements-freeze.txt"
fi

# ── Claude settings (not auth) ────────────────────────────────────────────────
bold "→ Snapshotting ~/.claude settings..."
CLAUDE_SNAPSHOT="$FREEZE_DIR/claude-settings"
mkdir -p "$CLAUDE_SNAPSHOT"

# Copy settings but explicitly exclude auth tokens
if [[ -f ~/.claude/settings.json ]]; then
  cp ~/.claude/settings.json "$CLAUDE_SNAPSHOT/settings.json"
  green "  ✓ settings.json"
fi
if [[ -f ~/.claude/CLAUDE.md ]]; then
  cp ~/.claude/CLAUDE.md "$CLAUDE_SNAPSHOT/CLAUDE.md"
  green "  ✓ CLAUDE.md (user-level)"
fi

# Explicitly skip credentials/auth files
yellow "  ~ Skipping auth tokens (credentials.json, .credentials) — these should not be backed up"

# ── Shell config / dotfiles ───────────────────────────────────────────────────
bold "→ Snapshotting dotfiles..."
DOTFILES_SNAPSHOT="$FREEZE_DIR/dotfiles"
mkdir -p "$DOTFILES_SNAPSHOT"

for dotfile in .bashrc .zshrc .zsh_history .bash_history .profile .vimrc .tmux.conf; do
  if [[ -f ~/$dotfile ]]; then
    cp ~/$dotfile "$DOTFILES_SNAPSHOT/$dotfile"
    green "  ✓ $dotfile"
  fi
done

# ── Node version ──────────────────────────────────────────────────────────────
bold "→ Recording tool versions..."
{
  echo "=== Snapshot: $TIMESTAMP ==="
  echo ""
  echo "--- OS ---"
  cat /etc/os-release 2>/dev/null || true
  echo ""
  echo "--- Node ---"
  node --version 2>/dev/null || echo "not installed"
  echo ""
  echo "--- npm ---"
  npm --version 2>/dev/null || echo "not installed"
  echo ""
  echo "--- Python ---"
  python3 --version 2>/dev/null || echo "not installed"
  echo ""
  echo "--- Claude Code ---"
  claude --version 2>/dev/null || echo "not installed"
  echo ""
  echo "--- Git ---"
  git --version 2>/dev/null || echo "not installed"
} > "$FREEZE_DIR/versions.txt"
green "  ✓ versions.txt"

# ── Write a reconstruction guide ──────────────────────────────────────────────
cat > "$FREEZE_DIR/HOW-TO-RESTORE.md" << 'EOF'
# How to restore from this snapshot

This snapshot captures state that lives outside git. Use it as a reference
when rebuilding the container or setting up a new machine.

## apt packages

To restore manually-installed packages on a fresh Ubuntu system:
```bash
xargs sudo apt-get install -y < apt-packages-manual.txt
```

## npm global packages

Install packages listed in npm-global.txt:
```bash
npm install -g <package>@<version>
```
Or use npm-global.json for exact versions.

## pip packages

```bash
pip3 install -r requirements-freeze.txt
```

## Claude settings

Copy `claude-settings/settings.json` to `~/.claude/settings.json`.
Copy `claude-settings/CLAUDE.md` to `~/.claude/CLAUDE.md` if present.

You will need to re-authenticate with `claude` — auth tokens are intentionally
not included in this snapshot.

## Dotfiles

Copy files from `dotfiles/` to `~/`.
EOF

green "  ✓ HOW-TO-RESTORE.md"

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
bold "Freeze complete."
echo ""
echo "  Location : $FREEZE_DIR"
echo "  Size     : $(du -sh "$FREEZE_DIR" | cut -f1)"
echo ""
yellow "Note: This snapshot does NOT include:"
yellow "  - Auth tokens (intentional — don't back these up to git)"
yellow "  - Your repo files (those are in git)"
yellow "  - The container image itself (rebuild from Dockerfile)"
echo ""
echo "Consider committing the snapshots/ folder or adding it to .gitignore"
echo "depending on whether you want these shared with the team."
echo ""

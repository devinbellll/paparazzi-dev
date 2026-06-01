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

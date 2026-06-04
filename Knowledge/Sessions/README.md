# Sessions

Session retros — a human-readable record of what each Claude Code session did.

One file per session, named `YYYY-MM-DD-<topic>.md` (e.g.
`2026-06-02-paparazzi-auth-fix.md`). Each retro covers:

- **Changed** — what was actually modified.
- **Learned** — anything surprising or worth remembering.
- **Next** — open threads for the following session.

Retros are prompted automatically at session end by the `Stop` hook
`.claude/hooks/prompt-retro.sh`) and by `closs session-end`. They complement
OpenWolf's `cerebrum.md`: cerebrum holds Claude's own machine-readable
learnings, this holds the human-readable narrative.

This README also keeps the directory tracked in git when otherwise empty.

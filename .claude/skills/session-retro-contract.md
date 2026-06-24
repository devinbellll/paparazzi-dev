# Session Retro Contract

**Scope:** This workspace (repo root).  
**Skill location:** `.claude/skills/session-retro-contract.md`  
**Rule:** This file defines the session retro format. The daily rollup skill
depends on this contract. Do not change section names or frontmatter keys
without updating `daily-rollup.md` to match.

---

## When to write a retro

The `prompt-retro.sh` stop hook fires when the session was substantive
(≥3 new entries in `.wolf/memory.md` since the last retro today). Write
a retro then, or any time you judge the session worth recording.

**Dead-end sessions:** If the session went nowhere — nothing committed, nothing
verified — write a dead-end retro. Do not skip it; the absence of progress is
still information and keeps the daily note honest.

---

## File naming

```
Knowledge/Sessions/YYYY-MM-DD-<topic>.md
```

`<topic>` is a short kebab-case label for the session's main thread.
Multiple sessions on the same day get distinct topic slugs.

---

## Format — standard retro

```markdown
---
date: 2026-06-17
effort:       # effort or experiment name if applicable, else blank
session_n: 1  # which session of this day (1-indexed)
status: raw   # always raw — this file is immutable after writing
---

## What happened
Narrative of the session — approach, pivots, what was tried.
Dead ends belong here. Be specific; future-you needs to reconstruct context.

## Outcomes
- ✅ Verified this session: <concrete fact confirmed to be true>
- ⚠️  Unverified / assumed: <thing that seemed to work but wasn't fully tested>
- ❌ Dead end: <what was tried and why it didn't work or was dropped>

## Open threads
- <specific thing the next session needs to pick up>

## Promotable to Knowledge/
- `<NN - Knowledge file name>`: <specific fact or section to add>
<!-- Leave blank if nothing is clearly promotable yet. -->
<!-- You write to Knowledge/ files manually — nothing does it automatically. -->
```

**Epistemic markers:**
- `✅` — confirmed working/true in this session. Not "I think so."
- `⚠️` — seemed to work but not fully verified.
- `❌` — definitively didn't work, or was dropped.
- When in doubt, use `⚠️`.

**Immutability:** Once written, a retro is never edited. It records what you
knew at the time. If a `✅` later proves wrong, the correction goes to the
relevant `Knowledge/NN - *.md` file — not back into the retro.

---

## Format — dead-end retro

```markdown
---
date: 2026-06-17
effort:
session_n: 1
status: raw
---

## Dead end
Tried <X>. <One sentence on why it didn't work or was abandoned>.
No changes committed.
```

---

## What the daily rollup reads

The daily rollup skill reads from each retro:
- Frontmatter: `date`, `session_n`
- `## Outcomes` — `✅` lines → daily note `## Today` checklist
- `## Outcomes` — `⚠️` and `❌` lines → daily note `## Open Threads`
- `## Open threads` — all bullets → daily note `## Open Threads`
- `## Promotable to Knowledge/` — all non-comment bullets → surfaced as suggestions
- Filename stem → `[[wikilink]]` in `## Linked Notes`

It does not read or interpret `## What happened`.

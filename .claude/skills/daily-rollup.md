# Daily Rollup Skill

**Location:** `.claude/skills/daily-rollup.md`
**Trigger:** "Run the daily rollup for [date / date range / all missing]"
**Depends on:** `.claude/skills/session-retro-contract.md`

---

## What this skill does

For one or more dates, reads all session retros and writes or updates a daily
note scaffold. It handles the mechanical assembly so the author only needs to
fill in TLDR and any personal context the retros can't provide.

---

## Invocation forms

```
Run the daily rollup for 2026-06-18
Run the daily rollup for 2026-06-18 through 2026-06-22
Run the daily rollup for all days missing a daily note
```

"All missing" means: dates that have at least one `Knowledge/Sessions/YYYY-MM-DD-*.md`
but no `Knowledge/Daily Notes/YYYY-MM-DD.md`, or one that contains only unmodified
template placeholders.

---

## Step 1 — Discover retros for the target date

Find all `Knowledge/Sessions/YYYY-MM-DD-*.md` where the date prefix matches.
Sort by `session_n` frontmatter field; fall back to filename alphabetical order.

**Pre-format retros** (no `## Outcomes` section, no `✅/⚠️/❌` markers): link
them in `## Linked Notes` marked `(pre-format)`. Extract `## What happened`,
`## Wins`, and `## Open threads` / `## Next` sections if present. Do not attempt
to infer epistemic status from prose.

---

## Step 2 — Section assembly rules

Read each section of each retro and assemble the daily note as follows.

### `## Today` checklist

One checkbox line per `✅` outcome across all retros, in session order.
Rewrite each line in plain English — no code symbols, no filenames, no
variable names, no shell syntax. Capture the intent and result, not the
implementation.

Good: `- [x] Got both controllers building and linking together cleanly`
Bad:  `- [x] ✅ Phase 0: de-conflicted MFC/INDI link-time symbol collision — both stabilizers co-link in ANTON_MFC`

If a `✅` line is already plain English (as you would have written it), use
it verbatim. Only rewrite when it contains code-level detail.

`⚠️` and `❌` outcomes do NOT go in `## Today`. They go in `## Open Threads`.

### `## Morning —` and `## Afternoon —`

Write a plain-English narrative of what happened during the session(s).
Source material: `## What happened` sections from all retros for this date.

Rules:
- No code symbols, no filenames, no variable names, no shell commands.
  Refer to things by what they are, not what they're called in code.
  "the thrust type mismatch" not "`vthz=-552960` from union bit reinterpret".
  "the controller initialisation function" not "`oneloop_mfc_init()`".
- One to three sentences per morning/afternoon block. Do not pad.
- If multiple sessions cover the same period, combine them into one narrative.
- If the retro has no temporal split, put everything in `## Morning —` and
  leave `## Afternoon —` blank.
- If retros are pre-format and have no `## What happened`, write one sentence
  per retro based on its title/goal section only.

### `## Questions Raised`

Default: leave blank.

Only add an item if ALL of the following are true:
- A question appears explicitly in a retro's `## Questions Raised` section
  (not inferred by the skill)
- It was unresolved across ALL sessions for this date
- It is consequential enough that the author would want to see it tomorrow

Maximum one item. If multiple qualify, pick the most consequential. Add a
parenthetical attribution: `← *retro-slug*`.

Do not add questions the skill infers or generates itself.

### `## Notes to Self`

Default: leave blank.

Only add an item if ALL of the following are true:
- Something critical was discovered that would be easy to forget
- It is not already captured in `## Open Threads` or `## Wins`
- It clearly affects future work in a non-obvious way

Maximum one item per day. Add attribution: `← *retro-slug*`.

Do not copy Claude's implementation notes, debugging breadcrumbs, or
anything that is only relevant within a session context. The test: would
the author care about this when opening the note in a week?

### `## Wins`

Copy verbatim from `## Wins` sections in retros, in session order. These
were written by the author in plain English — do not rewrite them.
If a win contains code-level detail the author wrote intentionally, keep it.

### `## Open Threads`

Collect from:
- `⚠️` lines from `## Outcomes`
- `❌` lines from `## Outcomes` (mark as dead end)
- All bullets from `## Open threads` sections

Rewrite in plain English using the same rule as `## Today`.
Attribute each item: `← *retro-slug*`

### `## Next Steps — Tomorrow / Backlog`

Copy verbatim from retro `## Next` / `## Open threads` / `## Next Steps`
sections. These reflect decisions already made — do not rewrite or summarise.
If multiple retros have next steps, combine without duplicating.
Also mirror into frontmatter `tomorrow:` and `backlog:` fields.

### `## Linked Notes`

One wikilink per retro, sorted by `session_n`:
```
- [[YYYY-MM-DD-topic]]  (session N)
- [[YYYY-MM-DD-topic]]  (session N, pre-format)
```

### `## Promotable to Knowledge/`

Copy verbatim from `## Promotable to Knowledge/` sections in retros.
Omit this section entirely if nothing was flagged.

---

## Step 3 — Write the daily note

### If no daily note exists

Write `Knowledge/Daily Notes/YYYY-MM-DD.md`:

```markdown
---
tags:
  - daily-log
date: YYYY-MM-DD
tomorrow: [verbatim from retro next steps, single line summary]
backlog: [verbatim from retro backlog items, comma separated if multiple]

---

> [!TLDR]
> One sentence. What was today actually about?

## Today
- [x] [plain English ✅ outcome]
- [x] [plain English ✅ outcome]

---

## Morning — [Rough Title, or Blank]
- [plain English narrative, 1–3 sentences, 1-5 bullets]

---

## Afternoon — [Rough Title, or Blank]
- [plain English narrative, 1–3 sentences, 1-5 bullets]

---

## Questions Raised
- [one item only if it earns its place, else blank]

## Notes to Self
- [one item only if it earns its place, else blank]

## Wins
- [verbatim from retro ## Wins]

---

## Next Steps

### Tomorrow
*(mirrors frontmatter — keep in sync)*
- [verbatim from retro]

### Backlog
*(mirrors frontmatter — keep in sync)*
- [verbatim from retro]

---

## Linked Notes
- [[YYYY-MM-DD-topic]]  (session N)

---

## Promotable to Knowledge/
- `NN - File`: fact  ← *retro-slug*
```

Leave TLDR blank with the placeholder text. Never fill it in.
Omit `## Open Threads` if empty. Omit `## Promotable` if nothing flagged.

### If a daily note already exists

Do not modify anything already written. Check whether all retros for this
date are already in `## Linked Notes`. If yes: report "already up to date."

If retros are missing, append at the end of the file:

```markdown

---
<!-- daily rollup appended YYYY-MM-DD -->

## Session retros (auto)

### Linked Notes (additional)
- [[YYYY-MM-DD-topic]]  (session N)

### Today (additional)
- [x] [plain English outcome]

### Morning / Afternoon (additional)
[plain English narrative of the additional session(s)]

### Wins (additional)
- [verbatim]

### Open Threads (additional)
- [item]  ← *retro-slug*

### Promotable to Knowledge/ (additional)
- `NN - File`: fact  ← *retro-slug*
```

---

## Step 4 — Report

```
Knowledge/Daily Notes/2026-06-18.md  [created | appended | already up to date]
  Retros: 5 (new-format: 4, pre-format: 1)
  ✅ outcomes: 6  →  Today checklist
  Wins: 3  →  verbatim
  Open threads: 2
  Questions Raised: 1 (justified: unresolved across all sessions)
  Notes to Self: 0
  Promotable suggestions: 2
```

---

## Integrity constraints

- Never write TLDR.
- Never write into `## Morning —` or `## Afternoon —` using code symbols,
  filenames, or variable names.
- Never populate `## Questions Raised` or `## Notes to Self` unless the item
  genuinely earns its place by the criteria above.
- Never write to any `Knowledge/NN - *.md` file.
- Never modify a session retro.
- Never overwrite author-written content in an existing daily note.
- If a fact wasn't present in a retro, it does not appear in the daily note.
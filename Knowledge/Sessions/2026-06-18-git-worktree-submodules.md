# Git Worktree Setup with Nested Submodules (2026-06-18)

## Goal

Create a host-side git worktree of `paparazzi_dev` for branch isolation (e.g. `feat/shadow-handoff`), where `paparazzi` is a large multi-level submodule with local unpushed commits.

## Problems Encountered and Fixes

### 1. Submodule URL points to container-internal path

`paparazzi_dev/.gitmodules` has `url = /workspace/paparazzi` — a path that only exists inside the build sandbox, not on the host.

**Fix:** After creating the worktree, override the URL locally before initializing:
```bash
cd paparazzi-dev-shadow-handoff
git config submodule.paparazzi.url \
  "/Users/devinbellll/Workspace/ENAC Workspace/Firmware/paparazzi_dev/paparazzi"
```

### 2. `file` transport blocked by Git security policy

Git 2.38.1+ blocks local `file://` clones by default (CVE-2022-39253 symlink attack mitigation).

**Fix:** Pass `-c protocol.file.allow=always` as a one-time flag:
```bash
git -c protocol.file.allow=always submodule update --init --recursive
```

No real risk here since you control both source and destination.

### 3. Nested submodule (`pprzlink`) pinned to unpushed local commit

`paparazzi` has `sw/ext/pprzlink` pinned to commit `0f2145d...` which exists locally but was never pushed to the upstream GitHub repo. `git submodule update --recursive` fails with:
```
fatal: remote error: upload-pack: not our ref 0f2145d...
```

**Fix (two steps):**

Step 1 — override the URL in `paparazzi/.git/config` (run from inside `paparazzi-dev-shadow-handoff/paparazzi/`):
```bash
git config submodule.sw/ext/pprzlink.url \
  "/Users/devinbellll/Workspace/ENAC Workspace/Firmware/paparazzi_dev/paparazzi/sw/ext/pprzlink"
```

Step 2 — if pprzlink was already partially cloned with GitHub as `origin`, fix the remote inside the submodule itself:
```bash
cd sw/ext/pprzlink
git remote set-url origin \
  "/Users/devinbellll/Workspace/ENAC Workspace/Firmware/paparazzi_dev/paparazzi/sw/ext/pprzlink"
git -c protocol.file.allow=always fetch origin
git checkout 0f2145d68814ba7dc853c437407dfcada7657641
```

Then finish from `paparazzi/`:
```bash
cd ..
git -c protocol.file.allow=always submodule update --init --recursive
```

## Key Insight: Worktrees Share `.git/config`

The top-level worktree (`paparazzi-dev-shadow-handoff`) shares the `.git` directory with `paparazzi_dev`. Running `git config` from the worktree root writes to the shared config. The `paparazzi` submodule has its own independent `.git/` (it was freshly cloned), so config changes inside it are isolated.

## Complete Working Recipe

```bash
# On host, from paparazzi_dev/
git worktree add -b feat/my-feature ../paparazzi-dev-my-feature

cd ../paparazzi-dev-my-feature
git config submodule.paparazzi.url \
  "/Users/devinbellll/Workspace/ENAC Workspace/Firmware/paparazzi_dev/paparazzi"
git -c protocol.file.allow=always submodule update --init

# If pprzlink (or any other nested submodule) fails with "not our ref":
cd paparazzi
git config submodule.sw/ext/pprzlink.url \
  "/Users/devinbellll/Workspace/ENAC Workspace/Firmware/paparazzi_dev/paparazzi/sw/ext/pprzlink"
cd sw/ext/pprzlink
git remote set-url origin \
  "/Users/devinbellll/Workspace/ENAC Workspace/Firmware/paparazzi_dev/paparazzi/sw/ext/pprzlink"
git -c protocol.file.allow=always fetch origin
git checkout <commit>
cd ../..
git -c protocol.file.allow=always submodule update --init --recursive
```

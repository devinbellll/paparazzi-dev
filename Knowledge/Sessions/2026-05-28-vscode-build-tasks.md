# Session — 2026-05-28: VSCode Build Tasks and IDE Integration

## What changed
- Knowledge update: added documentation for control system workflows
- Wired Makefile Tools extension for ANTON/ANTON_MFC IntelliSense and builds
- Added VSCode build tasks (`Makefile Tools.Makefile`) for incremental firmware compilation
- Created launch tasks for running ANTON sim (ANTON and ANTON_MFC targets)
- Saved VSCode build status (semi-working — builds execute, debug integration pending)

## Bugs fixed
- None (configuration and integration work)

## What I learned
- Makefile Tools extension works well for embedded Cortex-M projects once configured
- VSCode launch tasks can invoke the Paparazzi build pipeline directly
- IntelliSense requires proper compile_commands.json generation (via gen_compile_db.sh)

## What's next
- Full VSCode debugging support (gdb integration for firmware)
- Verification of build artifacts (.elf file output)

# Project instructions

- Keep replies short. Report changes, files, verification and the next track test.
- Inspect relevant files first. Make the smallest safe change; reuse helpers.
- Do not rename unrelated files, delete user data or overwrite frozen release ZIPs.
- Keep both sketch files at the same revision. Target the stock 3pi+ 32U4 OLED
  Standard with four AAA NiMH cells and the Arduino Leonardo board definition.
- Preserve immediate line-loss stops, motor bounds, button stops and battery checks.
- Do not upload to the robot, increase speeds blindly or claim a simulated lap
  establishes physical grip, safety or competition performance.
- The large-track failure reported on 2026-10-19 was **Map mismatch**, not Map full.
  The mat has S-bends and crossings. R42's larger buffer range is not a diagnosis
  of that failure. A corner was also taken incorrectly once.
- Run `python tests/run_tests.py --focused` and the Leonardo build after firmware
  changes. Use the pinned versions in README.md. Record flash and static RAM.
- Use paired models only for a concrete regression; retain known failing baselines.
- Keep credentials and local compiler/build files outside Git. Sync tested source,
  release artifacts and test results to the private repository when authorized.

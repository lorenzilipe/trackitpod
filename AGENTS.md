# Repository Guidelines

## Documentation and Workflow

- Start with [STATE.md](STATE.md) for the current checklist. Move a claimed item to **In progress** with your name. Link its spec, plan, or evidence when available. Keep the context section brief and retire completed items once the result is recorded.
- [The product spec](docs/product.md) describes the surfer's experience. [The pilot spec](docs/pilots/01-phone-tracking.md) defines the demo, scope, and acceptance criteria.
- Keep technical design and rationale in [subsystem specs](docs/specs/README.md). Put measurements, results, and links to clips or logs in [experiment records](docs/experiments/README.md).
- Use `$brainstorming` to develop specs together. Load `$interview-me` for its question phase, then return to brainstorming after the interview summary is confirmed. Save accepted technical designs in `docs/specs/<subsystem>.md`.
- Use `$writing-plans` to create an implementation plan under `docs/superpowers/plans/` for an approved piece of work. Use `$executing-plans` to implement the plan in the current session. Keep the plan's detailed steps in the plan and link it from the corresponding checklist item in `STATE.md`.
- [Repository-local skills](.agents/skills/README.md) are shared through Git. When a skill refers to `superpowers:<name>`, use the corresponding `$<name>` skill in Codex. Adapt references to agent tools and models to the Codex session.

## Firmware Layout

- `firmware/tag_unit/src/main.cpp` reads GPS fixes and broadcasts them over the HC-12 radio.
- `firmware/base_station/src/main.cpp` receives tag positions, handles calibration, and drives the camera's pan and tilt.
- `firmware/lib/common/` contains shared GPS, radio, display, motor, and tracking math code. Put shared headers in `include/common/` and implementations in `src/`. Both PlatformIO projects load this library through `lib_extra_dirs`.
- `README.md` describes the hardware, calibration workflow, and troubleshooting.

## Build and Hardware Checks

Both projects target the Seeed Studio XIAO ESP32-C3 with Arduino C++. From the repository root, build each target with PlatformIO.

```sh
pio run -d firmware/base_station
pio run -d firmware/tag_unit
```

With the matching board connected, add `-t upload` to the build command. From that project's directory, run `pio device monitor` to read its serial output at 115200 baud.

The current tree has no automated test suite or coverage target. Build both targets when shared code changes. For radio or tracking changes, verify that the base receives valid tag fixes, calibrates, tracks, and stops issuing new movements when packets go stale. Report which checks ran and which still need hardware.

## Code and Contributions

Use four spaces, same-line opening braces, and the `trackit` namespace for shared code. Follow the existing `PascalCase` type names, `lowerCamelCase` functions, and `k`-prefixed constants. Keep units explicit in names such as `bearingDeg` and `timestampMs`.

Keep commit subjects short and descriptive, as in the existing history. In pull requests, describe the affected firmware targets and validation. Document wiring, serial settings, or packet format changes when relevant.

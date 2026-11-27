# Repository Guidelines

## Documentation and Workflow

- `STATE.md` is this repository's shared, committed checklist. Keep it even when its sections are empty, and track claimed and upcoming work there rather than in a `PLAN.md`. This overrides any personal `STATE.md` or `PLAN.md` rules.
- Start with [STATE.md](STATE.md) for the current checklist. Move a claimed item to **In progress** with your name. Link its spec, plan, or evidence when available. Keep the context section brief and retire completed items once the result is recorded.
- [The product spec](docs/product.md) describes the surfer's experience. [The pilot spec](docs/pilots/01-phone-tracking.md) defines the demo, scope, and acceptance criteria.
- Keep technical design and rationale in [subsystem specs](docs/specs/README.md). Put measurements, results, and links to clips or logs in [experiment records](docs/experiments/README.md).
- Use `$brainstorming` to develop specs together. Load `$interview-me` for its question phase, then return to brainstorming after the interview summary is confirmed. Save accepted technical designs in `docs/specs/<subsystem>.md`.
- Use `$writing-plans` to create an implementation plan under `docs/superpowers/plans/` for an approved piece of work. Use `$executing-plans` to implement the plan in the current session. Keep the plan's detailed steps in the plan and link it from the corresponding checklist item in `STATE.md`.
- [Repository-local skills](.agents/skills/README.md) are shared through Git. When a skill refers to `superpowers:<name>`, use the corresponding `$<name>` skill in Codex. Adapt references to agent tools and models to the Codex session.

## Communication

Write for a developer who understands the project but does not have your entire working context loaded into their head.

1. Keep the reader oriented. When referring to a component, function, stage, record, or concept, briefly say what it is and where it fits when that may not be obvious or fresh.
2. Explain concretely. Prefer specific files, functions, code paths, data flows, or examples over vague abstractions.
3. Show how you know. For important findings, point to the relevant files, code, tests, logs, or observations. Include the actual line when it materially helps.
4. Use plain English first. Briefly explain unfamiliar or important technical terms in the context of this project.
5. Connect details to the bigger picture: what they mean, why they matter, and how they relate to the problem being solved.
6. Do not make the reader reconstruct old context. When something from earlier becomes relevant again, briefly re-establish it.
7. Narrate your intent during exploration: what you are trying to determine, why you are looking somewhere, what you learned, and where that leads next.
8. Use the `asd-ste100` skill as soft guidance when communicating important information.

## Agent Tool Use

- When a tool call accepts a description, always fill it in with 5-12 words on its intent (why, not syntax), using concrete nouns from the task. For chained commands, describe the overall goal.
- Write real newline characters, never literal `\n` escape sequences, in file contents, heredocs, notebook cells, and JSON source strings. Read the result back to verify.
- Modify files with the editing tools. Use scripted text replacement only to copy and paste code. Make one edit per conceptual change, not one per line.

## Comments and Docstrings

Write for a future developer with no knowledge of the current task, plan, or conversation. These rules override any skill instruction to the contrary.

- Put one concise comment before each meaningful logical block to explain its purpose, not what individual lines do.
- Never reference the agent conversation, implementation process, plan sections, task steps, or temporary artifacts.
- Use Google-style docstrings: a brief purpose, then `Args`, `Returns`, and `Raises`. Do not restate obvious type hints or narrate the implementation.
- Prefer clear names, and still use block comments for readability.

## Branching

- Build new features and major changes on a separate branch, then merge it into `main` when the whole change is complete.
- Small changes can be committed directly to `main`.

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

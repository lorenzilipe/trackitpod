# TrackIt Pod skills

Codex discovers these skills from `.agents/skills/` when a session starts in the repository. Claude Code finds the same files through the `.claude/skills` symlink, so add and edit skills here only. Invoke a skill by its folder name, such as `$brainstorming` in Codex or `/brainstorming` in Claude Code.

The Superpowers workflow skills and supporting files come from [obra/superpowers at 22f0b46](https://github.com/obra/superpowers/tree/22f0b463e7a5e579904dd6211a520321035526f7/skills). See [the upstream MIT license](LICENSE.superpowers).

`$interview-me`, `$asd-ste100`, and `$excalidraw-diagram` come from [Felipe's shared skill source](https://github.com/lorenzilipe/.claude-config/tree/main/skills). `asd-ste100` is [upstream work](https://github.com/danyuchn/asd-ste100-skill) under the MIT license in its folder.

Rendering a preview with `$excalidraw-diagram` needs [uv](https://docs.astral.sh/uv/) and a one-time setup, described in its `SKILL.md`.

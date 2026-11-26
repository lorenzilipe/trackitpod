---
name: interview-me
description: Use when the user says "interview me", "spec this out", "scope this", "let's align on", or similar; when a requested idea, implementation, or plan is large or ambiguous enough that building it now would mean guessing at what they want; or when superpowers:brainstorming reaches its question-asking phase.
---

# Interview Me

Align with the user on an idea, implementation, or plan, and scope it, before any work starts. The interview runs in chat, in rounds of numbered multiple-choice questions. Never use the AskUserQuestion tool for this skill.

**Core principle:** facts are Claude's job, decisions are the user's. Look up everything you can; ask only what changes what gets built.

## Before Round 1

1. **Research first.** Read the code, files, docs, and conversation context that bear on the request. Use subagents (model `sonnet` or `haiku`) for broad lookups. Never ask the user a fact you could find.
2. **Map the decisions as a tree.** Each decision can unlock others that depend on it. The **frontier** is every open decision whose prerequisites are settled.
3. **If you started this skill on your own** (the user did not ask for an interview), the first line of Round 1 is: `Reply "skip" to go straight to work.` On "skip", end the skill and do the task.

## Each Round

Ask the **whole frontier** in one round. A question whose answer depends on another question that is still open waits for a later round. After the user answers, recompute the frontier and ask the next round.

A round has these parts, in order:

1. **Recap** (every round after the first): a few bullets of what the user's last answers settled.
2. **Questions**, numbered continuously across rounds (Round 1 is Q1–Q5, Round 2 starts at Q6). Separate questions with `---`.
3. **My calls** (only if there are any): decisions you made without asking, each with a one-line reason, so the user can object.

### The open "why" question

If the user's reason for wanting this is not yet clear, Round 1 starts with one open question about it, with no options. Options steer people toward a menu instead of their own intent. Every other question is multiple choice.

While the why is open, treat it as a prerequisite like any other decision. Every question whose answer the why could change (features, design, scope, approach) waits for the next round. Round 1 holds the why question plus only the questions the why cannot change.

### Question template

```markdown
### Q7: <short title>

**Context:** <why this decision matters and what you found: file paths, functions, quotes, constraints>

- **A. <Bold subtitle> (Recommended).** <What this option means. Trade-off: what it costs or risks compared with the others.>
- **B. <Bold subtitle>.** <What this option means. Trade-off: ...>
- **C. <Bold subtitle>.** <What this option means. Trade-off: ...>

**Recommendation: A.** <One or two sentences on why.>
```

Rules for the template:

- **Context** explains everything specific to this project, codebase, data, or decision, as fully as the decision needs. Skip general knowledge that a working developer already has (what Python, git, or a JOIN is).
- **Every option states its trade-offs** whenever trade-offs exist. This applies to every question, not only to "how do we build it" questions.
- **Use as many options as the decision has.** Usually 2–4; more is fine when the choices are real.
- **Mark the recommendation twice:** `(Recommended)` inside the option's bold subtitle, and the bold `**Recommendation: X.**` line after the options.
- **No emojis** in the round.
- **No "Other" option and no note about free text.** The user can always answer in their own words.

The user answers in shorthand like `7A 8C 9B` plus notes. Treat notes and custom answers as decisions.

### Approaches

The question about the overall way to build the thing (the approach) is a normal question. Ask it as soon as it is on the frontier, which is usually after the goal and the why are settled. Give each approach full trade-offs, grounded in what the project already does.

## Ending the Interview

The interview ends when the frontier is empty, or when the user says "go", "good enough", or similar. On an early end, take your recommendation for every open question and list those under **Assumed**.

Then post the summary:

```markdown
## Summary: <topic>

### Goal and why
### Decisions
### In scope
### Out of scope
### Assumed
### Open risks
```

Leave out sections that are empty. Ask the user to confirm or correct it. After they confirm:

1. Ask whether to save the summary to a file, or keep it in chat.
2. Ask: "Start on this now?" Do not start work until they say yes.

## Inside superpowers:brainstorming

When this skill runs as brainstorming's question phase, it replaces brainstorming's one-question-at-a-time conversation. After the user confirms the summary, return to brainstorming at "Size the Work" and continue from there (design doc, builder check, approval, writing-plans). Use the confirmed summary as the content of the design doc; the approach question's options and answer fill its **Approaches** section. Skip the "save it?" and "Start on this now?" questions, because brainstorming's own steps replace them.

## Common Mistakes

| Mistake | Fix |
|---|---|
| Asking one question per turn | Ask the whole frontier each round. |
| Asking a question that depends on another open question in the same round | Move it to a later round. |
| Options with no trade-offs | Add what each option costs or risks. |
| Asking the user for a fact that is in the code or docs | Look it up, then state it in **Context**. |
| Using the AskUserQuestion tool | Write the round in chat. |
| Restarting question numbers at Q1 each round | Continue numbering from the last round. |
| Starting work after the summary is confirmed | Ask "Start on this now?" first. |

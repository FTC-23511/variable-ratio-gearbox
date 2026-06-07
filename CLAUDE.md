# CLAUDE.md — Variable-Ratio Gearbox Sim

Project instructions for Claude Code. Read this first.

## What this project is

An FTC drivetrain simulator that quantifies the time benefit of a variable gear
ratio (multi-gear shifter / CVT) vs a single optimized ratio, over representative
match paths. **Single self-contained `index.html`** — HTML + CSS + vanilla JS,
Plotly via CDN, no build step, no backend. Opens by double-click in any browser.

Plan / spec:
- [`ftc_variable_ratio_build_brief.md`](./ftc_variable_ratio_build_brief.md) — stack, UI, defaults, ranges, scope, build order.
- [`ftc_variable_ratio_sim_spec.md`](./ftc_variable_ratio_sim_spec.md) — physics engine: state, per-timestep solve order, control policies.

## Workflow — the routine (standard across all my projects)

This repo uses the shared **routine workflow**. It is integral, not optional —
treat `docs/ROUTINE.md` as authoritative alongside this file.

- **`docs/ROUTINE.md`** — source of truth for the prep → ship → report cycle, the
  auto-merge vs approval-required tier rules, and the hard safety rules. Edit §4
  to match this repo's real sensitive surfaces.
- **`docs/BACKLOG.md`** — the work queue. Top of "Next up" ships first.
- **`/prep-backlog`** — interactively queue work (asks before writing).
- **`/run-routine`** — run one cycle now: prep, then ship the top item end to end.
- **`/human-task-list`** — compact list of what only *you* need to do.

When asked to "do the next thing" / "ship a task" / "process the backlog," run
`/run-routine`. When asked "what do I need to do," run `/human-task-list`.

## Conventions

- Default branch: `main`. Feature branches: `routine/<slug>` (routine) or
  `<type>/<slug>` (manual).
- Verify before pushing: open `index.html` in a browser and run the §6 acceptance
  self-tests from the build brief (stall, free speed, convex baseline, energy
  balance, ordering, brownout, traction). No build/test tooling — the file is the
  whole app.
- Keep it a single file: no bundler, framework, server, or backend (build brief §1, §7).
- BACKLOG state changes commit directly to `main`; feature work goes via PR.

## Safety (also enforced in `.claude/settings.local.json`)

Never force-push, `reset --hard`, weaken CI, commit secrets/`.env*`, or run
irreversible data operations. Prefer reversible actions; verify state before
anything destructive. See `docs/ROUTINE.md` §5 for the full list.

# FTC Variable-Ratio Drivetrain Sim

A single-file browser tool that quantifies the **time benefit of a variable gear
ratio** (multi-gear shifter or CVT) versus a single optimized ratio, over a set
of representative FTC match paths. Brownout / traction / power-limited DC physics
on a goBILDA 5000-series core, mecanum (N=4).

## Run it

Open **`index.html`** in any modern browser — double-click it. No build step, no
server, no install. Charts via Plotly CDN (needs internet on first load).

## What it does

- **Three architectures** compared on the same paths: single fixed ratio
  (optimized), multi-gear shifter (geometric ratios, configurable `k`, shift
  dead-time), and CVT (continuous ratio, efficiency slider).
- **Fair baseline**: the single-gear ratio is re-optimized for every config (and
  at every sweep point) — it picks the one `G_fixed` that minimizes total weighted
  time across the whole path set.
- **Outputs**: summary table (Δ time vs baseline, best `G_fixed`, flags), per-run
  traces (`v(t)`, `I(t)`, `V_bus(t)`, motor operating point on the torque-speed
  curve), integrated **loss budget** (battery+wire / copper / iron / friction /
  drivetrain / useful), and a **sweep** of benefit vs any of: segment length, μ,
  mass, gear count `k`, `eta_cvt`, `R_sys`, `I_sw`.
- **Acceptance self-tests** (build brief §6) run from the UI — stall, free speed,
  convex baseline, energy balance (incl. the Ke≠Kt iron term), architecture
  ordering, brownout, traction.

## Docs

- [`ftc_variable_ratio_build_brief.md`](./ftc_variable_ratio_build_brief.md) — stack, UI, defaults, ranges, scope.
- [`ftc_variable_ratio_sim_spec.md`](./ftc_variable_ratio_sim_spec.md) — physics engine: state, per-timestep solve order, control policies.
- [`CLAUDE.md`](./CLAUDE.md) / [`docs/ROUTINE.md`](./docs/ROUTINE.md) — dev workflow.

## Scope (v1)

1D straight segments with stops. No backend, DB, field render, strafing model, or
per-wheel dynamics — see build brief §7.

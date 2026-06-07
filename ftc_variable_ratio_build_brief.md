# FTC Variable-Ratio Sim — Build Brief (agent instructions)

Read `ftc_variable_ratio_sim_spec.md` first — that is the physics engine (state, per-timestep solve order, control policies). This doc is everything *around* it: stack, UI, defaults, paths, ranges, acceptance, scope. Build the whole thing end-to-end. Make no design decisions that aren't already decided here; where a number is given, use it as the default and expose it as editable.

---

## 1. Stack — decided, do not deviate

- **Single self-contained `index.html`.** HTML + CSS + vanilla JS in one file. No build step, no server, no framework. Opens by double-click in any modern browser.
- **Charts: Plotly.js via CDN** (`cdn.plot.ly`). All sim runs client-side in JS — the math is light.
- No backend, no database, no login, no bundler. If config persistence is wanted, `localStorage` only (optional, not required).

Rationale: team tool, must be shareable and runnable with zero setup.

---

## 2. UI — single page, four regions

**A. Config panel** (collapsible groups, number inputs unless marked slider):
- *Robot*: `m_base`, `N`, `r_wheel`, `mu` (slider), `Crr`, `eta_drive` (slider)
- *Motor*: **fixed = goBILDA 5000-series** (see §5 / spec §2). Display the constants read-only; not user-editable.
- *Electrical*: `V_oc, R_sys, V_bn, I_sw`
- *Architectures*: `m_trans` for each of fixed / shifter / CVT; shifter `k` + `t_shift`; CVT `G_min, G_max, eta_cvt` (slider) + grid resolution
- *Sim*: `dt`, ratio-optimizer sweep range + resolution

**B. Path editor**: editable list of named paths, each a list of segments `{length_m, end: stop|through}`. Preload the default set (§4). Add/remove segments and paths. Optional per-path weight (default 1).

**C. Run controls**:
- `Run comparison` → runs all three architectures over the full path set.
- `Run sweep` → dropdown to pick one swept variable + range/points; recomputes the full comparison at each point (**re-optimize the single-gear baseline at every sweep point** — fairness).

**D. Results**:
- *Summary table*: total time per architecture across the path set, Δ vs single-gear baseline (s and %), best `G_fixed` found, brownout/slip flags raised.
- *Traces* (pick a path + architecture): `v(t)`, `I(t)`, `V_bus(t)`, and motor operating point traced on its torque-speed curve.
- *Loss budget*: stacked bar — battery+wire / copper / friction / iron / useful, integrated over the run (from spec §3-J).
- *Sweep chart*: benefit (Δ%) vs swept variable, one line per architecture.

Use clean, legible styling — grouped cards, readable defaults visible on load, results update without page reload.

---

## 3. Decided numerics & rules

- `dt = 0.001 s` for single-run traces. For sweeps, allow coarser `dt = 0.003 s` for speed.
- **Segment timeout guard**: if the robot hasn't covered `length_m` within 10 s sim time, abort that run, set `unreachable_flag` ("gear too tall / brownout lock"). Never infinite-loop.
- **CVT ratio search**: grid-scan `G` over `[G_min, G_max]` at 50 points per timestep, pick max clamped `F_drive` (accel mode). This is the perf hot spot — keep it tight.
- **Single-gear optimizer**: sweep `G_fixed` over `[G_min, G_max]` at 40 points, run full path set each, pick min total weighted time.
- **Shifter ratios**: geometric spacing — `G_i = G_min * (G_max/G_min)^(i/(k-1))` for `i = 0..k-1`. Special-case `k=1` → single ratio. Geometric = equal speed-ratio steps (standard gearbox practice).
- Guard all divisions: `r_wheel, R_sys, (k-1)` never zero.
- Negative current = back-driving/braking, allowed (spec step B).

---

## 4. Default path set (FTC field ≈ 3.66 m square) — preload these

Segments are 1D straight runs; `S` = stop at end, `T` = pass through.

- **Path A — "Short cycle"** (tight cyclic scoring): `0.6S, 0.6S, 0.6S, 0.6S, 0.6S`
- **Path B — "Cross-field traverse"** (exposes top-speed benefit): `3.0S, 3.0S`
- **Path C — "Mixed match"** (realistic blend): `1.5S, 0.8S, 2.4S, 0.5S, 1.2S`

Equal weight default. Metric = sum of weighted total times across the set. Keep B in the set deliberately — short fields hide any top-speed advantage, so one long path is needed to surface it.

Turns: model as a fixed time penalty per turn, default `0 s` (off in v1). Strafing: not modeled — note in UI that it only shifts effective `mu`/`eta`.

---

## 5. Default slider/sweep ranges

| param | min | max | default |
|---|---|---|---|
| `mu` | 0.3 | 1.2 | 0.85 |
| `eta_cvt` | 0.50 | 0.98 | 0.85 |
| `eta_drive` | 0.50 | 0.95 | 0.70 |
| `k` (gears) | 2 | 6 | 3 |
| `t_shift` (s) | 0.00 | 0.40 | 0.10 |
| `R_sys` (Ω) | 0.02 | 0.20 | 0.07 |
| `I_sw` (A) | 5 | 30 | 20 |
| `m_base` (kg) | 8 | 20 | 13 |
| `m_trans` (kg) | 0 | 4 | fixed 0.3 / shifter 1.2 / CVT 1.5 |
| `G_min` | — | — | 8 |
| `G_max` | — | — | 30 |

**Motor — FIXED, goBILDA 5000-series bare core. Hard-code, not user-editable:**
`R_m=1.304 Ω, Kt=0.01611 N·m/A, Ke=0.01922 V·s/rad, I_free=0.25 A` (derived in spec §2 from `V_nom=12, I_stall=9.2, tau_stall=0.1442 N·m, w_free=607.4 rad/s` tested). Note `Kt ≠ Ke` deliberately — step B uses `Ke`, step E uses `Kt`.

`G_min/G_max` are total motor-shaft → wheel reduction. Since the constants are **bare core (6000 rpm)**, `G` = goBILDA internal planetary ratio × external stages.

---

## 6. Acceptance self-tests — agent runs these before declaring done

1. **Stall**: `v=0, d=1` → `I = V_oc/(R_m + N·R_sys)`, clamped by `I_sw` and `I_bn`. With defaults ≈ 7.9 A/motor, `V_bus ≈ 10.3 V` (> `V_bn`). Force positive, finite.
2. **Free speed**: set `v` so `Ke·w_motor ≈ V_oc` → `I→0`, `F→~0`. No sign blowups.
3. **Convex baseline**: total-time vs `G_fixed` is smooth with a single minimum. Jagged ⇒ bug.
4. **Energy balance**: `∫P_in ≈ ∫(P_batt_wire + P_copper + P_friction + P_iron + P_mech)` within ~2%. The `P_iron = (Ke−Kt)·I·w` term is required to close the sum — omitting it leaves a ~19%-scaled residual and flags a real bug.
5. **Ordering**: with `m_trans=0` and `eta_cvt=1`, CVT time ≤ shifter time ≤ single time on every path. Violation ⇒ control-policy bug.
6. **Brownout**: raise `R_sys` until `brownout_flag` trips → current caps, force drops, no NaN.
7. **Traction**: high `mu` ⇒ `slip_flag` rarely set; very low `mu` ⇒ slip dominates launch.

Print a pass/fail line for each on a debug panel or console.

---

## 7. Scope guardrails — do NOT build in v1

- No backend, DB, auth, or accounts.
- No 2D field render or animation — paths are 1D segment lists.
- No strafing model, no per-wheel dynamics, no thermal derating over a match.
- No multi-user, no export-to-cloud. CSV download of results is fine if cheap.

Ship the working single file. Keep it quick.

---

## 8. Build order

1. Engine core (spec §3 A–F) + acceptance tests 1–2.
2. Clamps + integration (spec §3 G–I); single-segment sprint plotting `v(t)`.
3. Arch 1 + brake/cruise + baseline optimizer (spec §5.2) + test 3.
4. Path set + weighted total-time metric (§4).
5. Arch 2 shifter, Arch 3 CVT + test 5.
6. Full UI wiring, sweep harness, loss-budget logging (§7 of spec) + tests 4,6,7.

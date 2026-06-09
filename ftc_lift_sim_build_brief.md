# FTC Variable-Ratio LIFT Sim — Build Brief (Claude Code)

Goal: add a **Lift** tab to the existing live site (repo `FTC-23511/variable-ratio-gearbox`). The lift engine is the **drivetrain engine with 4 deltas** — reuse, do not rewrite the motor/electrical/brownout/loss core. Every number below was validated against a reference implementation; **treat §6 acceptance targets as hard pass/fail**. Where a number is given, it is the default and is exposed editable.

Read the drivetrain `ftc_variable_ratio_sim_spec.md` first — §2 (motor constants), §3 (solve order A–J), §3-J (loss budget). This brief only documents what changes and what to build around it.

---

## 0. Where this goes — decided

- Existing site is a single-file vanilla page. **Refactor to a tab shell if not already one**: top nav with `Drivetrain` (existing sim, untouched) + `Lift` (new). Shared CSS/Plotly theme. Keep it one self-contained `index.html`, no build step, no framework. Plotly via `cdn.plot.ly`.
- Do not regress the drivetrain tab. The lift tab is additive and reuses the same motor/electrical/loss functions — factor those into shared helpers if they aren't already.

**How to wrap the existing sim (do this first, before adding lift logic):** Read the current `index.html` end-to-end and inventory its top-level element IDs and its global JS function/var names. Then: (1) wrap the existing body content verbatim in `<section class="tab" id="tab-drive">` — do **not** rename its IDs or globals, so it keeps running untouched. (2) Add a `<nav>` with two buttons and a sibling `<section class="tab" id="tab-lift">`. (3) Tab switch = toggle a `.on` class to show/hide, and on show **fire `window.dispatchEvent(new Event('resize'))`** — Plotly charts laid out while their div is `display:none` render at zero size until a resize. (4) **Namespace all new lift code** inside one IIFE/object and **prefix every new element ID** (e.g. `lf_m_pay`, `lf_p_kin`) so nothing collides with the drivetrain globals/IDs. (5) Keep a single Plotly CDN `<script>`. (6) If the drivetrain sim auto-runs heavy work on load, leave it as the default tab so behavior is unchanged; gate the lift sim's first run to when its tab is shown (or a button press). Verify the drivetrain tab still produces identical output after wrapping before writing any lift code.

---

## 1. Stack — do not deviate

Same as drivetrain: single `index.html`, HTML+CSS+vanilla JS, Plotly CDN, all client-side, no backend/DB/bundler. Runnable by double-click. `localStorage` only if config persistence wanted (optional).

---

## 2. Physics — drivetrain engine, 4 deltas

The lift = the **same per-timestep DC + brownout solve** (drivetrain spec §3 A–F), same clamps step C/D, same loss budget §3-J. Changes:

| # | Drivetrain | Lift |
|---|---|---|
| D1 | Traction clamp (§3-G): `F=clamp(F,±μmg)`, slip_flag | **Removed.** Force reacts through the spool, not the floor. Ceiling = motor stall / current limit only. Low-gear torque is **kept**, not wasted. |
| D2 | Rolling resist `F_roll=Crr·m·g·sign(v)` | **Constant gravity** `F_grav=m_lift·g` opposing up-motion the *whole stroke* (a body force — does **not** flip sign with v). Plus small constant guide friction `F_fric` (flips with v). |
| D3 | Path = segment list, traction-limited brake | **Single vertical stroke** `0→L`. Metric = time to reach `x≥L` at `d=1` throughout. **No brake phase** (both archs hit the end-stop identically; omitting it does not move the ratio comparison). |
| D4 | `w_motor = v·G/r_wheel` | `w_motor = v·G/r_eff`, `r_eff = r_spool·n_rig`. Cascade rigging folds into one effective radius. Carriage force `F = N·τ·G·η/r_eff`. |

Add one loss term the drivetrain budget may already have: **gearbox** `P_gear = F·v·(1−η)/η` (the `(1−η)` mechanical loss). Required to close energy (see §6 test 4).

### Lift solve order (per timestep, run in order; `d=1` in accel)
```
re = r_spool * n_rig
m  = m_carriage + m_pay
Fg = m * 9.81
A  w_motor = v * G / re
B  I = (d*V_oc - Ke*w_motor) / (R_m + d*N*R_sys)
C  I_bn = (V_oc - V_bn)/(N*R_sys);  I = clamp(I, -I_sw, min(I_sw, I_bn))
D  V_bus = V_oc - N*I*R_sys;  brownout_flag = V_bus < V_bn
E  tau = Kt*(I - sign(I)*I_free)
F  eta = (arch==CVT)? eta_cvt : eta_drive
   F_drive = N*tau*G*eta/re                 # NO traction clamp
H  fric_dir = (v!=0)? sign(v) : sign(F_drive - Fg)
   F_net = F_drive - Fg - F_fric*fric_dir
I  a = F_net/m;  v += a*dt;  x += v*dt;  t += dt
J  loss log (integrate):
   batt_wire += (N*I)^2*R_sys*dt
   copper    += N*I^2*R_m*dt
   friction  += N*Kt*I_free*abs(w_motor)*dt
   iron      += N*(Ke-Kt)*I*w_motor*dt
   gearbox   += F_drive*v*(1-eta)/eta*dt
   useful    += F_drive*v*dt
   p_in      += V_oc*(N*I)*dt
```
Guard divisions: `r_eff, R_sys, eta, (G_max−G_min)` never zero. Negative `I` (back-driving) allowed.

**Segment/stroke timeout guard:** if `x<L` after `t>10 s` sim time, abort → `unreachable_flag` ("gear too tall / can't lift / brownout lock"). Never infinite-loop.

---

## 3. Decided constants, defaults, ranges

**Motor — FIXED goBILDA 5000 bare core (hard-code, read-only display, identical to drivetrain):**
`R_m=1.304 Ω, Kt=0.01611 N·m/A, Ke=0.01922 V·s/rad, I_free=0.25 A`. `Kt≠Ke` deliberate (iron loss; Ke in step B, Kt in step E). Bare core 6000 rpm → `G` = planetary × external.

| param | min | max | default | note |
|---|---|---|---|---|
| `m_carriage` (kg) | 0 | 20 | 1.5 | moving mass; **frame-mounted gearbox excluded** |
| `m_pay` (kg) | 0 | 24 | 0.5 | payload |
| `N` | 1 | 4 | 2 | lift motors |
| `r_spool` (m) | — | — | 0.018 | effective output radius |
| `n_rig` | 1 | 4 | 1 | cascade multiplier (folds into r_eff) |
| `F_fric` (N) | 0 | 40 | 6 | constant guide friction |
| `L` (m) | 0.1 | 2.4 | 0.7 | stroke |
| `eta_drive` (slider) | 0.50 | 0.95 | 0.70 | fixed/shifter |
| `eta_cvt` (slider) | 0.50 | 0.98 | 0.85 | CVT headline knob |
| `V_oc` (V) | — | — | 13.6 | fresh pack |
| `R_sys` (Ω) | 0.02 | 0.30 | 0.07 | brownout ceiling |
| `V_bn` (V) | — | — | 9.0 | brownout threshold |
| `I_sw` (A) | 5 | 30 | 20 | per-motor cap |
| `G_min` | — | — | 2 | total reduction floor |
| `G_max` | — | — | 20 | total reduction ceiling |
| `t_shift` (s) | 0.00 | 0.40 | 0.10 | shifter dead-time |
| `dt` (s) | — | — | 0.001 run / 0.003 sweep | semi-implicit Euler |

**Decided modeling choices (state in UI, expose where noted):**
- Gearbox mass **frame-mounted → NOT added to lifted mass** (true for a winch lift; for a self-hang it would be lifted — out of scope here).
- **Backdrivable spool** modeled (gravity pulls back down at zero torque) — worst case for the variable-ratio argument. Non-backdrivable lead-screw holding not modeled.
- Guide friction constant (no static/kinetic split).

---

## 4. Control policies (3 architectures)

**Arch 1 — single fixed `G`.** `d=1` accel to `x≥L`. Outer optimizer (§5) picks `G_fixed`.

**Arch 2 — 2-speed shifter** `{G_lo, G_hi}`. Each step pick the gear giving max instantaneous `F_drive` (run steps A–F at `d=1` for each gear, argmax). On gear change start `shift_timer=t_shift`; while `>0`: **F_drive=0** (clutch cut), decrement, then commit. Count shifts. **Optimize the pair** over the grid (do not pin to bounds) — fair.

**Arch 3 — CVT (ceiling)** `G` continuous. Each step **grid-scan `G` over [G_min,G_max] at 50 points, run A–F, pick max clamped `F_drive`** (max accel). `eta_cvt` scales all CVT force.

---

## 5. Outer loop — fairness (identical to drivetrain)

Re-optimize the single gear for **every** config / sweep point: sweep `G_fixed` over `[G_min,G_max]` at 40 points, run the stroke each, pick min time-to-`L`. That is the honest baseline. Benefit = `(single − arch)/single`. For sweeps, report **ratio-only (set `eta_cvt=eta_drive`)** so the chart isolates the ratio benefit from the efficiency bump.

---

## 6. Acceptance targets — VALIDATED, hard checks

Build a debug/test panel that runs these and prints pass/fail. Numbers are from the reference impl at defaults — your port must reproduce them.

**Default compare** (`m_lift=2 kg, L=0.7, N=2`):
- single: `t≈0.593 s @ G≈5.23`
- CVT (η=0.85): `t≈0.479 s` (+19.2% — includes the η bump)
- CVT ratio-only (η=0.70 both): `t≈0.558 s` → **+5.90%**
- 2-speed: pair `2.0/4.6`, `t≈0.606 s` → **−2.2%, 0 shifts**

**Seven self-tests:**
1. **Stall** (`v=0,d=1`): `I=V_oc/(R_m+N·R_sys)` clamped → `I≈9.418 A`, `V_bus≈12.28 V`. Positive, finite.
2. **Free speed** (`Ke·w≈V_oc`): `I→0, F→0`. No sign blowups.
3. **Convex baseline**: time-vs-`G_fixed` single minimum (best `G≈5.23`). Jagged ⇒ bug.
4. **Energy balance**: `p_in ≈ batt_wire+copper+friction+iron+gearbox+useful` → **err ≈ 0.13%** (`<2%` pass). Drop the iron term → err jumps to **~7%** (proves `Ke≠Kt` term real). Drop gearbox term → ~11%.
5. **Ordering** (η equal): CVT time ≤ single time always (CVT is a superset of fixed-G policies).
6. **Brownout/power-limit**: raise `R_sys` 0.07→0.30 → `I_bn` 32.9→7.7 A, stall force 57→43 N (capped), no NaN. (Note: the I_bn cap *prevents* `V_bus<V_bn`, so the flag rarely trips — the symptom is current-capping, not a bus dip.)
7. **Heavy-load feasibility**: `m_lift=22 kg` → single gear **UNREACHABLE**, CVT completes `≈5.122 s`.

**Crossover sweep — vs lifted mass (`L=0.7`, ratio-only). Expected shape (the headline):**

| m_lift kg | 1 | 2 | 4 | 6 | 8 | 10 | 12 | 20 | 22 |
|---|---|---|---|---|---|---|---|---|---|
| CVT % | +7.8 | +5.9 | +2.8 | +1.4 | +0.6 | +0.2 | **0.0** | 0.0 | UNREACH |
| single G | 3.8 | 5.2 | 8.5 | 11.7 | 14.9 | 18.6 | **20.0 (pinned)** | 20.0 | — |

2-speed ≈ 0 to slightly negative across the board (0 shifts on a single lift). At 12 kg+ the single gear is **pinned at G_max** → CVT has nothing better to pick (the limit is total reduction, a *fixed-gear* fix). Raising `G_max` 20→40 lifts both together; CVT stays ~+0.5%.

**Crossover — vs stroke (`m_lift=2`, ratio-only):** L=0.2→+8.6%, 0.7→+5.9%, 1.4→+4.0%, 2.4→+2.5% (benefit falls as the move spends more time near one steady operating point).

**Realistic-η spot check** (good fixed gear 0.88 vs friction CVT 0.82): 1 kg +4.5%, 2 kg +2.2%, **4 kg −2.0%** — the modest light-lift gain is eaten/flipped by real CVT efficiency loss.

---

## 7. UI — the Lift tab

**Config (collapsible groups, number inputs unless slider):**
- *Mechanism*: `m_carriage, m_pay, N, r_spool, n_rig, F_fric, L`, `eta_drive` (slider), `eta_cvt` (slider)
- *Electrical & ratio*: `V_oc, R_sys, V_bn, I_sw, G_min, G_max, t_shift`
- *Motor*: read-only goBILDA constants

**Run controls:** `Run comparison` (all 3 at current config); `Sweep · mass`; `Sweep · stroke` (each re-optimizes the single baseline at every point, ratio-only).

**Results:**
- *Summary table*: per arch — ratio, time, Δ% vs single, shifts, flags (brownout / UNREACHABLE). Verdict line: if single UNREACHABLE → "add G_max, not variability"; if CVT Δ≈0 → "single pinned, no speed range, don't build".
- *Traces* (CVT default): `x(t)+v(t)`, `I(t)+V_bus(t)`, motor operating point on its torque-speed curve (`τ=Kt(I−sign(I)I_free)` vs `w_motor`, overlaid on the static T-S line).
- *Loss budget*: stacked bar — battery+wire / copper / iron / motor-friction / gearbox / useful, integrated.
- *Sweep chart*: Δ% vs swept var, CVT + 2-speed lines, break-even at 0.
- *Acceptance tests*: the 7 above, pass/fail.

---

## 8. Scope guardrails — do NOT build

No backend/DB/auth. No 2D animation — stroke is 1D. No brake/soft-land phase (v1). No thermal derating. No non-backdrivable holding model. No lifted-gearbox-mass (that's the hang case, separate). CSV export of sweep data is fine if cheap.

---

## 9. Build order

1. Shared helpers: confirm/extract motor+electrical+loss core from drivetrain; add `r_eff`, gravity load, no-clamp force (§2 deltas). Run tests 1–2.
2. `run_single` + stroke integration + outer optimizer (§5) + test 3, single-run `v(t)` plot.
3. Loss budget + test 4 (energy + iron/gearbox term proof).
4. `run_cvt` + `run_shifter` + tests 5,7; default-compare numbers must match §6.
5. Tab shell + Lift UI wiring + sweep harness; reproduce the §6 crossover table + realistic-η check.
6. Don't-regress pass on the drivetrain tab.

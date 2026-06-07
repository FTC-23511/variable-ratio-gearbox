# FTC Variable-Ratio Drivetrain Sim — Solve Spec

Purpose: quantify time benefit of variable ratio vs a single optimized ratio on a mecanum drivetrain, over a representative set of match paths. Brownout/power-limited DC physics. Three architectures: single fixed, multi-gear shifter, CVT.

---

## 1. State (carried across timesteps)

| sym | meaning | unit |
|---|---|---|
| `t` | time | s |
| `s` | distance into current segment | m |
| `v` | robot speed (signed, + = forward) | m/s |
| `seg` | index of current path segment | — |
| `shift_timer` | remaining clutch dead-time (shifter only) | s |
| `G` | active total reduction this step | — |

Integrate with `dt = 0.001 s`. Semi-implicit Euler.

---

## 2. Config (inputs)

### Robot
- `m_base` — robot mass without transmission (kg)
- `m_trans` — **transmission mass penalty, per architecture** (kg). Fixed gear ≈ smallest; shifter + CVT heavier. Non-negotiable for a fair fight.
- `m = m_base + m_trans`
- `N` — drive motors (mecanum = 4)
- `r_wheel` — wheel radius (m)
- `mu` — effective traction coeff (slider; modern 20–30A urethane → high, ~0.8–1.0)
- `Crr` — rolling resistance coeff (~0.01–0.03)
- `eta_drive` — gearbox+bearing mechanical efficiency (fixed/shifter)
- `eta_cvt` — **CVT efficiency slider** (the headline knob for arch 3)
- aero: omit. Negligible at <2.5 m/s.

### Motor — FIXED: goBILDA 5000-series bare core (brushed DC), do not parameterize
Datasheet: `V_nom=12`, `I_stall=9.2 A`, `tau_stall=0.1442 N·m` (20.45 oz·in / 1.47 kgf·cm), no-load speed 6000 rpm theoretical / **5800 rpm tested**, `I_free=0.25 A`.

Hard-code these derived constants:
- `R_m = V_nom / I_stall = 1.304 Ω`
- `Kt = tau_stall / (I_stall − I_free) = 0.01611 N·m/A`   (friction-corrected; pairs with the `I − I_free` term in step E)
- `Ke = (V_nom − I_free·R_m) / w_free = 0.01922 V·s/rad`  (using **tested** `w_free = 607.4 rad/s` = 5800 rpm)
- `I_free = 0.25 A`

**Kt ≠ Ke (0.0161 vs 0.0192).** This is a real-motor property, not a bug — the ~19% gap is iron/stray loss the copper+friction model doesn't capture. The solve already keeps them independent (`Ke` in step B, `Kt` in step E), so no structural change. The gap surfaces only as an extra loss term in the energy balance (see §3-J, acceptance test 4).

These are **bare-core** constants (6000 rpm). So `G` = goBILDA internal planetary reduction × external stages (total motor-shaft → wheel).

### Electrical series stack
- `V_oc` — battery open-circuit (~12.5 fresh)
- `R_sys = R_batt + R_wire` — shared resistance before current splits to motors (~0.05–0.10 Ω total). Sets the brownout ceiling.
- `V_bn` — brownout threshold (~9 V, configurable)
- `I_sw` — per-motor software current limit (A)

### Starting placeholders (replace w/ datasheet)
`V_oc=12.5, R_sys=0.07, V_bn=9.0, R_m=1.3, Kt=Ke=0.0206, I_free=0.25, r_wheel=0.048, N=4, m_base=13, mu=0.85, Crr=0.02, eta_drive=0.70, eta_cvt=0.85`

---

## 3. Per-timestep solve order

Run these in order, every step. `G` and duty `d` come from the control policy (§4) before step A.

**A. Motor speed from robot speed (no-slip assumption)**
```
w_motor = v * G / r_wheel          # rad/s
```

**B. Demanded current (coupled battery + back-EMF, averaged PWM duty d)**
Derivation: `d·V_bus = I·R_m + Ke·w_motor` and `V_bus = V_oc − N·I·R_sys`. Solve for I:
```
I = (d * V_oc - Ke * w_motor) / (R_m + d * N * R_sys)
```
(I can go negative → motor is back-driving / braking. Allowed.)

**C. Clamp current — both limits**
```
I_bn  = (V_oc - V_bn) / (N * R_sys)     # brownout-limited current ceiling
I = clamp(I, -I_sw, min(I_sw, I_bn))    # software + brownout caps
```

**D. Bus voltage (for logging / brownout flag)**
```
V_bus = V_oc - N * I * R_sys
brownout_flag = (V_bus < V_bn)
```

**E. Motor torque (subtract no-load friction)**
```
tau_motor = Kt * (I - sign(I)*I_free)   # friction always opposes
```

**F. Tractive force at the floor**
```
eta = eta_cvt if architecture==CVT else eta_drive
F_drive = N * tau_motor * G * eta / r_wheel
```

**G. Traction clamp (slip ceiling)**
```
F_tract_max = mu * m * g                 # g = 9.81
F_drive = clamp(F_drive, -F_tract_max, F_tract_max)
# if |unclamped| > F_tract_max -> set slip_flag (expect rare w/ grippy rollers)
```

**H. Net force + resistance**
```
F_roll = Crr * m * g * sign(v)           # opposes motion
F_net  = F_drive - F_roll
```

**I. Integrate**
```
a = F_net / m
v = v + a*dt
s = s + v*dt                              # semi-implicit (use updated v)
t = t + dt
```

**J. Heat-loss log (optional but valuable — proves where watts die)**
```
P_batt_wire = (N*I)^2 * R_sys             # shared stack
P_copper    = N * I^2 * R_m               # winding heat (the big one)
P_friction  = N * Kt * I_free * w_motor   # windage/brush friction
P_iron      = N * (Ke - Kt) * I * w_motor # iron/stray (the Ke!=Kt gap)
P_mech_out  = F_drive * v                 # useful
P_in        = V_oc * (N*I)
```
Integrate each over the run for the loss budget chart. With the friction + iron terms included, `P_in` closes against the sum (see acceptance test 4).

---

## 4. Control policy — sets `G` and `d` each step

Three modes per segment: **ACCEL → CRUISE → BRAKE**. Brake trigger (all architectures), traction-limited stop:
```
a_brake = mu * g
d_stop  = v^2 / (2 * a_brake)              # distance needed to stop
if (L_seg - s) <= d_stop  -> BRAKE mode
```
BRAKE: command reverse (`d` reversed / negative target) capped at traction; or model as `a = -a_brake` directly for v1.

### Arch 1 — single fixed ratio
- `G = G_fixed` (constant; chosen by outer optimizer §5).
- ACCEL: `d = 1`.
- CRUISE (if segment long enough to near top speed): trim `d` to hold `v` against `F_roll`.

### Arch 2 — multi-gear shifter
- Discrete ratios `{G_1 > G_2 > ... > G_k}`, **k configurable**.
- Selection each step = the gear giving most wheel force *right now*:
```
for each gear i:
    w_i  = v * G_i / r_wheel
    I_i  = clamp((V_oc - Ke*w_i)/(R_m + N*R_sys), -I_sw, min(I_sw,I_bn))   # d=1
    F_i  = N * Kt*(I_i - sign(I_i)*I_free) * G_i * eta_drive / r_wheel
G_target = argmax_i F_i
```
- On change of `G_target`: start `shift_timer = t_shift` (config, ~0.05–0.20 s). While `shift_timer > 0`: **F_drive = 0** (clutch cut), decrement timer, then commit new `G`.
- `d = 1` in ACCEL.

### Arch 3 — CVT (theoretical ceiling)
- `G` continuous in `[G_min, G_max]`.
- ACCEL (objective = max force): grid-scan `G` over its range, run steps A–G for each, pick `G` that maximizes clamped `F_drive`. The optimizer naturally parks the motor against whichever limit binds (traction or current/brownout).
- CRUISE (objective = max efficiency): pick `G` placing `w_motor` near peak-efficiency speed while producing only the force needed to hold `v`. Motor efficiency `= Ke*w_motor / V_applied`.
- `eta_cvt` slider scales all CVT force — lower it to find the break-even efficiency where CVT stops beating the fixed gear.

---

## 5. Outer loop — fairness (matches a real season)

1. Build **representative path set** (§6): a handful of segment sequences mimicking match traversals.
2. **Single fixed ratio**: sweep `G_fixed` across a range, run the *entire path set* for each, pick the `G_fixed` minimizing **total time across all paths**. That is the season's "one best gear" — the honest baseline.
3. Shifter + CVT: run each with its optimal policy on the same path set.
4. Compare total times. Benefit = baseline − architecture.

---

## 6. Path model

- Path = ordered segments. Each: `L_seg` (m), `end = stop | through`.
- v1 = 1D straight segments with stops. Captures all drivetrain dynamics.
- Turns: add fixed time penalty per turn for v1; full rotational sim later. Strafing only shifts effective `mu`/`eta`, not the conclusion — defer.
- FTC field ≈ 3.66 m → most segments short → accel-dominated. Keep some long segments in the set to expose any top-speed benefit.

---

## 7. Outputs

**Primary — the decision tool (not one number):** benefit (Δtime, %) swept against each of:
`segment length, mu, m (incl. m_trans), k (gear count), eta_cvt, R_sys, I_sw`.

**Per-run traces:** `v(t)`, `I(t)`, `V_bus(t)`, motor operating point on its torque-speed curve, time spent in each speed regime, brownout/slip flags.

**Loss budget (from §3-J):** stacked watts → battery+wire / copper / mechanical / useful, integrated over the run.

**Expected shape (sanity check):** small benefit on short segments (accel-dominated, grippy → near traction or current limited either way); benefit grows with segment length and with `R_sys` (tighter power budget rewards holding the motor on its best point); benefit shrinks fast as `m_trans` and `(1−eta_cvt)` rise. If CVT shows huge gains on short fields, suspect a missing clamp.

---

## 8. Build order in Claude Code

1. Motor + electrical core (§3 A–F), unit-test against datasheet stall/free points.
2. Add clamps + integration (§3 G–I), single-segment sprint, plot `v(t)`.
3. Arch 1 + brake/cruise + outer ratio optimizer (§5.2).
4. Path set + total-time metric (§6).
5. Arch 2 shifter, then Arch 3 CVT.
6. Sweep harness + loss-budget logging (§7).

# DroneGuard — project memory

Student project (Perinba Athiban.S, R. Rithvik Nimal, S Sarvesh). A drone health
monitor and mission optimiser: score a drone's health from telemetry and decide
when it must turn for home. Built first as a MATLAB/Simulink simulation; next
stages turn it into a planning website and a live monitor.

- Original deck: `docs/DroneGuard.pdf`
- Component-selection paper the next stage builds on: `7.Component selection.pdf` (repo root)
- Full roadmap page: https://claude.ai/artifact/VxRS7D3hXPE6XbEYTZL5qx

User-facing docs: `README.md` (project case and results) and `matlab/README.md`
(how to run, what each plot means, what to change). Keep them in sync with code.

---

## Implementation progress

_Last updated 2026-10-22._

| Stage | Status |
|---|---|
| Simulation (MATLAB/Simulink) | **Done** |
| P0 Portable core (Python port) | Not started |
| P1 Sizing engine | Not started |
| P2 Design check in simulation | Not started |
| P3 Planning website | Not started |
| P4 Virtual monitoring dashboard | Not started |
| P5 Real hardware | Not started |

### Done — simulation
- [x] Closed-loop multirotor: altitude + speed PID, 2-DOF point-mass dynamics, drag
- [x] Powertrain: propulsion (thrust → current → heat), battery (SOC, voltage sag)
- [x] Two ramped faults: motor efficiency loss, battery ageing (clamped 0..1 by Saturation blocks)
- [x] Health score: linear 0–100 per indicator, fused with `min()`, alert 0/1/2
- [x] Mission manager merged into the DroneGuard module: out-and-back route, dynamic RTH
      threshold with current-trend projection, health ALARM forces abort
- [x] Plots: `droneguard_run`, `droneguard_compare` (dynamic vs fixed 25% rule)
- [x] Animated dashboard with FPV city camera, wall-clock pacing, MP4 export
- [x] Repo layout, READMEs, `.gitignore`
- [x] Turn-back summary reports the true reason (MISSION COMPLETE vs the two ABORT cases)

### Next — roadmap phases

**P0 Portable core** — port sizing maths, the 10-state flight model and the
DroneGuard module to Python. Simulink stays the reference.
Done when the port reproduces the reference results below.

**P1 Sizing engine** — paper's method, corrected:
1. AUW₀ = payload + frame(n)
2. T_motor = 2 × AUW × g ÷ n (2:1 thrust)
3. Pick motor + propeller from catalogue (enough thrust, lowest current)
4. ESC ≥ I_motor,max ÷ 0.8
5. Battery: cells from motor voltage; capacity from hover current × endurance ÷ 0.8;
   check C × Ah ≥ n × I_motor,max
6. Recompute AUW with real part masses; iterate until it settles

Done when it sizes the paper's seven payload cases (2–30 kg) and flags the
firefighting battery.

**P2 Design check** — map spec → sim params (`m_drone`, `k_thrust`, `Cap_Ah`,
`V_full/V_empty`, `R_int`, `Cd_forward`, `Cruise`, `Range`, `Alt_cmd`, health
limits). Run a healthy and a stressed mission; return go/no-go with reserve.

**P3 Planning website** — application presets, tunable mission inputs, spec card
with cost, simulation plots. Start with the **delivery** application.

**P4 Virtual monitoring** — live dashboard over WebSockets, fed first by the
Simulink model sending MAVLink (UAV Toolbox `mavlinkio`), then PX4/ArduPilot SITL.

**P5 Hardware** — Pixhawk + Raspberry Pi companion + telemetry ESCs
(BLHeli_32/AM32). Bench → tethered hover → short flight. Check India Drone Rules
2021 / Digital Sky before outdoor flights.

### Decisions taken
- Port the web-facing maths to Python; Simulink remains the reference to test against.
- Web stack: FastAPI + WebSockets with a light frontend (Streamlit only for a throwaway prototype).
- First application: delivery. Healthcare VTOL (fixed-wing) is out of scope.
- One spec file generates the thresholds used by both simulation and the Pi.

### Open questions (need team answers before P0)
- Is multirotor-only acceptable?
- Hardware budget for P5?
- Who owns which track (sizing+sim / website / telemetry+hardware)?
- Accounts and saved designs, or a stateless planner?

---

## Repo layout

```
startup.m                     run first: adds matlab/{src,sim,model} to the path
matlab/model/DroneGuard_Sim.slx
matlab/src/                   the algorithm - plain MATLAB, no toolboxes, codegen-safe
  droneguard_module.m           whole module: health -> mission (passes alert through)
  droneguard_health.m           scores; battery health = voltage SAG, not charge
  droneguard_mission.m          RTH decision; persistent state, I_prev, dI_smooth
  droneguard_thresholds.m       single source of truth: limits + mission rules (literals)
matlab/sim/                   simulation only, never deployed
  droneguard_params.m           script: aircraft numbers into the calling workspace
  droneguard_run.m              load params -> sim -> plot
  droneguard_compare.m          dynamic vs fixed rule
  droneguard_plot.m             4-panel figure + summary
  droneguard_dashboard.m        animated playback, FPV camera
  droneguard_missionend.m       index where the mission really ended
docs/                         deck + README images
```

## Running

```matlab
cd E:\drones_project
startup
droneguard_run                 % or droneguard_run('fixed')
droneguard_compare
droneguard_dashboard           % options: Speed=5, Until=335, Pace=false, Save="run.mp4"
```

Requires MATLAB + Simulink only (verified with `dependencies.toolboxDependencyAnalysis`).
Developed on R2026a Update 5. Solver: fixed-step ode3 at `Ts = 0.1` s, `T_end = 800`.

## Reference results — must not drift

Any change to the model or a port must reproduce these:

| Case | Result |
|---|---|
| Default (dynamic RTH) | turn at t=321 s, 3817 m (energy abort); home at 649 s with 10% charge; WARN 529 s, ALARM 613 s |
| `droneguard_run('fixed')` | reaches waypoint 6001 m at 503 s; battery dead 3913 m from home at 687 s |
| Severe motor fault (`MotorFault_Start=30; MotorFault_Slope=1/200; k_eff_fault=0.80`) | health ALARM abort at 145 s, 1710 m, with 81% charge; home at 298 s with 19% |
| Battery fault off | overall health ends 56 (not 25); ALARM never fires |

`simData` columns: 1 alt, 2 speed, 3 dist, 4 T_motor, 5 I_motor, 6 V_batt, 7 SOC,
8 motor_h, 9 batt_h, 10 overall_h, 11 alert, 12 rth_threshold, 13 soc_needed, 14 mission_state.

## Model facts worth knowing
- Three sample-time tasks: continuous plant, discrete 0.1 s module, constants.
- 10 continuous/discrete states (dynamics ×4, thermal lag, SOC, PID integrators/filter, Command_Delay).
- `Command_Delay` (Unit Delay on the speed command) breaks the algebraic loop
  module → FC → propulsion → current → module. Do not remove it.
- DroneGuard MATLAB Function block must stay `ChartUpdate='DISCRETE'`, `SampleTime='Ts'`
  because of the persistent variables in `droneguard_mission.m`.
- The motor fault reduces thrust-per-amp; current is never faulted directly.
- Blocks hold variable NAMES resolved from the base workspace. The green Run
  button fails unless `droneguard_params` ran first.

## Paper findings (don't inherit these numbers)
- Table 3 "Power Required" is ~50× too high (46,517 W for a 2 kg quad; its parts max out at 4×15 V×16 A = 960 W).
- Firefighting battery 1200 mAh 15C delivers 18 A max — cannot feed eight motors.
- AUW is payload + a fixed guess; must iterate instead.
- Keep: the method, the 2:1 thrust rule (matches its Table 3), the 80% rules, Table 2 as a seed catalogue, Table 4 costs labelled as 2022 INR basic parts.

## Working conventions and gotchas
- **User preferences:** keep things simple and explained in plain words (the user is
  new to Simulink); don't run git commands unless asked — the user commits.
- **MATLAB MCP:** if tools fail to attach, ask the user to run `shareMATLABSession` in
  MATLAB. NEVER call `restoredefaultpath` (it strips the MCP functions). If `model_edit`
  is missing: `addpath('C:\Users\admin\.matlab\agentic-toolkits\simulink'); satk_initialize`.
- Edit the model with the `model_edit` tool, not `set_param`/`add_block`. MATLAB Function
  block code is set via `r = sfroot; ch = r.find('-isa','Stateflow.EMChart','Path',p); ch.Script = ...`.
- The dashboard figure contains a uipanel: export it with `exportapp`, not `exportgraphics`.
- After simulating, delete the regenerated `slprj/` and `DroneGuard_Sim.slxc` (also gitignored).
- Some files (e.g. `startup.m`) use CRLF. Prefer the Edit tool over shell heredocs with
  `\n` inside — heredoc escaping has corrupted MATLAB strings before.
- Python with `pypdf`: `C:/Users/admin/AppData/Local/Programs/Python/Python312/python.exe`
  (the bare `python` on the bash PATH may differ).
- When changing behaviour, re-run all three entry points and compare to the reference results.

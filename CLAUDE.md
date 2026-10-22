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
| P0 Portable core (Python port) | **Done** (sample-by-sample Simulink trace check still to run) |
| P1 Sizing engine | **Done** (catalogue masses are estimates) |
| P2 Design check in simulation | **Done** |
| P3 Planning website | **Done** (delivery only; stateless) |
| P4 Virtual monitoring dashboard | **Done** (fed by the Python virtual drone; SITL supported, not run) |
| P5 Real hardware | **Software ready**, guide written; hardware steps need parts + the team |

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

### Done — P0 Python port (`python/`)
- [x] `thresholds`, `health`, `mission` (state on a `MissionManager` object, not globals), `module`
- [x] `params` (same names as MATLAB; slopes derived unless overridden) and `sim`: the
      same 9 continuous states + Command_Delay, Simulink's ode3 (Bogacki–Shampine) at Ts,
      same per-step order (delay → slew limit → read plant → module → log → integrate)
- [x] `summary` = `droneguard_missionend` + the console summary of `droneguard_plot`
- [x] `python -m droneguard [dynamic|fixed|compare] [--plot]`; under 0.5 s per mission
- [x] `tests/test_reference.py`: all 4 reference cases match exactly, plus 2 module tests
- [x] `matlab/sim/droneguard_export.m` + `python/tools/compare_with_matlab.py` for a
      sample-by-sample comparison
- [ ] Run that trace comparison (MATLAB was not attached when P0 was built)
- Sizing maths did not exist in MATLAB to port; it is written fresh in Python in P1.

### Done — P1 sizing engine (`python/droneguard/sizing.py`, `catalogue.py`)
- [x] Catalogue = paper Table 2 (16 motor+prop rows). Masses ESTIMATED: motor 3.2 g/cm³ of
      can volume (larger dim = diameter), prop 0.15 g/in², ESC 5 + 0.9 g/A, frame
      100 + 8 g per arm per prop-inch, LiPo 150 Wh/kg, avionics 300 g
- [x] Hover current = I_max × (T/T_max)^1.5 (momentum theory, anchored at rig point)
- [x] Six steps iterated to a settled AUW; `selection="lightest"` (default) runs the loop
      for every motor × {4,6,8} and keeps the lightest feasible design; `"paper"` re-picks
      the smallest strong-enough motor each pass
- [x] Checks: 2:1 thrust, ESC ≥ I_max/0.8, C×Ah ≥ n×I_max, endurance on 80% of pack;
      impossible requests return the closest miss with % thrust shortfall and a fix
- [x] Cost = paper Table 4 interpolated by payload (2022 INR, basic parts, not our parts)
- [x] `python -m droneguard.sizing [--payload-kg --endurance-min --motors --paper-cases]`
- [x] `tests/test_sizing.py` (8 tests): Table 3 motor picks reproduced, power error,
      firefighting pack flagged, rules obeyed, impossible request explained
- Findings at 15 min hover: 2/5/10 kg OK (AUW 6.4/12.7/19.6 kg); 15–30 kg NOT POSSIBLE with
  this catalogue (3–7% short on thrust); paper's 6 kg / 45 min delivery claim NOT POSSIBLE

### Done — P2 design check (`python/droneguard/check.py`)
- [x] `to_sim(design, Mission)` → (SimParams, Thresholds). From the design: m_drone, cells
      (V_full 4.2/cell, V_empty 3.5/cell), Cap_Ah, k_thrust = m·g / I_hover_total.
      Scaled from the reference drone: R_int ∝ cells/Ah, Cd ∝ mass^(2/3), PID gains ∝ mass,
      K_therm ∝ 1/I_hover² (same hover temp rise), I_good/I_bad ∝ I_hover, Sag ∝ cells
- [x] Anchor test: a design equal to the reference drone maps to exactly the reference
      params and reproduces 321 s / 3817 m / 649 s
- [x] Three flights: healthy, stressed (both faults full at planned home time t_plan =
      2·d/v, starting at the reference fractions 150/800 and 400/800), range (waypoint
      out of reach → max one-way distance)
- [x] Verdict GO / GO WITH CAUTION (stressed lands below reserve) / NO-GO (healthy
      can't complete, or stressed doesn't get home), with reasons
- [x] `python -m droneguard.check --payload-kg --endurance-min --distance-km --cruise --altitude`
- [x] `tests/test_check.py` (7 tests); whole suite 21 tests, ~4 s
- Known limitation: drag and current scale alike, so range ≈ f(hover endurance) only
  (15 min → ~5.3 km out and back at 12 m/s for every design)

### Done — P3 planning website (`python/web/`)
- [x] `droneguard/applications.py`: presets with slider ranges; delivery available,
      firefighting / agriculture / healthcare defined with paper notes but `available=False`,
      so the planner hides their tabs (set `available=True` to show one)
- [x] FastAPI `web/app.py`: `GET /api/applications`, `POST /api/plan` (pydantic-validated;
      sizing + check + cost + series thinned to ≤400 points, trimmed at mission end;
      `lru_cache` on inputs), `GET /` serves `web/static/index.html`
- [x] One-page frontend, no build step, no JS libraries: sliders (debounced re-plan),
      verdict banner, parts table with Table 4 prices, key numbers, mass bar, three canvas
      charts (distance; charge vs needed-to-get-home; worn-out health with WARN/ALARM).
      Light/dark tokens; checked at 1300 px and a true 390 px phone width
- [x] Run: `cd python; .venv\Scripts\python -m web` → http://127.0.0.1:8000
- [x] `python/.venv` (gitignored) holds fastapi/uvicorn/httpx; `tests/test_web.py` (5 tests)
      skips without FastAPI. Suite: 26 tests in the venv, 21 + 5 skipped in plain Python
- Decision (user answered "yes" to "go ahead"; accounts question left open): stateless,
  nothing stored. Screenshot: `docs/images/planner.png`
- Headless Chrome cannot go below ~500 px window width: test phone layout in a 390 px iframe

### Done — P4 live monitoring (`python/live/`, `web/live.py`, `web/static/live.html`)
- [x] Feed = Python virtual drone, NOT Simulink: UAV Toolbox (`mavlinkio`) is not installed
- [x] MAVLink 2, ArduPilot dialect. Drone → `ESC_TELEMETRY_1_TO_4` (temp degC uint8, current
      cA), `SYS_STATUS` (mV, cA), `BATTERY_STATUS` (mAh used → SOC), `LOCAL_POSITION_NED`
      (sent last = frame complete; monitor ticks when ≥0.095 s since last). DroneGuard →
      `COMMAND_LONG NAV_RETURN_TO_LAUNCH`, once, only for aborts (waypoint = autopilot's job);
      logs the autopilot's `COMMAND_ACK` accepted/REFUSED
- [x] `VirtualDrone`: plant + autopilot mission (out, back, land), same tick order as sim.py so
      lockstep reproduces the reference within field rounding (3816 m, ALARM 612 s); `inject()`
- [x] `Monitor`: same `DroneGuardModule`; no-ESC-temperature fallback (SITL); event log with
      5 s clear-hysteresis (module output itself unfiltered)
- [x] `run_lockstep` (bytes in memory, deterministic) and `LiveSession` (two threads, UDP,
      wall-clock paced, `monitor_port=` for drone-only); CLIs `python -m live.monitor`,
      `python -m live.drone`
- [x] Dashboard: start/stop, scenario, rule, speed 1–50×, inject motor/battery faults, alert
      badge, meters, telemetry, ground-truth faults, event log, 5 live charts with RTL mark;
      planner "Fly it live" + "Spec file for the Pi" (`GET /api/spec`, `live/spec.py`)
- [x] `tests/test_live.py` (9 tests). Suite: 35 in the venv; 21 + 14 skipped in plain Python
- Screenshot `docs/images/live.png`

### Done — Raspberry Pi plays the drone (`docs/pi_demo.md`)
- [x] User chose: **Pi = virtual drone**, laptop = DroneGuard + dashboard (not Pi = DroneGuard)
- [x] `live/autopilot.py` `fly()` shared by laptop mode and Pi: HEARTBEAT with ArduCopter modes
      (STABILIZE idle / AUTO / RTL / LAND), NAMED_VALUE_FLOAT f_motor/f_batt truth,
      STATUSTEXT, obeys RTL; sim-only COMMAND_LONG USER_1 fault / USER_2 start (plan packed in
      7 params: speed, faults+10*motors, payload, endurance, distance, cruise, altitude) / USER_3 stop
- [x] `python -m live.drone --serve --host <laptop IP>` on the Pi: waits, flies, waits again
- [x] `live/remote.py` `PiLink` on the laptop (udpin 0.0.0.0:14550): detects the Pi by heartbeat,
      feeds telemetry to DroneGuard only after the Pi ACKs the start
- [x] Dashboard: "Drone runs on" laptop / Raspberry Pi, Pi status dot + exact command,
      `/live?source=pi`; `/api/live/pi/listen`
- [x] Verified over real Wi-Fi: DroneGuard sends RTL at 320.7 s / 3816 m as in sim. Pi receives
      it 1.5 s (flight time) later at 20× → home 652 s 9%; 15 s later at 50× → 2%. So Pi
      flights are capped at 20× (`PI_MAX_SPEED`, 50× button disabled)
- [x] Launch scripts: `scripts\demo.bat` → `demo.ps1` (website in own window, reuses/asks to
      restart an old one; `ssh pi true`, else runs user's `~/.ssh/findpi.ps1`; opens
      `ssh -t pi ~/droneguard/pi/start_drone.sh`; browser `/live?source=pi`; `-NoPi`).
      `scripts\deploy_pi.ps1` copies droneguard/live/tests/pi, fixes CRLF, runs Pi tests.
      `python/pi/start_drone.sh` gets the laptop IP from `$SSH_CLIENT` → survives IP changes.
      `.gitattributes` keeps *.sh / *.service LF. User chose ssh + script over broadcast discovery
- [x] Python 3.8 compatible (`from __future__ import annotations` in droneguard/ and live/)
- [x] Tests: 37 on the laptop venv; 37 (7 skipped) on the Pi
- Pi facts: Raspberry Pi 4, Ubuntu 20.04 aarch64, Python 3.8.10, user `plufle`, code in
  `~/droneguard`, venv `.venv` (made `--without-pip` + get-pip 3.8; no sudo; needs
  `lxml<5.4` prebuilt). SSH alias `pi`; IP changes - user's `findpi` updates ~/.ssh/config.
  Laptop Wi-Fi 192.168.1.5 (Private profile; python.exe allowed inbound on Private)

### P5 — software ready, hardware pending (`docs/hardware.md`, `python/deploy/`)
- [x] Guide: ArduPilot (not PX4 - its ESC_STATUS/ESC_INFO is not decoded), TELEM2 wiring,
      SERIAL2/SR2 params, Pi UART setup, venv, systemd unit, 5-step test ladder, Drone Rules note
- [ ] Buy parts (planner + datasheet masses), bench tests, tethered hover, first flight - team
- [ ] Optional: decode PX4 `ESC_STATUS`/`ESC_INFO`; run against ArduPilot SITL in WSL

### Next — roadmap phases

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
- Accounts and saved designs, or a stateless planner? (built stateless for now)

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
  droneguard_export.m           one mission -> CSV (t + 14 simData columns)
python/                       P0 port; Simulink is the reference it is tested against
  droneguard/                   thresholds, health, mission, module, params, sim, summary,
                                catalogue + sizing (P1), check (P2), applications (P3)
  web/                          FastAPI: planner app.py + live.py; static/{index,live}.html,
                                style.css, charts.js (shared)
  live/                         P4: mavlink_io, virtual_drone, monitor, session, drone, spec
  deploy/droneguard.service     P5: systemd unit for the Pi
docs/hardware.md              P5 bring-up guide
  tests/test_reference.py       the reference table below, as unit tests
  tools/compare_with_matlab.py  max |diff| per signal vs an exported Simulink run
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

```bash
cd python
python -m droneguard compare --plot
python -m unittest -v              # must stay green
```
Use `C:/Users/admin/AppData/Local/Programs/Python/Python312/python.exe` (has numpy,
matplotlib; no pytest - tests use stdlib unittest).

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
- Table 3 "Power Required" is 13–54× too high (46,517 W for a 2 kg quad; its parts max out at 4×15 V×16 A = 960 W).
- Table 3 "Total require thrust" is really the AUW guess; the garbled formula is
  single motor thrust = (AUW × 2) / motors.
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
- Any change to the MATLAB algorithm or model must be mirrored in `python/` (same
  names) and `python -m unittest` re-run; update the reference table if results move.
- pymavlink: `import live` first - it sets `MAVLINK20=1` (default MAVLink 1 silently drops
  ESC_TELEMETRY, id 11030). On Windows `udpout` binds the destination port, so replies loop
  back to the sender: the drone side uses `udpin` on its own port with the ground address
  pre-seeded in `clients`. `SIO_UDP_CONNRESET` is switched off to avoid WinError 10054.
- Background servers/processes: stop them by task ID or exact PID, never a blanket
  `taskkill /IM python.exe`. The user may run their own `python -m web` on :8000 - check
  the owner (Get-NetTCPConnection) before assuming a server is ours; never kill theirs.
- SSH to the Pi from PowerShell (Windows OpenSSH knows its key; Git Bash's ssh does not).
  Send multi-line scripts with `Get-Content ... | ssh pi bash -s` (strip CR); inline quotes
  get mangled by PowerShell.

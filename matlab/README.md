# DroneGuard — MATLAB guide

Practical notes for running and changing the simulation.
For the project write-up and results, see the [main README](../README.md).

---

## Running it

`startup.m` lives in the **parent** folder, not in here.

```matlab
cd E:\drones_project     % the parent folder, not matlab/
startup
```

Then:

| Command | What it does |
|---|---|
| `droneguard_run` | Fly one mission, draw 4 plots |
| `droneguard_run('fixed')` | Same flight with the dumb 25% turn-back rule |
| `droneguard_compare` | Fly both, draw 5 plots comparing them |
| `open_system('DroneGuard_Sim')` | Open the block diagram |

---

## The files

### `src/` — the algorithm

This is the part that would be copied onto a Raspberry Pi. Plain MATLAB, no
toolboxes, no Simulink.

| File | What it does |
|---|---|
| `droneguard_module.m` | The whole module. Calls the two below, in order. |
| `droneguard_health.m` | Telemetry in, health scores out. |
| `droneguard_mission.m` | Decides when to turn back. |
| `droneguard_thresholds.m` | All the limits and mission rules, in one place. |

### `sim/` — simulation only

Never deployed. Exists to fly the virtual drone and draw pictures.

| File | What it does |
|---|---|
| `droneguard_params.m` | All the numbers describing the drone itself. |
| `droneguard_run.m` | Load numbers → simulate → plot. |
| `droneguard_compare.m` | Runs it twice and compares. |
| `droneguard_plot.m` | Draws the 4-panel mission figure. |
| `droneguard_missionend.m` | Works out when the mission really ended. |

### `model/`

`DroneGuard_Sim.slx` — the Simulink model. 17 blocks on the top level.

---

## Reading the plots

### `droneguard_run` — 4 panels

| Panel | Shows | What to look for |
|---|---|---|
| 1. Mission | Distance from home, speed | The triangle: out, turn, back |
| 2. The decision | Charge left vs charge needed to get home | **Where the two lines meet is the turn-back moment** |
| 3. Motor | Temperature and current | Both climb as the motor wears out |
| 4. Health | Motor, battery, overall | The grey band (overall) rides on whichever is worse |

### `droneguard_compare` — 5 panels

Blue thick solid = DroneGuard. Orange thin dashed = the fixed 25% rule.
A circle marks where each mission ended.

| Panel | Shows |
|---|---|
| 1. Where it went | Blue's triangle closes (home). Orange's doesn't — its circle is 3.9 km out. |
| 2. Charge | Solid = charge left. Dotted = each rule's turn-back threshold. Blue's dotted climbs; orange's is flat at 25%. |
| 3. Motor current | ~19 A healthy, ~31 A degraded. The two runs overlap. |
| 4. Motor temperature | Warms to ~50 °C normally, then climbs to ~80 °C. Never hits the 100 °C limit. |
| 5. Overall health | Almost identical in both runs. |

Panel 5 shows **only the combined score**, because six lines would be
unreadable. For the motor/battery breakdown use `droneguard_run`.

---

## How the health score works

Three measurements, each mapped onto 0–100:

| Measurement | 100 (fine) | 0 (critical) |
|---|---|---|
| Motor temperature | 60 °C | 100 °C |
| Motor current | 28 A | 40 A |
| Battery voltage sag | 0.10 V | 1.50 V |

Then:

```
motor health   = min(temperature score, current score)
battery health = sag score
overall        = min(motor health, battery health)
```

**`min()`, not average.** Averaging a healthy battery at 100 with a dying motor
at 20 gives 60, which looks survivable. It isn't. The worst component wins.

Overall health becomes an alert: **0 = OK**, **1 = WARN** (below 70),
**2 = ALARM** (below 40).

### Battery health is not battery charge

This trips people up.

Going from 100% to 10% charge is **not** unhealthy — that is just using the
battery. Health measures **sag**: whether the pack delivers the voltage its
charge level says it should.

```
expected voltage = V_empty + (V_full − V_empty)·SOC − current·resistance
sag              = expected − measured
```

A pack at 5% charge with good internal resistance scores **100**.
A pack at 80% charge that sags a volt under load scores badly — it still has
charge, it just cannot deliver it. That is what ageing does.

The two feed different places:

- **charge** → the mission decision (*can I get home?*)
- **sag** → the health score (*is this pack still sound?*)

In the default run the **battery is what drives the alarm**. Turn its fault off
and health ends at 56 instead of 25, and ALARM never fires at all.

---

## How the mission decision works

Every cycle the module asks: *if I turn around now, can I get home?*

```
time home   = distance / cruise speed
charge home = expected current × time home
threshold   = charge home / capacity × safety margin + reserve

turn back when  charge left ≤ threshold
```

The threshold is not a fixed number. It rises as the drone flies further out,
and rises again as the motor degrades and pulls more current. In the default run
it climbs from 10% at takeoff to about 60% by the time the drone turns.

There are **three** reasons to stop heading away from home:

1. it reached the waypoint,
2. it no longer has the charge to get back,
3. health says ALARM — the aircraft is not fit to keep going.

---

## Changing things

Edit the file, save, re-run. Never touch the blocks — they only hold *names*,
and the numbers behind those names live in these two files.

### `sim/droneguard_params.m` — the drone

```matlab
T_end            = 800;   % how long the run lasts [s]
Alt_cmd          = 50;    % cruise altitude [m]
m_drone          = 2.0;   % mass [kg]
MotorFault_Start = 150;   % when the motor starts wearing out [s]
k_eff_fault      = 0.50;  % how bad it gets (0 = none, 1 = total)
BattFault_Start  = 400;   % when the battery starts ageing [s]
```

### `src/droneguard_thresholds.m` — the module

```matlab
'Range',     6000,   % how far out the waypoint is [m]
'Cruise',    12,     % cruise speed [m/s]
'Reserve',   0.10,   % charge it refuses to land below
'RTH_fixed', 0.25,   % the naive rule, for comparison
'Warn',      70,     % health below this -> caution
'Alarm',     40,     % health below this -> abort
```

### Temporary changes (for one run only)

Set them **after** `droneguard_params`, or they get overwritten:

```matlab
droneguard_params
MotorFault_Start = 30;      % break the motor early
MotorFault_Slope = 1/200;   % and fast
k_eff_fault      = 0.80;    % and badly
out = sim('DroneGuard_Sim');
droneguard_plot(out)
```

That one turns back at 145 s with 81% charge still in the pack — because health
hit ALARM. The energy maths alone would have kept flying.

---

## Common mistakes

**`Unrecognized function or variable 'droneguard_run'`**
You skipped `startup`, or you are in the wrong folder. Run `pwd` to check —
you want the parent folder, not `matlab/`.

**Errors about undefined `k_thrust`, `Alt_cmd`, etc. when simulating**
You pressed the green Run button in Simulink. The blocks hold *names*, not
numbers, and nothing loaded the numbers first. Run `droneguard_params`, then the
button works. Or just use `droneguard_run`, which does it for you.

**`slprj` folders and `.slxc` files keep appearing**
Normal. Simulink's build cache. They are in `.gitignore` and will not reach
GitHub.

**A plot looks like it only has one line**
Two traces are probably sitting exactly on top of each other. That is usually
real — for example the two runs in `droneguard_compare` degrade identically, and
overall health sits exactly on whichever component is worse.

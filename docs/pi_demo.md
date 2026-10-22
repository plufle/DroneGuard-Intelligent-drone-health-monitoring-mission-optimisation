# Demo: the Raspberry Pi plays the drone

The Raspberry Pi runs the virtual drone and sends real MAVLink telemetry over
Wi-Fi, the way a flight controller would. The laptop runs DroneGuard and the
live dashboard. You control everything from the dashboard: take off, inject
faults, stop. DroneGuard's `RETURN_TO_LAUNCH` goes back to the Pi over the air.

```
RASPBERRY PI  (the drone)                    LAPTOP  (DroneGuard + dashboard)
python -m live.drone --serve ── telemetry ──▶ python -m web  →  /live
     AUTO → RTL → LAND        ◀── RTL, take off, faults, stop ──
```

Tested on a Raspberry Pi 4 (Ubuntu 20.04, Python 3.8) and a Windows laptop on
the same Wi-Fi. DroneGuard decides exactly as in the simulation (RTL at 321 s,
3816 m). The RTL takes a moment to cross the Wi-Fi, so at 20× the drone lands at
652 s with 9%, against 649 s with 10% when everything runs on one machine.

## Once: install on the Pi

Already done on the Pi (user `plufle`), in `~/droneguard`. To redo it, first copy
the code with `powershell -File scripts\deploy_pi.ps1 -NoTest`, then on the Pi:

```bash
ssh pi
cd ~/droneguard
# Ubuntu 20.04 has no python3-venv pip; this needs no sudo:
python3 -m venv --without-pip .venv
curl -sSf https://bootstrap.pypa.io/pip/3.8/get-pip.py | .venv/bin/python
.venv/bin/pip install --only-binary=lxml "lxml<5.4" numpy pymavlink   # lxml 6 won't build on 3.8
.venv/bin/python -m unittest            # 37 tests, 7 skipped (website ones)
```

## Every demo: one double-click

Laptop and Pi on the same Wi-Fi, then double-click **`scripts\demo.bat`**. It:

1. starts the DroneGuard website in its own window (or reuses one already running;
   if it finds an older version running, it asks before restarting it),
2. checks it can reach the Pi, and runs your `findpi` if the Pi's address changed,
3. opens a window that runs `ssh -t pi ~/droneguard/pi/start_drone.sh`,
4. opens http://127.0.0.1:8000/live?source=pi in the browser.

When the dot turns green (**Raspberry Pi connected**), press **Take off**.
**Wear the motor** / **Age the battery** send the fault to the Pi.
Close the Pi window (or press Ctrl+C in it) to stop the drone.
`scripts\demo.bat -NoPi` runs the laptop-only demo instead.

**By hand**, the same thing:
```bash
# laptop
cd E:\drones_project\python
.venv\Scripts\python -m web           # then open http://127.0.0.1:8000/live?source=pi
# Pi
ssh pi
~/droneguard/pi/start_drone.sh        # works out the laptop's IP from the ssh session
```

**Speed: 20× at most with the Pi.** At 50× the Pi sends about 2,000 messages a
second, the laptop falls slightly behind, and the RTL arrived 15 s of flight late
(home with 2% instead of 10%). At 20× it arrived 1.5 s late (home with 9%). The
page disables 50× when the Pi is selected. A real drone flies at 1×, where this
is a fraction of a second.

**After changing the code**, copy it to the Pi: `powershell -File scripts\deploy_pi.ps1`
(it keeps the Pi's Python environment and runs the tests there).

## When the network changes

| What changes | Does it still work? |
|---|---|
| The laptop gets a new IP | **Yes.** `start_drone.sh` reads the laptop's address from the ssh session each time. |
| The Pi gets a new IP | **Yes.** The laptop learns the Pi's address from its packets; `demo.bat` runs `findpi` so `ssh pi` finds it. The first login to a new address may ask you to confirm the Pi's key. |
| A different Wi-Fi (another router, a phone hotspot) | **Yes, if** the Pi already knows that Wi-Fi's password, Windows marks it **Private** (Settings → Network → Wi-Fi → network profile), and the network lets devices talk to each other. |
| College / office / guest Wi-Fi | **Often no.** These usually block device-to-device traffic ("client isolation"). Use a phone hotspot instead. |
| Wi-Fi drops during a flight | The ssh session drops and the drone stops. Run `demo.bat` again. |

## If it doesn't connect

| Symptom | Fix |
|---|---|
| Dot stays grey | Is the Pi window running `start_drone.sh`? Does it print the laptop's current IP? |
| Still grey | Windows firewall: the Wi-Fi network must be **Private**, and Python allowed on Private networks (Windows asks the first time). |
| `ssh pi` fails | Run `findpi`; on a new network the Pi may be at a new address. |
| "Raspberry Pi option missing" on the page | An older website is still running. Restart it, or let `demo.bat` do it. |
| "cannot listen on UDP 14550" | Something else (Mission Planner, QGroundControl) already uses 14550 - close it. |
| Pi says `cannot fly that` | That plan can't be built; pick another in the planner. |

## What goes over the air

| Direction | Message | Why |
|---|---|---|
| Pi → laptop | `HEARTBEAT` | flight mode, as ArduCopter reports it: STABILIZE / AUTO / RTL / LAND |
| Pi → laptop | `ESC_TELEMETRY_1_TO_4`, `SYS_STATUS`, `BATTERY_STATUS`, `LOCAL_POSITION_NED` | the readings DroneGuard scores, 10 per flight-second |
| Pi → laptop | `NAMED_VALUE_FLOAT` f_motor / f_batt | simulation only: the true fault level, for the "really wrong" box |
| Pi → laptop | `STATUSTEXT` | "takeoff", "RTL received", "landed" - shown in the event log as `Pi: …` |
| laptop → Pi | `COMMAND_LONG` RETURN_TO_LAUNCH | DroneGuard's decision, exactly as it would go to a Pixhawk |
| laptop → Pi | `COMMAND_LONG` USER_1 / 2 / 3 | simulation only: inject fault / start a flight / stop |

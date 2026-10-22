# DroneGuard on a real drone (P5)

This guide moves DroneGuard from the virtual drone to a real one. The software
side is ready: `python -m live.monitor` is the program that runs on the Pi, and
it is the same code the tests fly against the virtual drone. What is left needs
parts, a bench and a flying field, and has **not been tested on hardware yet**.

## What you need

| Part | Why | Notes |
|---|---|---|
| Flight controller running **ArduPilot** (Pixhawk 2.4.8 / 6C or similar) | flies the drone, sends telemetry, obeys RTL | ArduPilot sends the `ESC_TELEMETRY_1_TO_4` message DroneGuard reads. PX4 sends `ESC_STATUS`/`ESC_INFO` instead, which isn't decoded yet. |
| ESCs **with telemetry** (BLHeli_32, AM32, or bidirectional DShot) | per-motor temperature and current | without them DroneGuard falls back to battery current and has no motor temperature |
| Power module with current sensor | pack voltage and current | calibrate it: DroneGuard's battery score depends on it |
| Raspberry Pi 4 / 5 (or Zero 2 W) | runs DroneGuard | powered from its own 5 V BEC, never from the flight controller |
| The rest of the parts list | motors, props, battery, frame | from the planner, after swapping in datasheet weights |

## 1. Make the spec file

In the planner (`python -m web`, http://127.0.0.1:8000), set the real payload,
hover time and trip, then press **Spec file for the Pi**. This saves a JSON file
with the limits and battery capacity the design was checked with. Fix
`Cap_Ah` in it if the pack you bought differs.

## 2. Wire the Pi to the flight controller

```
Flight controller TELEM2         Raspberry Pi GPIO header
  TX  ─────────────────────────▶  pin 10  GPIO15 (RX)
  RX  ◀─────────────────────────  pin 8   GPIO14 (TX)
  GND ──────────────────────────  pin 6   GND
  5V  ✗ leave unconnected - power the Pi from its own BEC
```

Both sides are 3.3 V logic on Pixhawk-class boards. Check yours.

## 3. Flight controller settings (ArduPilot)

| Parameter | Value | Meaning |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 | MAVLink 2 on TELEM2 |
| `SERIAL2_BAUD` | 921 | 921600 baud |
| `SR2_EXT_STAT` | 10 | SYS_STATUS and battery messages at 10 Hz |
| `SR2_POSITION` | 10 | LOCAL_POSITION_NED at 10 Hz |
| `SR2_EXTRA3` | 10 | further status messages, incl. battery |
| `BATT_CAPACITY` | your pack, mAh | must match `Cap_Ah` in the spec |
| ESC telemetry | per your ESC type, see ArduPilot's "ESC Telemetry" page | e.g. `SERVO_BLH_AUTO`, `SERVO_BLH_TRATE`, or `SERVO_BLH_BDMASK` for bidirectional DShot |

Which stream group carries which message differs between ArduPilot versions.
Check in Mission Planner's MAVLink Inspector that all four of these arrive at
about 10 Hz: `ESC_TELEMETRY_1_TO_4`, `SYS_STATUS`, `BATTERY_STATUS`,
`LOCAL_POSITION_NED`.

## 4. Set up the Pi

```bash
sudo raspi-config        # Interface Options -> Serial Port:
                         #   login shell over serial: NO, serial hardware: YES
# Pi 3/4: put  dtoverlay=disable-bt  in /boot/firmware/config.txt so /dev/serial0
# is the full UART, then reboot.

mkdir -p ~/droneguard && cd ~/droneguard
# copy the repo's python/ folder contents here, and the spec file as spec.json
python3 -m venv .venv
.venv/bin/pip install numpy pymavlink

.venv/bin/python -m live.monitor --connect /dev/serial0 --baud 921600 --spec spec.json
```

You should see `telemetry received - DroneGuard is monitoring`, then a status
line every second. To start it at boot, use `python/deploy/droneguard.service`.

## 5. Test before it flies, in this order

Never skip a step. At each one, the DroneGuard log must show what the step
expects before you move on.

1. **Desk, props off, USB power.** The monitor prints telemetry. Voltage matches a multimeter. Charge reads 100%.
2. **Bench, props off, battery in.** Arm, raise throttle slowly. Motor current and temperature rise in the monitor. Disarm.
3. **RTL command on the bench, props off.** Temporarily set `Warn` and `Alarm` to 101 in the spec, so any health counts as ALARM. DroneGuard must log `RETURN TO LAUNCH sent`, then `autopilot accepted RETURN TO LAUNCH`. ArduPilot refuses RTL without a GPS fix and home position; the log then says `REFUSED` with the reason code, so test outdoors with a fix. Restore the spec.
4. **Tethered hover**, low, in an open area. Watch health stay at 100 and current match the planner's hover current.
5. **Short line-of-sight flight**, with a geofence set and the pilot ready to take over on the RC transmitter at any moment.

DroneGuard only ever *advises*: it sends RTL, and the pilot and the flight
controller's own failsafes stay in charge.

## 6. Rules (India)

Before any outdoor flight, check the current requirements under the Drone Rules
2021 on the Digital Sky platform (digitalsky.dgca.gov.in): drone registration
(UIN), remote pilot certificate where it applies, and the airspace map. Green
zones allow flight up to 120 m without permission; yellow and red zones need it.
Rules change, so check the official source, not this page.

## Testing against SITL first (optional)

ArduPilot's simulator runs on Linux or WSL. It is not tested here:

```bash
sim_vehicle.py -v ArduCopter --out=udp:<windows-ip>:14550
python -m live.monitor --connect udpin:0.0.0.0:14550 --spec spec.json
```

SITL has no ESC telemetry, so DroneGuard says so and scores the motor on
battery current alone. Set `Cap_Ah` in the spec to SITL's `BATT_CAPACITY`.

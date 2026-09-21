# DroneGuard Laptop Mission Control

This is a standalone Python/Streamlit dashboard. It does not require MATLAB or Simulink.

```powershell
cd E:\drones_project\laptop_droneguard
python -m pip install -r requirements.txt
python -m streamlit run streamlit_app.py
```

Streamlit opens the dashboard in your browser (normally `http://localhost:8501`).
The simulator is paused until you choose **Start simulation**. It then flies an
outbound mission, develops motor and battery faults, and returns based on its
health-aware energy threshold.

Use the **Mission configuration** sidebar to set distance, payload, maximum
mission time, cruise speed, battery capacity, landing reserve, and fault start
times. Each fault begins a fixed 120-second ramp as soon as its selected start
time arrives. Choose **Apply configuration & reset** before starting the run.

## Moving to a real drone

Keep the dashboard and decision logic. Replace `SimulatedDrone.step()` in `app.py` with a MAVLink adapter that supplies `Telemetry` values from the flight controller. When the decision changes to `RETURNING`, send an RTL/mission-change MAVLink command.

For flight safety, retain the flight controller's own battery failsafe; this laptop tool should be an additional supervisory layer, not the only safety mechanism.

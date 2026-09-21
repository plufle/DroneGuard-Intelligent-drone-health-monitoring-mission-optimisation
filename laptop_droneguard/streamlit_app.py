"""DroneGuard laptop mission control — Streamlit UI, no MATLAB required."""
from __future__ import annotations

from dataclasses import asdict

import altair as alt
import pandas as pd
import streamlit as st

from app import SimulatedDrone, SimulationConfig


st.set_page_config(page_title="DroneGuard Laptop", page_icon="🚁", layout="wide")


def sensor_snapshot(t) -> str:
    return (f"SOC {t.soc * 100:.1f}% | V {t.voltage_v:.2f} V | I {t.current_a:.1f} A | "
            f"Temp {t.motor_temp_c:.1f} °C | Health {t.overall_health:.0f}/100")


def add_event(message: str) -> None:
    t = st.session_state.drone.t
    st.session_state.events.append({"Time (s)": round(t.time_s), "Event": message,
                                    "Sensor snapshot": sensor_snapshot(t)})


def reset(config: SimulationConfig, message: str = "Simulation reset — ready to start.") -> None:
    st.session_state.config = config
    st.session_state.drone = SimulatedDrone(config)
    st.session_state.running = False
    st.session_state.history = []
    st.session_state.events = []
    st.session_state.last_state = "OUTBOUND"
    st.session_state.last_alert = "OK"
    add_event(message)


def initialise() -> None:
    if "config" not in st.session_state:
        st.session_state.config = SimulationConfig()
    if "drone" not in st.session_state:
        reset(st.session_state.config, "System ready — simulation is paused.")


def two_axis_chart(frame: pd.DataFrame, left: str, left_label: str, right: str, right_label: str) -> alt.Chart:
    base = alt.Chart(frame).encode(x=alt.X("time_s:Q", title="Mission time [s]"))
    first = base.mark_line(color="#1f77b4", strokeWidth=2).encode(
        y=alt.Y(f"{left}:Q", title=left_label, axis=alt.Axis(titleColor="#1f77b4")))
    second = base.mark_line(color="#ff7f0e", strokeWidth=2).encode(
        y=alt.Y(f"{right}:Q", title=right_label,
                axis=alt.Axis(titleColor="#ff7f0e", orient="right")))
    return alt.layer(first, second).resolve_scale(y="independent").properties(height=270)


def multi_line_chart(frame: pd.DataFrame, columns: dict[str, str], y_label: str, title: str) -> alt.Chart:
    """A labelled chart with a legend and an explicit y-axis unit."""
    long = frame[["time_s", *columns]].rename(columns=columns).melt(
        id_vars="time_s", var_name="Metric", value_name="Value")
    return alt.Chart(long, title=title).mark_line(strokeWidth=2).encode(
        x=alt.X("time_s:Q", title="Mission time [s]"),
        y=alt.Y("Value:Q", title=y_label),
        color=alt.Color("Metric:N", title="Trace"),
        tooltip=[alt.Tooltip("time_s:Q", title="Time [s]"), "Metric:N", alt.Tooltip("Value:Q", format=".2f")]
    ).properties(height=270)


initialise()
st.title("🚁 DroneGuard Laptop Mission Control")
st.caption("Standalone simulated mission · continuous 1-second telemetry · ready to replace with live MAVLink telemetry")

with st.sidebar:
    st.header("Mission configuration")
    st.caption("Choose values, then apply them before starting a run.")
    current = st.session_state.config
    with st.form("mission_configuration"):
        mission_distance = st.number_input("Mission distance [m]", 100.0, 50000.0, current.mission_distance_m, 100.0)
        payload = st.number_input("Carrying load / payload [kg]", 0.0, 20.0, current.payload_kg, 0.1)
        max_time = st.number_input("Maximum mission time [s]", 30.0, 7200.0, current.max_time_s, 10.0)
        cruise_speed = st.number_input("Cruise speed [m/s]", 1.0, 40.0, current.cruise_speed_mps, 0.5)
        reserve = st.number_input("Landing reserve [%]", 0.0, 50.0, current.reserve_percent, 1.0)
        capacity = st.number_input("Battery capacity [Ah]", 0.5, 30.0, current.battery_capacity_ah, 0.1)
        waypoint_action = st.selectbox("When the mission distance is reached", ["Stop at waypoint", "Return home"],
                                       index=0 if current.waypoint_action == "Stop at waypoint" else 1)
        st.divider()
        st.caption("Fault simulation")
        motor_fault = st.number_input("Motor wear starts [s]", 0.0, 7200.0, current.motor_fault_start_s, 10.0)
        battery_fault = st.number_input("Battery ageing starts [s]", 0.0, 7200.0, current.battery_fault_start_s, 10.0)
        st.caption("Each fault ramps from 0% to 100% over 120 seconds, starting immediately at the chosen time.")
        apply_configuration = st.form_submit_button("Apply configuration & reset", use_container_width=True)
    new_config = SimulationConfig(mission_distance_m=mission_distance, payload_kg=payload, max_time_s=max_time,
                                  cruise_speed_mps=cruise_speed, reserve_percent=reserve, battery_capacity_ah=capacity,
                                  motor_fault_start_s=motor_fault, battery_fault_start_s=battery_fault,
                                  waypoint_action=waypoint_action)
    if apply_configuration:
        reset(new_config, "Configuration applied — simulation is paused.")
        st.success("Configuration applied. Start the new mission when ready.")

active = st.session_state.config
st.info(f"Active configuration: **{active.mission_distance_m:.0f} m** mission · **{active.payload_kg:.1f} kg** payload · "
        f"**{active.max_time_s:.0f} s** maximum time · waypoint action: **{active.waypoint_action}**")


@st.fragment(run_every="1s")
def live_mission() -> None:
    left, right = st.columns([1, 5])
    with left:
        if st.button("▶ Start simulation", type="primary", disabled=st.session_state.running, use_container_width=True):
            st.session_state.running = True
            add_event("Simulation started. Drone is departing home.")
        if st.button("↺ Reset run", use_container_width=True):
            reset(st.session_state.config)
            st.rerun()
    with right:
        status = "RUNNING — telemetry updates every second" if st.session_state.running else "PAUSED — press Start simulation"
        st.info(status)

    if st.session_state.running:
        t = st.session_state.drone.step()
        st.session_state.history.append(asdict(t))
        if t.alert != st.session_state.last_alert:
            add_event(f"Health alert changed: {st.session_state.last_alert} → {t.alert}.")
            st.session_state.last_alert = t.alert
        if t.state != st.session_state.last_state:
            add_event(f"Mission state changed: {st.session_state.last_state} → {t.state}. {t.reason}")
            st.session_state.last_state = t.state
        if t.state in ("HOME", "BATTERY DEAD", "TIME LIMIT", "WAYPOINT REACHED"):
            st.session_state.running = False

    t = st.session_state.drone.t
    alert_colour = {"OK": "🟢", "WARN": "🟡", "ALARM": "🔴"}[t.alert]
    values = [("Mission state", t.state), ("Distance from home", f"{t.distance_m:.0f} m"),
              ("Speed", f"{t.speed_mps:.1f} m/s"), ("Battery charge", f"{t.soc * 100:.1f}%"),
              ("Battery voltage", f"{t.voltage_v:.2f} V"), ("Motor current", f"{t.current_a:.1f} A"),
              ("Motor temperature", f"{t.motor_temp_c:.1f} °C"),
              ("Overall health", f"{t.overall_health:.0f} / 100"),
              ("Return-home threshold", f"{t.rth_threshold * 100:.1f}%"), ("Alert", f"{alert_colour} {t.alert}")]
    values.extend([("Motor fault severity", f"{t.motor_fault_pct:.0f}%"),
                   ("Battery fault severity", f"{t.battery_fault_pct:.0f}%")])
    metrics = st.columns(5)
    for index, (label, value) in enumerate(values):
        metrics[index % 5].metric(label, value)
    st.caption(f"Decision: **{t.reason}**")

    if not st.session_state.history:
        st.info("Press Start simulation to begin collecting telemetry and drawing the mission plots.")
    else:
        frame = pd.DataFrame(st.session_state.history)
        st.subheader("1. Mission — distance and speed")
        st.altair_chart(two_axis_chart(frame, "distance_m", "Distance from home [m]", "speed_mps", "Speed [m/s]"), use_container_width=True)

        st.subheader("2. Return-to-home decision")
        battery = frame.assign(charge_pct=frame.soc * 100, threshold_pct=frame.rth_threshold * 100)
        st.altair_chart(multi_line_chart(battery, {"charge_pct": "Charge remaining", "threshold_pct": "Return-home threshold"},
                                         "Charge [%]", "Battery charge vs. required return-home threshold"),
                         use_container_width=True)

        st.subheader("3. Motor — current and temperature")
        st.altair_chart(two_axis_chart(frame, "motor_temp_c", "Motor temperature [°C]", "current_a", "Motor current [A]"), use_container_width=True)

        st.subheader("4. Health — worst component controls the overall score")
        st.altair_chart(multi_line_chart(frame, {"motor_health": "Motor health", "battery_health": "Battery health",
                                                  "overall_health": "Overall health"}, "Health score [0–100]",
                                         "Health score — overall follows the weakest component"), use_container_width=True)

        st.subheader("5. Fault simulation — applied fault severity")
        st.altair_chart(multi_line_chart(frame, {"motor_fault_pct": "Motor / ESC wear", "battery_fault_pct": "Battery ageing"},
                                         "Fault severity [%]", "Injected fault severity"), use_container_width=True)

    st.subheader("How the health score is calculated")
    st.markdown("""
    - **Motor health** is the worse of temperature and current: temperature maps from **100 at 60 °C** to **0 at 100 °C**; current maps from **100 at 28 A** to **0 at 40 A**.
    - **Battery health** scores voltage sag (`expected voltage − measured voltage`): **100 at 0.10 V sag**, **0 at 1.50 V sag**.
    - **Overall health** is `min(motor health, battery health)`. **WARN** is below 70; **ALARM** is below 40 and commands return-to-home.
    - The return-home threshold estimates energy needed from distance, configured cruise speed, current draw and its rising trend, then adds the configured reserve.
    """)

    st.subheader("Live drone telemetry log")
    st.caption("One row arrives every second while the simulation runs. The table retains the full current mission.")
    if st.session_state.history:
        log = pd.DataFrame(st.session_state.history).assign(soc_pct=lambda d: d.soc * 100,
                                                              rth_pct=lambda d: d.rth_threshold * 100)
        log = log[["time_s", "state", "distance_m", "speed_mps", "soc_pct", "voltage_v", "current_a", "motor_temp_c",
                   "motor_fault_pct", "battery_fault_pct",
                   "motor_health", "battery_health", "overall_health", "rth_pct", "alert", "reason"]].rename(columns={
            "time_s": "Time [s]", "state": "State", "distance_m": "Distance [m]", "speed_mps": "Speed [m/s]",
            "soc_pct": "SOC [%]", "voltage_v": "Voltage [V]", "current_a": "Current [A]", "motor_temp_c": "Motor temp [°C]",
            "motor_fault_pct": "Motor fault [%]", "battery_fault_pct": "Battery fault [%]",
            "motor_health": "Motor health", "battery_health": "Battery health", "overall_health": "Overall health",
            "rth_pct": "RTH threshold [%]", "alert": "Alert", "reason": "Decision"})
        st.dataframe(log.iloc[::-1].round(2), hide_index=True, use_container_width=True, height=360)
    else:
        st.dataframe(pd.DataFrame(columns=["Time [s]", "State", "Distance [m]", "SOC [%]", "Voltage [V]", "Current [A]"]),
                     hide_index=True, use_container_width=True)

    st.subheader("Mission event log")
    st.dataframe(pd.DataFrame(st.session_state.events).iloc[::-1], hide_index=True, use_container_width=True)


live_mission()

function [motor_health, batt_health, overall_health, alert, ...
          speed_cmd, rth_threshold, soc_needed, mission_state] = ...
         droneguard_module(T_motor, I_motor, V_batt, soc, dist_home, use_dynamic)
%DRONEGUARD_MODULE  The whole DroneGuard companion computer, in one call.
%
%   Health scoring and mission optimisation are not two products bolted
%   together - they are one loop. The module reads telemetry, works out how
%   healthy the aircraft is, and then decides what the mission should do about
%   it. Running them as separate blocks made it far too easy for the mission
%   half to ignore the health half, which is exactly what happened.
%
%   This is the function that would be compiled to C and run on a Raspberry Pi
%   at 10 Hz alongside the flight controller.
%
%   Inputs  (telemetry + one config flag)
%     T_motor     [degC] motor temperature
%     I_motor     [A]    motor current
%     V_batt      [V]    pack voltage under load
%     soc         [-]    state of charge, 0..1
%     dist_home   [m]    distance from home
%     use_dynamic [-]    1 = health-aware RTH, 0 = naive fixed threshold
%
%   Outputs (advice, never direct control)
%     motor_health, batt_health, overall_health  0..100
%     alert          0 = OK, 1 = WARN, 2 = ALARM
%     speed_cmd      [m/s] forward speed command (negative = flying home)
%     rth_threshold  [-]   SOC at which we must turn back
%     soc_needed     [-]   charge the trip home would cost
%     mission_state  [-]   0 = outbound, 1 = returning, 2 = home

%#codegen

c = droneguard_thresholds();

% 1. How healthy is the aircraft, right now?
[motor_health, batt_health, overall_health, alert] = ...
    droneguard_health(T_motor, I_motor, V_batt, soc, c);

% 2. Given that, what should the mission do? The alert goes in here - this
%    single argument is the link that made the two halves one module.
[speed_cmd, rth_threshold, soc_needed, mission_state] = ...
    droneguard_mission(dist_home, soc, I_motor, use_dynamic, alert);
end

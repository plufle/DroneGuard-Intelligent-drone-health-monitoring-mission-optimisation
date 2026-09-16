function p = droneguard_thresholds()
%DRONEGUARD_THRESHOLDS  Single source of truth for everything the DroneGuard
%   module needs to know - health limits AND mission rules.
%
%   Returns plain literals only, so this function is safe to call from a
%   Simulink MATLAB Function block AND from generated C on the Raspberry Pi.
%   Edit the numbers here and both the simulation and the deployed code
%   change together.

p = struct( ...
    ... % ---- health score limits ----
    'T_good',     60,    ... % [degC] motor temperature still healthy
    'T_bad',      100,   ... % [degC] motor temperature critical
    'I_good',     28,    ... % [A]    motor current still healthy
    'I_bad',      40,    ... % [A]    motor current critical
    'Sag_good',   0.10,  ... % [V]    voltage sag beyond prediction - healthy
    'Sag_bad',    1.50,  ... % [V]    voltage sag beyond prediction - critical
    'Warn',       70,    ... % [-]    below this -> caution
    'Alarm',      40,    ... % [-]    below this -> abort / return to home
    ... % ---- what a healthy pack looks like ----
    'V_full',     16.8,  ... % [V]    open-circuit voltage at 100% SOC
    'V_empty',    14.0,  ... % [V]    open-circuit voltage at 0% SOC
    'R_int',      0.020, ... % [Ohm]  healthy internal resistance
    'Cap_Ah',     5.0,   ... % [Ah]   rated pack capacity
    ... % ---- mission rules ----
    'Range',      6000,  ... % [m]    how far out the mission goes before turning
    ...                      %         (deliberately further than the battery can
    ...                      %          manage, so the RTH rule is what turns us back)
    'Cruise',     12,    ... % [m/s]  commanded cruise speed
    'Reserve',    0.10,  ... % [-]    SOC we refuse to land below
    'Ts',         0.1,   ... % [s]    rate this module runs at
    'RTH_margin', 1.25,  ... % [-]    safety factor on the energy-to-home estimate
    'Trend_tau',  20,    ... % [s]    smoothing on the current-growth estimate
    'RTH_fixed',  0.25);     % [-]    the naive "always turn back at 25%" rule
end

function [motor_health, batt_health, overall_health, alert] = droneguard_health( ...
        T_motor, I_motor, V_batt, soc, p)
%DRONEGUARD_HEALTH  Component and overall drone health scores (0..100).
%
%   This is the algorithm that eventually runs on the Raspberry Pi.
%   It is deliberately plain MATLAB - no toolbox calls, no persistent state -
%   so it drops straight into a Simulink MATLAB Function block and is
%   supported by code generation.
%
%   Inputs
%     T_motor  [degC]  motor temperature
%     I_motor  [A]     motor current draw
%     V_batt   [V]     measured pack voltage under load
%     soc      [-]     state of charge, 0..1
%     p        struct  thresholds from droneguard_params.m
%
%   Outputs
%     motor_health, batt_health, overall_health   0..100
%     alert                                       0 = OK, 1 = WARN, 2 = ALARM

%% --- Motor: two independent indicators -----------------------------------
% Running hot and pulling too much current are different symptoms of the
% same wear, so score each one and keep the worse of the two.
h_temp    = linScore(T_motor, p.T_good, p.T_bad);
h_current = linScore(I_motor, p.I_good, p.I_bad);
motor_health = min(h_temp, h_current);

%% --- Battery: voltage sag beyond what the SOC predicts --------------------
% A healthy pack at a known SOC and current has a predictable terminal
% voltage. Anything sagging below that means the internal resistance has
% risen - the classic ageing signature.
V_expected = p.V_empty + (p.V_full - p.V_empty) * soc - I_motor * p.R_int;
sag        = V_expected - V_batt;
batt_health = linScore(sag, p.Sag_good, p.Sag_bad);

%% --- Fuse into one number ------------------------------------------------
% Deliberately min(), NOT mean(). Averaging would let a healthy battery
% mask a dying motor - exactly the silent failure this project exists to
% catch.
overall_health = min(motor_health, batt_health);

%% --- Turn the score into the one thing the operator acts on ---------------
if overall_health < p.Alarm
    alert = 2;      % abort / return to home
elseif overall_health < p.Warn
    alert = 1;      % caution
else
    alert = 0;      % nominal
end
end


function s = linScore(x, goodVal, badVal)
%LINSCORE  Map an indicator onto 100 (at goodVal) .. 0 (at badVal), clamped.
%   Works whether "bad" is a higher or a lower number than "good".
s = 100 * (badVal - x) / (badVal - goodVal);
if s > 100
    s = 100;
elseif s < 0
    s = 0;
end
end

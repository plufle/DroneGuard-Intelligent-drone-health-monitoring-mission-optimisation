function [speed_cmd, rth_threshold, soc_needed, mission_state] = droneguard_mission( ...
        dist_home, soc, I_motor, use_dynamic, alert)
%DRONEGUARD_MISSION  Mission optimisation: decide when to turn back.
%
%   This is the second half of DroneGuard. It runs on the same Raspberry Pi
%   as the health score and answers one question continuously:
%
%       "If I turn around RIGHT NOW, do I have enough charge to get home?"
%
%   Inputs
%     dist_home   [m]  how far the drone currently is from home
%     soc         [-]  state of charge, 0..1
%     I_motor     [A]  current being drawn right now
%     use_dynamic [-]  1 = health-aware threshold, 0 = naive fixed threshold
%     alert       [-]  0 = OK, 1 = WARN, 2 = ALARM, from the health score
%
%   Outputs
%     speed_cmd     [m/s] forward speed command (negative = flying home)
%     rth_threshold [-]   the SOC at which we must turn back
%     soc_needed    [-]   charge the trip home would actually cost
%     mission_state [-]   0 = outbound, 1 = returning, 2 = home

%#codegen

persistent state I_prev dI_smooth
if isempty(state)
    state     = 0;      % 0 = outbound, 1 = returning, 2 = home
    I_prev    = I_motor;
    dI_smooth = 0;
end

c = droneguard_thresholds();

%% --- What would the trip home actually cost? -----------------------------
% This is the whole idea. A healthy drone sips current and needs little to get
% back. A degraded one draws far more for the same distance, so the charge it
% must hold in reserve grows - automatically, without anyone re-tuning a limit.
%
% The subtlety: using the current we see RIGHT NOW under-predicts, because the
% motor keeps degrading all the way home. So we measure how fast the draw is
% growing and project it over the return trip. Only upward trends count - we
% never talk ourselves into believing the trip home got cheaper.
d_home = max(dist_home, 0);                         % [m]  never negative
t_home = d_home / c.Cruise;                         % [s]  time to fly home

dI_raw    = (I_motor - I_prev) / c.Ts;              % [A/s] how fast draw is rising
alpha     = c.Ts / c.Trend_tau;
dI_smooth = dI_smooth + alpha * (dI_raw - dI_smooth);
I_prev    = I_motor;

I_expected = I_motor + 0.5 * max(dI_smooth, 0) * t_home;   % [A] average on the way home
Ah_home    = I_expected * t_home / 3600;                   % [Ah]
soc_needed = (Ah_home / c.Cap_Ah) * c.RTH_margin;          % [-] plus a safety factor

if use_dynamic > 0.5
    rth_threshold = soc_needed + c.Reserve;
else
    rth_threshold = c.RTH_fixed;   % the naive rule, blind to how the drone feels
end

%% --- Mission state machine ----------------------------------------------
if state == 0                                   % outbound
    % Three independent reasons to stop heading away from home. The third is
    % the health score overruling the energy maths: a drone that is not fit to
    % fly should not keep flying outbound just because the charge adds up. The
    % cheapest trip home is always the one you start soonest.
    reached_waypoint = dist_home >= c.Range;
    out_of_margin    = soc <= rth_threshold;
    unfit_to_continue = alert >= 2;
    if reached_waypoint || out_of_margin || unfit_to_continue
        state = 1;
    end
elseif state == 1                               % returning
    if dist_home <= 1.0
        state = 2;                              % home
    end
end

%% --- Turn the state into a command --------------------------------------
if state == 0
    speed_cmd =  c.Cruise;
elseif state == 1
    speed_cmd = -c.Cruise;
else
    speed_cmd =  0;
end

mission_state = state;
end

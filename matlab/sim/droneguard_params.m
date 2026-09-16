%% DRONEGUARD - Simulation Parameters
% Run this BEFORE simulating DroneGuard_Sim.slx.
% Every number the model uses lives here - tweak these, not the blocks.

%% ---------- Solver / mission ----------
Ts      = 0.1;   % [s]  health-monitoring loop rate (10 Hz)
T_end   = 800;   % [s]  how long we let the mission run before giving up
Alt_cmd = 50;    % [m]  commanded cruise altitude

% Forward speed is no longer a constant - the mission manager commands it, so
% it can send the drone home. Cruise speed and mission range live in
% droneguard_thresholds.m alongside the rest of the module's config.
Use_Dynamic_RTH = 1;   % 1 = health-aware return-to-home, 0 = naive fixed 25%

%% ---------- Airframe ----------
m_drone    = 2.0;   % [kg]        all-up mass
g_accel    = 9.81;  % [m/s^2]     gravity
Cd_forward = 0.55;  % [N/(m/s)]   forward drag coefficient (linear)
Accel_max  = 2.5;   % [m/s^2]     how fast the speed command may change - a real
                    %             drone cannot reverse instantly, and without
                    %             this the turnaround looks like a current spike

% Trim condition the drone starts in, so the mission begins already cruising.
% Set these to 0 instead if you want to watch it take off and accelerate.
Alt_init   = Alt_cmd;
Speed_init = 0;

%% ---------- Flight controller gains ----------
% Altitude is a double integrator, so it needs derivative action to damp.
Kp_alt = 8.0;   Ki_alt = 0.5;   Kd_alt = 12.0;
% Forward speed is first-order (drag), so PI is enough.
Kp_spd = 6.0;   Ki_spd = 2.0;

%% ---------- Propulsion (motors + ESCs) ----------
k_thrust  = 1.09;  % [N/A]        thrust produced per amp when healthy
K_therm   = 0.055; % [degC/A^2]   steady-state temperature rise per amp squared
tau_therm = 40;    % [s]          motor thermal time constant
T_ambient = 30;    % [degC]       ambient air temperature

%% ---------- Battery (4S LiPo) ----------
% Pack constants come from droneguard_thresholds.m so that the plant and the
% health algorithm can never disagree about what a healthy pack looks like.
cfg      = droneguard_thresholds();
V_full   = cfg.V_full;    % [V]
V_empty  = cfg.V_empty;   % [V]
R_int    = cfg.R_int;     % [Ohm]
Cap_Ah   = cfg.Cap_Ah;    % [Ah]  rated capacity when new
SOC_init = 1.0;           % [-]   state of charge at the start of the mission

%% ---------- FAULT 1: Motor / ESC degradation ----------
% Worn bearings and rising winding resistance mean less thrust per amp. The
% flight controller still has to hold altitude, so it simply demands more
% current - and that extra current is what shows up as heat. The fault is
% never injected into the current directly; the closed loop produces it.
MotorFault_Start = 150;   % [s]     when degradation begins
MotorFault_Mag   = 1.0;   % [0..1]  severity reached by end of mission
k_eff_fault      = 0.50;  % [-]     fraction of thrust-per-amp lost at full severity
MotorFault_Slope = MotorFault_Mag / (T_end - MotorFault_Start);

%% ---------- FAULT 2: Battery ageing ----------
BattFault_Start = 400;   % [s]     when ageing starts to bite
BattFault_Mag   = 1.0;   % [0..1]  severity reached by end of mission
k_R_fault       = 3.00;  % [-]     resistance multiplier at full severity = 1 + 3.00
k_drain_fault   = 0.67;  % [-]     SOC drains this much faster at full severity
BattFault_Slope = BattFault_Mag / (T_end - BattFault_Start);

disp('DroneGuard parameters loaded.');

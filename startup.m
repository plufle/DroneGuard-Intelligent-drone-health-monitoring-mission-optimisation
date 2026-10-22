%STARTUP  Put DroneGuard on the MATLAB path.
%
%   MATLAB runs this automatically if you start it in this folder.
%   Otherwise, once per session:
%
%       cd <this folder>
%       startup

droneguard_root = fileparts(mfilename('fullpath'));
addpath(fullfile(droneguard_root, 'matlab', 'src'));   % algorithm (deploys to the Pi)
addpath(fullfile(droneguard_root, 'matlab', 'sim'));   % simulation-only helpers
addpath(fullfile(droneguard_root, 'matlab', 'model')); % the Simulink model
clear droneguard_root

fprintf('\nDroneGuard ready. Commands:\n');
fprintf('  droneguard_run              fly one mission and plot it\n');
fprintf('  droneguard_run(''fixed'')     same flight, naive fixed 25%% turn-back rule\n');
fprintf('  droneguard_compare          fly both and compare side by side\n');
fprintf('  droneguard_dashboard        watch the mission play back live\n');
fprintf('  droneguard_export           save a mission to CSV to check the Python port\n');
fprintf('  open_system(''DroneGuard_Sim'')   open the block diagram\n\n');

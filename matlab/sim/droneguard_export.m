function droneguard_export(file, mode)
%DRONEGUARD_EXPORT  Save one simulated mission to CSV for the Python port to check.
%
%   droneguard_export                      writes python/tests/matlab_dynamic.csv
%   droneguard_export(file, 'fixed')       any file, either rule
%
%   Columns: t, then the 14 simData columns in order. The Python script
%   python/tools/compare_with_matlab.py reads this and reports the largest
%   difference in every signal.

arguments
    file (1,:) char = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
                               'python', 'tests', 'matlab_dynamic.csv')
    mode (1,:) char {mustBeMember(mode, {'dynamic','fixed'})} = 'dynamic'
end

evalin('base', 'droneguard_params;');
assignin('base', 'Use_Dynamic_RTH', double(strcmp(mode, 'dynamic')));
out = evalin('base', "sim('DroneGuard_Sim')");

names = {'t','alt','speed','dist','T_motor','I_motor','V_batt','soc', ...
         'motor_h','batt_h','overall_h','alert','rth_threshold', ...
         'soc_needed','mission_state'};
T = array2table([out.tout out.simData], 'VariableNames', names);
writetable(T, file);
fprintf('Wrote %d rows to %s\n', height(T), file);
end

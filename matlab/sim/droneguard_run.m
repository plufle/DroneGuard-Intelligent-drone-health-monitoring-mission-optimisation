function out = droneguard_run(mode)
%DRONEGUARD_RUN  Fly one DroneGuard mission and plot the result.
%
%   droneguard_run              fly using DroneGuard's health-aware
%                               return-to-home threshold (the default)
%   droneguard_run('fixed')     fly the identical mission using the naive
%                               "always turn back at 25%" rule instead
%   out = droneguard_run(...)   also return the raw simulation output
%
%   Parameters are loaded into the BASE workspace, not here, because that is
%   where Simulink looks up the values behind block expressions like
%   Gain = k_thrust. It also means you can poke at them afterwards.
%
%   See also DRONEGUARD_COMPARE, DRONEGUARD_PLOT.

arguments
    mode (1,:) char {mustBeMember(mode, {'dynamic','fixed'})} = 'dynamic'
end

evalin('base', 'droneguard_params;');
assignin('base', 'Use_Dynamic_RTH', double(strcmp(mode, 'dynamic')));

out = evalin('base', "sim('DroneGuard_Sim')");
droneguard_plot(out);

if nargout == 0
    clear out
end
end

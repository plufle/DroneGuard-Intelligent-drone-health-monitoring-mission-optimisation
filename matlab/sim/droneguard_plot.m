function droneguard_plot(out)
%DRONEGUARD_PLOT  Plot one DroneGuard mission.
%
%   droneguard_plot(out)   where out = sim('DroneGuard_Sim')
%
%   Four stacked views: where the drone went, the return-to-home decision,
%   what the motor did, and what the health module concluded.

c = droneguard_thresholds();
d = out.simData;  t = out.tout;

% The mission ends when the drone is home or the pack is empty; everything
% after that is a model artefact, so trim it off before plotting.
iEnd = droneguard_missionend(d(:,14), d(:,7));
d = d(1:iEnd,:);  t = t(1:iEnd);

spd   = d(:,2);  dist  = d(:,3);  T_mot = d(:,4);  I_mot = d(:,5);
soc   = d(:,7);  mot_h = d(:,8);  bat_h = d(:,9);  ovr   = d(:,10);
alert = d(:,11); rth   = d(:,12); state = d(:,14);

iTurn = find(state >= 1, 1);
iHome = find(state >= 2, 1);

fig = figure('Name','DroneGuard Mission','Position',[80 40 900 900]);
if exist('theme','file')      % theme() needs R2025a or newer
    theme(fig,'light');       % keep plots readable whatever the desktop theme is
end

% --- Mission ------------------------------------------------------------
ax1 = subplot(4,1,1);
yyaxis left
plot(t, dist, 'LineWidth',1.8); ylabel('Distance from home [m]');
yline(c.Range, ':', 'waypoint', 'LabelHorizontalAlignment','left', ...
      'HandleVisibility','off');
yyaxis right
plot(t, spd, 'LineWidth',1.2); ylabel('Speed [m/s]');
title('Mission - out, turn, and back');
grid on

% --- The decision -------------------------------------------------------
ax2 = subplot(4,1,2);
hS = plot(t, soc*100, 'LineWidth',2.0); hold on
hR = plot(t, rth*100, '--', 'LineWidth',1.8);
yline(c.Reserve*100, ':', 'reserve', 'HandleVisibility','off');
ylabel('Charge [%]'); ylim([0 105]);
legend([hS hR], {'charge remaining','charge needed to get home'}, ...
       'Location','northeast');
title('The return-to-home decision - turn back where these two lines meet');
grid on

% --- Motor --------------------------------------------------------------
ax3 = subplot(4,1,3);
yyaxis left
plot(t, T_mot, 'LineWidth',1.6); ylabel('Motor temp [\circC]');
yyaxis right
plot(t, I_mot, 'LineWidth',1.6); ylabel('Motor current [A]');
title('Motor - the controller pulls more current to hold the same flight');
grid on

% --- Health -------------------------------------------------------------
ax4 = subplot(4,1,4);
% Overall is by definition min(motor, battery), so it always sits exactly on
% top of whichever component is worse - drawing it as a normal line would hide
% that component completely. Draw it first as a wide pale band instead, then
% the two components thin on top, so you can see all three at once.
hO = plot(t, ovr, 'LineWidth', 6, 'Color', [0 0 0 0.18]); hold on
hM = plot(t, mot_h, 'LineWidth', 1.6, 'Color', [0.00 0.45 0.74]);   % blue
hB = plot(t, bat_h, 'LineWidth', 1.6, 'Color', [0.85 0.33 0.10]);   % orange
yline(c.Warn,  '--', 'WARN',  'HandleVisibility','off');
yline(c.Alarm, '--', 'ALARM', 'HandleVisibility','off');
ylim([-2 105]); ylabel('Health score'); xlabel('Time [s]');
legend([hM hB hO], {'Motor','Battery','Overall (the worse of the two)'}, ...
       'Location','southwest');
title('Health - overall is the WORST component, never the average');
grid on

linkaxes([ax1 ax2 ax3 ax4],'x'); xlim([0 t(end)]);

for ax = [ax1 ax2 ax3 ax4]
    if ~isempty(iTurn)
        xline(ax, t(iTurn), '-', 'turn back', 'Color',[.7 .2 .1], 'Alpha',0.6, ...
            'LabelOrientation','horizontal','LabelVerticalAlignment','top', ...
            'LabelHorizontalAlignment','right','HandleVisibility','off');
    end
    if ~isempty(iHome)
        xline(ax, t(iHome), '-', 'home', 'Color',[0 .5 .2], 'Alpha',0.6, ...
            'LabelOrientation','horizontal','LabelVerticalAlignment','bottom', ...
            'LabelHorizontalAlignment','right','HandleVisibility','off');
    end
end

% --- Console summary ----------------------------------------------------
fprintf('\n--- DroneGuard mission summary ---\n');
fprintf('Waypoint            : %d m out\n', c.Range);
if ~isempty(iTurn)
    why = 'ran out of energy margin';
    if dist(iTurn) >= c.Range - 5, why = 'reached the waypoint'; end
    fprintf('Turned back         : t = %.0f s at %.0f m  (%s)\n', ...
        t(iTurn), dist(iTurn), why);
end
if ~isempty(iHome)
    fprintf('Outcome             : HOME at t = %.0f s with %.0f%% charge left\n', ...
        t(iHome), 100*soc(iHome));
else
    iDead = find(soc <= 0, 1);
    if isempty(iDead)
        fprintf('Outcome             : still flying when the run ended, %.0f m out\n', dist(end));
    else
        fprintf('Outcome             : BATTERY DEAD at t = %.0f s, %.0f m from home\n', ...
            t(iDead), dist(iDead));
    end
end
fprintf('Cruise current      : %.1f A -> %.1f A\n', I_mot(1), max(I_mot));
fprintf('Peak motor temp     : %.0f C\n', max(T_mot));
iw = find(alert >= 1, 1);  ia = find(alert >= 2, 1);
if ~isempty(iw), fprintf('WARN  raised at     : %.0f s\n', t(iw)); end
if ~isempty(ia), fprintf('ALARM raised at     : %.0f s\n', t(ia)); end
end

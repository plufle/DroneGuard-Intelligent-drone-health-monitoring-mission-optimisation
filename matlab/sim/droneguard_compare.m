function droneguard_compare()
%DRONEGUARD_COMPARE  Fly the same degrading drone twice and compare everything.
%
%   droneguard_compare
%
%   Run 1 uses DroneGuard's health-aware return-to-home threshold.
%   Run 2 uses the naive "always turn back at 25%" rule.
%   Same airframe, same faults, same mission - only the decision rule differs.
%
%   Every run is drawn only up to the point where its mission actually ended
%   (home, or battery empty). A circle marks that moment on each trace.

c = droneguard_thresholds();
evalin('base', 'droneguard_params;');

labels = {'DroneGuard (dynamic)', 'Fixed 25% rule'};
cols   = [0.00 0.45 0.74;      % blue  - dynamic
          0.85 0.33 0.10];     % orange - fixed
runs   = cell(1,2);

for k = 1:2
    assignin('base', 'Use_Dynamic_RTH', double(k == 1));
    o  = evalin('base', "sim('DroneGuard_Sim')");
    dk = o.simData;  tk = o.tout;
    iEnd = droneguard_missionend(dk(:,14), dk(:,7));   % stop at mission end
    runs{k} = struct('d', dk(1:iEnd,:), 't', tk(1:iEnd));
end

fig = figure('Name','DroneGuard - dynamic vs fixed return-to-home', ...
             'Position',[60 20 940 1000]);
if exist('theme','file')      % theme() needs R2025a or newer
    theme(fig,'light');       % keep plots readable whatever the desktop theme is
end

%  column, scale, y-label, title
panels = { 3, 1,   'Distance from home [m]', 'Where the drone went';
           7, 100, 'Charge [%]',             'Charge remaining, and the turn-back threshold each rule used (dotted)';
           5, 1,   'Motor current [A]',      'Motor current - the wear showing up as extra draw';
           4, 1,   'Motor temp [\circC]',    'Motor temperature';
          10, 1,   'Health score',           'Overall health - near identical in both runs. The rule does not change how sick the drone gets, only where it is when that happens.' };

ax = gobjects(1, size(panels,1));
for pnl = 1:size(panels,1)
    ax(pnl) = subplot(size(panels,1), 1, pnl); hold on
    col   = panels{pnl,1};
    scale = panels{pnl,2};

    % Run 1 is drawn thick and solid, run 2 thin and dashed ON TOP. On the
    % motor and health panels the two runs are almost identical - the drone
    % gets just as sick either way - so without this the first trace vanishes
    % underneath the second and the plot looks like it only has one line.
    styles = {'-','--'};
    widths = [2.8, 1.5];
    for k = 1:2
        d = runs{k}.d;  t = runs{k}.t;
        plot(t, d(:,col)*scale, styles{k}, 'LineWidth',widths(k), 'Color',cols(k,:));
        plot(t(end), d(end,col)*scale, 'o', 'MarkerSize',8, 'LineWidth',2, ...
             'Color',cols(k,:), 'HandleVisibility','off');
        % on the charge panel, overlay each run's own turn-back threshold
        if col == 7
            plot(t, d(:,12)*100, ':', 'LineWidth',1.6, 'Color',cols(k,:), ...
                 'HandleVisibility','off');
        end
    end

    ylabel(panels{pnl,3});
    title(panels{pnl,4});
    grid on
end

% Reference lines that give each panel its meaning
yline(ax(1), c.Range, ':', 'waypoint', 'HandleVisibility','off');
yline(ax(2), 0, '-', 'empty', 'Color',[.7 0 0], 'HandleVisibility','off');
ylim(ax(2), [-2 105]);
yline(ax(4), c.T_bad, ':', 'critical', 'HandleVisibility','off');
yline(ax(5), c.Warn,  '--', 'WARN',  'HandleVisibility','off');
yline(ax(5), c.Alarm, '--', 'ALARM', 'HandleVisibility','off');
ylim(ax(5), [-2 105]);

legend(ax(1), labels, 'Location','northwest');
xlabel(ax(end), 'Time [s]');
linkaxes(ax, 'x');
xlim(ax(1), [0 max(runs{1}.t(end), runs{2}.t(end))]);

%% --- Console summary -----------------------------------------------------
fprintf('\n--- return-to-home rule comparison ---\n');
fprintf('%-22s %9s %9s %9s %9s  %s\n', '', 'turned at', 'peak I', 'peak T', 'end health', 'outcome');
for k = 1:2
    d = runs{k}.d;  t = runs{k}.t;
    iTurn = find(d(:,14) >= 1, 1);
    iHome = find(d(:,14) >= 2, 1);
    if isempty(iHome)
        outcome = sprintf('BATTERY DEAD %.0f m from home', d(end,3));
    else
        outcome = sprintf('home at %.0f s with %.0f%% left', t(iHome), 100*d(iHome,7));
    end
    fprintf('%-22s %7.0f m %7.1f A %7.0f C %9.0f  %s\n', ...
        labels{k}, d(iTurn,3), max(d(:,5)), max(d(:,4)), d(end,10), outcome);
end

assignin('base', 'Use_Dynamic_RTH', 1);
end

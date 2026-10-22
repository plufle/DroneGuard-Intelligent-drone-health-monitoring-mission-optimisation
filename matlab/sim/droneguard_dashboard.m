function droneguard_dashboard(mode, opts)
%DRONEGUARD_DASHBOARD  Watch the mission fly, with the gauges updating live.
%
%   droneguard_dashboard                              play the default mission
%   droneguard_dashboard('fixed')                     play the naive 25% run
%   droneguard_dashboard('dynamic', Speed=5)          slow - 5x real time
%   droneguard_dashboard('dynamic', Speed=1)          real time (takes 11 min)
%   droneguard_dashboard('dynamic', Speed=60)         fast
%   droneguard_dashboard('dynamic', Save="run.mp4")   also write a video
%   droneguard_dashboard('dynamic', Until=330)        stop and hold at t=330 s
%   droneguard_dashboard('dynamic', Pace=false)       render flat out, ignore Speed
%
%   Speed is a real multiple of wall-clock time: the 649 s default mission
%   takes 649/Speed seconds to watch. Playback is paced to honour it, so a
%   low Speed genuinely slows the animation down rather than just drawing
%   more frames. Pace=false drops the pacing and renders as fast as the
%   graphics allow, which is what you want for a quick look or a screenshot.
%
%   The simulation runs first, then the logged result is played back. Playback
%   rather than live animation, because the solver does not run in wall-clock
%   time - but what you see is exactly the simulated flight, frame for frame.
%
%   A note on the camera panel: the model is longitudinal + vertical only, so
%   there is no 3D world and no camera in the physics. That panel is RENDERED
%   FROM POSITION - the ground scrolls at the true ground speed and the field
%   of view widens correctly with altitude, but it is scenery, not sensor data.

arguments
    mode (1,:) char {mustBeMember(mode,{'dynamic','fixed'})} = 'dynamic'
    opts.Speed (1,1) double {mustBePositive} = 20      % x real time; 1 = real time
    opts.Save  (1,:) string = ""                       % mp4 filename, "" = none
    opts.Until (1,1) double = inf                      % freeze at this sim time [s]
    opts.Pace  (1,1) logical = true                    % false = render flat out
end

c = droneguard_thresholds();

%% ---------- fly it ------------------------------------------------------
evalin('base','droneguard_params;');
assignin('base','Use_Dynamic_RTH', double(strcmp(mode,'dynamic')));
o = evalin('base', "sim('DroneGuard_Sim')");

d = o.simData;  t = o.tout;
iEnd = droneguard_missionend(d(:,14), d(:,7));
d = d(1:iEnd,:);  t = t(1:iEnd);

alt  = d(:,1);  spd = d(:,2);  dist = d(:,3);  T_mot = d(:,4);
I_mot= d(:,5);  soc = d(:,7);  mot_h= d(:,8);  bat_h = d(:,9);
ovr  = d(:,10); alert = d(:,11); rth = d(:,12); need = d(:,13); state = d(:,14);

% How far from home could it still get back from, right now? Derived from the
% module's own numbers so it agrees with the decision exactly: the turn happens
% when this shrinks to meet the drone's actual distance.
dRet = dist .* max(soc - c.Reserve, 0) ./ max(need, 1e-6);

dt     = t(2) - t(1);
stride = max(1, round(opts.Speed / (30*dt)));       % aim for ~30 fps
frames = 1:stride:numel(t);
if frames(end) ~= numel(t), frames(end+1) = numel(t); end
frames = frames(t(frames) <= opts.Until);          % optional freeze point
if isempty(frames), frames = 1; end

%% ---------- figure ------------------------------------------------------
fig = figure('Name','DroneGuard - mission playback','Color','w', ...
             'Position',[40 40 1280 760]);
if exist('theme','file'), theme(fig,'light'); end

xMax = max(max(dist)*1.08, c.Range*1.05);
yMax = max(alt)*1.6 + 5;

% --- scene: side-on view ------------------------------------------------
axScene = axes(fig,'Position',[0.045 0.57 0.52 0.33]); hold(axScene,'on');
fill(axScene,[0 xMax xMax 0],[0 0 -yMax*0.12 -yMax*0.12],[0.80 0.86 0.74], ...
     'EdgeColor',[0.55 0.62 0.48]);
xline(axScene, c.Range, ':', 'waypoint', 'Color',[0.35 0.35 0.35], ...
      'LabelVerticalAlignment','bottom');
plot(axScene, 0, 0, 'ks','MarkerSize',11,'MarkerFaceColor',[0.30 0.55 0.85]);
text(axScene, 0, -yMax*0.058, ' HOME','FontSize',8,'Color',[0.25 0.35 0.5]);
% everything beyond this line is unreachable - it moves as the drone degrades
hNoGo  = fill(axScene, nan, nan, [0.90 0.35 0.30], 'FaceAlpha',0.13, ...
              'EdgeColor','none');
hRetLn = plot(axScene, nan, nan, '-', 'Color',[0.85 0.25 0.20], 'LineWidth',1.4);
hTrail = plot(axScene, nan, nan, '-', 'Color',[0.30 0.55 0.85], 'LineWidth',1.4);
hDrone = plot(axScene, nan, nan, '-', 'LineWidth',2.6, 'Color',[0.1 0.5 0.2]);
text(axScene, xMax*0.995, yMax*0.90, 'cannot get home from beyond here  ', ...
     'Color',[0.75 0.25 0.20],'FontSize',8,'HorizontalAlignment','right');
xlim(axScene,[-xMax*0.02 xMax]); ylim(axScene,[-yMax*0.12 yMax]);
xlabel(axScene,'Distance from home [m]'); ylabel(axScene,'Altitude [m]');
title(axScene,'Mission','FontSize',10);
grid(axScene,'on'); box(axScene,'on');

% --- FPV: nose camera flying down a city corridor ------------------------
% Real perspective projection driven by the drone's own pose. The city is
% geometry the camera actually flies through, not a scrolling picture - but
% it is scenery only: the model has no obstacles and nothing can be hit.
% The axes has to live inside a uipanel. A 3D axes with a perspective camera
% sitting INSIDE its own data volume projects nearby geometry to enormous
% screen coordinates, and MATLAB does not clip that to the axes rectangle -
% the FPV simply paints over the whole figure. A uipanel does clip its
% children, so it is the container that keeps the view in its box.
pnlFPV = uipanel(fig,'Units','normalized','Position',[0.045 0.07 0.28 0.41], ...
                 'BorderType','line','BackgroundColor',[0.62 0.74 0.88], ...
                 'Title','FPV nose camera  (city is scenery, not obstacles)', ...
                 'FontSize',9,'TitlePosition','centertop');
axFPV = axes(pnlFPV,'Position',[0 0 1 1]);
[V, F, C] = cityBlocks(xMax);
patch(axFPV,'Vertices',V,'Faces',F,'FaceVertexCData',C, ...
      'FaceColor','flat','EdgeColor',[0.25 0.25 0.28],'LineWidth',0.1);
hold(axFPV,'on');
% ground and the road the drone follows
patch(axFPV,'XData',[-300 xMax+300 xMax+300 -300],'YData',[-260 -260 260 260], ...
      'ZData',[0 0 0 0],'FaceColor',[0.42 0.46 0.38],'EdgeColor','none');
patch(axFPV,'XData',[-300 xMax+300 xMax+300 -300],'YData',[-14 -14 14 14], ...
      'ZData',[0.4 0.4 0.4 0.4],'FaceColor',[0.30 0.30 0.32],'EdgeColor','none');
% The view volume is a CUBE that travels with the drone, not the whole 7 km
% route. With a perspective camera sitting inside the box, a box that long
% projects geometry far outside the panel and the FPV spills over the rest of
% the figure. A cube of side 2*fpvW keeps the projection sane, keeps the
% proportions isotropic, and doubles as the view distance.
fpvW = 320;                                  % view half-width [m]
set(axFPV,'Projection','perspective','Color',[0.62 0.74 0.88], ...
          'XTick',[],'YTick',[],'ZTick',[],'Box','on', ...
          'PlotBoxAspectRatio',[1 1 1], ...
          'XLim',[-fpvW fpvW],'YLim',[-fpvW fpvW],'ZLim',[0 2*fpvW], ...
          'CameraUpVector',[0 0 1],'CameraViewAngle',75, ...
          'Clipping','on','SortMethod','depth');
% A fixed sun rather than camlight('headlight'): camlight needs a valid camera
% position, which does not exist until the first frame, and an infinite light
% never has to be moved as the drone flies.
light(axFPV,'Position',[-0.6 -0.5 1],'Style','infinite');
lighting(axFPV,'gouraud'); material(axFPV,'dull');
set(axFPV,'CameraPosition',[0 0 50],'CameraTarget',[60 0 43]);
% HUD drawn on its own transparent axes, inside the same panel so it is
% clipped identically and always sits on top of the 3D view
axHUD = axes(pnlFPV,'Position',[0 0 1 1],'Color','none', ...
             'XTick',[],'YTick',[],'XLim',[0 1],'YLim',[0 1],'HitTest','off');
axHUD.XAxis.Visible = 'off'; axHUD.YAxis.Visible = 'off';
hold(axHUD,'on');
plot(axHUD,[0.47 0.53],[0.5 0.5],'w-','LineWidth',1.2);
plot(axHUD,[0.5 0.5],[0.47 0.53],'w-','LineWidth',1.2);
hCamTxt = text(axHUD,0.03,0.97,'','Color','w','FontSize',9, ...
               'FontName','Consolas','VerticalAlignment','top');

% --- status -------------------------------------------------------------
axStat = axes(fig,'Position',[0.35 0.07 0.215 0.41]);
axis(axStat,[0 1 0 1]); axis(axStat,'off');
hLamp = rectangle(axStat,'Position',[0.05 0.83 0.90 0.14],'Curvature',0.35, ...
                  'FaceColor',[0.2 0.7 0.3],'EdgeColor','none');
hLampTxt = text(axStat,0.5,0.90,'NOMINAL','FontWeight','bold','FontSize',13, ...
                'HorizontalAlignment','center','Color','w');
hStat = text(axStat,0.04,0.75,'','FontSize',11,'FontName','Consolas', ...
             'VerticalAlignment','top');

% --- live traces ---------------------------------------------------------
axH = axes(fig,'Position',[0.65 0.71 0.32 0.23]); hold(axH,'on');
yline(axH,c.Warn,'--','WARN','HandleVisibility','off');
yline(axH,c.Alarm,'--','ALARM','HandleVisibility','off');
hOvr = plot(axH,nan,nan,'LineWidth',5,'Color',[0 0 0 0.18]);
hMot = plot(axH,nan,nan,'LineWidth',1.6,'Color',[0.00 0.45 0.74]);
hBat = plot(axH,nan,nan,'LineWidth',1.6,'Color',[0.85 0.33 0.10]);
xlim(axH,[0 t(end)]); ylim(axH,[-2 105]);
ylabel(axH,'Health'); title(axH,'Health score','FontSize',10); grid(axH,'on');
legend(axH,[hMot hBat hOvr],{'motor','battery','overall'}, ...
       'Location','southwest','FontSize',7);

axC = axes(fig,'Position',[0.65 0.40 0.32 0.22]); hold(axC,'on');
hSoc = plot(axC,nan,nan,'LineWidth',2.2,'Color',[0.20 0.45 0.75]);
hRth = plot(axC,nan,nan,'--','LineWidth',1.6,'Color',[0.85 0.33 0.10]);
yline(axC,c.Reserve*100,':','reserve','HandleVisibility','off');
xlim(axC,[0 t(end)]); ylim(axC,[0 105]);
ylabel(axC,'Charge [%]'); grid(axC,'on');
title(axC,'Charge left vs charge needed to get home','FontSize',10);
legend(axC,[hSoc hRth],{'remaining','needed'},'Location','southwest','FontSize',7);

axM = axes(fig,'Position',[0.65 0.07 0.32 0.22]); hold(axM,'on');
yyaxis(axM,'left');
hTmp = plot(axM,nan,nan,'LineWidth',1.8); ylabel(axM,'Temp [\circC]');
ylim(axM,[20 max(T_mot)*1.15]);
yyaxis(axM,'right');
hCur = plot(axM,nan,nan,'LineWidth',1.8); ylabel(axM,'Current [A]');
ylim(axM,[min(I_mot)*0.9 max(I_mot)*1.1]);
xlim(axM,[0 t(end)]); xlabel(axM,'Time [s]'); grid(axM,'on');
title(axM,'Motor','FontSize',10);

% Drop the hover toolbars - they serve no purpose during playback and they
% show up in exported stills and video frames.
set(findobj(fig,'Type','axes'),'Toolbar',[]);

%% ---------- optional video ---------------------------------------------
vid = [];
if strlength(opts.Save) > 0
    vid = VideoWriter(char(opts.Save),'MPEG-4');
    vid.FrameRate = 30;
    open(vid);
end

%% ---------- play --------------------------------------------------------
alertCols = [0.20 0.70 0.30; 0.95 0.70 0.10; 0.85 0.20 0.15];
alertName = {'NOMINAL','WARN','ALARM'};
dec = max(1, round(stride/3));              % decimate traces - the eye cannot
                                            % see 6500 points anyway
% Playback is paced against the wall clock. Without this, Speed only changed
% how many samples were skipped and the animation ran at whatever rate the
% graphics happened to manage - which was ~100x real time regardless of what
% you asked for. Pacing is skipped when writing a video, where the frame rate
% of the file is what matters, not how long we sat here.
doPace = opts.Pace && isempty(vid);
tWall  = tic;
tZero  = t(frames(1));

for k = frames
    if ~isvalid(fig), break; end
    ia = min(max(round(alert(k)),0),2) + 1;
    ix = 1:dec:k;  if ix(end) ~= k, ix(end+1) = k; end %#ok<AGROW>

    % scene
    [gx, gy] = droneGlyph(dist(k), alt(k), spd(k), xMax, yMax);
    set(hDrone,'XData',gx,'YData',gy,'Color',alertCols(ia,:));
    set(hTrail,'XData',dist(ix),'YData',alt(ix));
    dr = min(dRet(k), xMax);
    set(hRetLn,'XData',[dr dr],'YData',[0 yMax]);
    set(hNoGo, 'XData',[dr xMax xMax dr],'YData',[0 0 yMax yMax]);

    % FPV - put the camera at the drone and point it where it is going
    hdg = sign(spd(k)); if hdg == 0, hdg = 1; end     % +1 outbound, -1 homeward
    camPos = [dist(k), 0, max(alt(k),2)];
    camTgt = camPos + [hdg*60, 0, -7];                % look ahead, nosed down
    set(axFPV,'XLim',dist(k)+[-fpvW fpvW], ...        % cube travels with it
              'CameraPosition',camPos,'CameraTarget',camTgt);
    set(hCamTxt,'String',sprintf('ALT %5.1f m\nGS  %5.1f m/s\nHDG %s', ...
                                 alt(k), abs(spd(k)), ternary(hdg>0,'OUT','HOME')));

    % status
    set(hLamp,'FaceColor',alertCols(ia,:));
    set(hLampTxt,'String',alertName{ia});
    set(hStat,'String',sprintf([ ...
        '  time      %5.0f s\n  distance  %5.0f m\n  altitude  %5.1f m\n' ...
        '  speed     %5.1f m/s\n\n  charge    %5.0f %%\n  need home %5.0f %%\n' ...
        '  can reach %5.0f m\n\n  health    %5.0f\n  motor     %5.0f\n' ...
        '  battery   %5.0f\n\n  phase     %s'], ...
        t(k), dist(k), alt(k), spd(k), 100*soc(k), 100*min(need(k),9.99), ...
        min(dRet(k),99999), ovr(k), mot_h(k), bat_h(k), phaseName(state(k))));

    % traces
    set(hOvr,'XData',t(ix),'YData',ovr(ix));
    set(hMot,'XData',t(ix),'YData',mot_h(ix));
    set(hBat,'XData',t(ix),'YData',bat_h(ix));
    set(hSoc,'XData',t(ix),'YData',soc(ix)*100);
    set(hRth,'XData',t(ix),'YData',min(rth(ix),1.05)*100);
    set(hTmp,'XData',t(ix),'YData',T_mot(ix));
    set(hCur,'XData',t(ix),'YData',I_mot(ix));

    if doPace
        lag = (t(k) - tZero)/opts.Speed - toc(tWall);
        if lag > 0.002
            pause(lag);         % pause() also flushes the draw queue
        else
            drawnow limitrate   % behind schedule - just render and move on
        end
    else
        drawnow limitrate
    end
    if ~isempty(vid) && isvalid(fig), writeVideo(vid, getframe(fig)); end
end

if ~isempty(vid)
    close(vid);
    fprintf('video written to %s\n', opts.Save);
end
if isvalid(fig), drawnow; end
end


function [V, F, C] = cityBlocks(xMax)
%CITYBLOCKS  Buildings lining the flight corridor, as one merged patch.
%
%   Every building is a box, and all of them are concatenated into a single
%   vertex/face list. That matters: one patch object means each animation
%   frame only moves the camera, instead of MATLAB re-sorting hundreds of
%   separate objects.
%
%   The drone flies along y = 0, and nothing is placed within 22 m of that
%   line, so it flies down a street rather than through a wall. There is no
%   collision model - this is scenery.

rng(11);                                    % same city every run
gapY   = 22;                                % clear corridor half-width [m]
rows   = [1 2 3];                           % how many blocks deep each side
pitch  = 55;                                % spacing along the route [m]
xs     = -200:pitch:(xMax+200);

V = zeros(0,3);  F = zeros(0,4);  C = zeros(0,3);
for x0 = xs
    for sideSign = [-1 1]
        for r = rows
            if rand < 0.18, continue; end   % gaps, so it is not a solid wall
            w  = 22 + rand*16;              % footprint along x
            dp = 20 + rand*18;              % footprint across y
            h  = 12 + rand*72;              % height - some above cruise height
            y0 = sideSign * (gapY + (r-1)*38 + rand*6);
            cx = x0 + rand*12;
            [v, f] = boxMesh(cx, y0 + sideSign*dp/2, w, dp, h);
            shade = 0.38 + 0.30*rand;
            col   = [shade, shade*0.99, shade*1.06];   % cool concrete greys
            F = [F; f + size(V,1)];         %#ok<AGROW>
            V = [V; v];                     %#ok<AGROW>
            C = [C; repmat(col, size(f,1), 1)];  %#ok<AGROW>
        end
    end
end
end


function [v, f] = boxMesh(cx, cy, w, d, h)
%BOXMESH  Eight vertices and six quad faces for one building.
x1 = cx-w/2;  x2 = cx+w/2;
y1 = cy-d/2;  y2 = cy+d/2;
v = [x1 y1 0; x2 y1 0; x2 y2 0; x1 y2 0; ...
     x1 y1 h; x2 y1 h; x2 y2 h; x1 y2 h];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end


function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end


function [gx, gy] = droneGlyph(x, y, speed, xMax, yMax)
%DRONEGLYPH  Points for a small side-on quadrotor at (x,y).
%   Separate x and y scales because the scene axes are ~100x wider than tall -
%   a shape built in raw data units would be invisible.
sx = xMax*0.026;
sy = yMax*0.045;
lean = -sign(speed)*0.45;                      % tilt into the direction of travel
lx = [-1.0 1.0 NaN -1.0 -1.0+lean NaN -1.6+lean -0.4+lean NaN ...
       1.0 1.0+lean NaN 0.4+lean 1.6+lean];
ly = [ 0.0 0.0 NaN  0.0  0.7      NaN  0.7       0.7      NaN ...
       0.0 0.7      NaN 0.7       0.7];
gx = x + lx*sx;
gy = y + ly*sy;
end


function s = phaseName(state)
switch state
    case 0,    s = 'OUTBOUND';
    case 1,    s = 'RETURNING';
    otherwise, s = 'HOME';
end
end

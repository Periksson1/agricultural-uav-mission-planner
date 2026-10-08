%% visualizeDubinsFieldTransfer
% Create a report figure showing how the planner connects one field to the
% next using a climb, an elevated Dubins transfer, descent, and run-in.
%
% This is a compact standalone visualisation of the same connection idea
% used by buildInterFieldTransferChunk in runMissionPlanning.m.

setupProject;
clc
close all

%% -------------------- USER SETTINGS --------------------
uavAirSpeed = 25.8;          % [m/s]
maxRollAngle = deg2rad(30);  % [rad]
flightHeight_m = 4.0;        % [m]
transferAltitude_m = 15.0;   % [m]
climbAngle_deg = 5.0;        % [deg]
descentAngle_deg = 5.0;      % [deg]
preDescentLevelDistance_m = 0.0;
preSprayLevelDistance_m = 75.0;
dsLine = 1.0;
dsTurn = 1.0;
cruisePowerW = 5447;
verticalExaggeration = 10.0;
exportFigure = true;
outputFile = fullfile(projectRoot(), 'outputs', 'dubinsFieldTransferVisual.png');

% Simple field outlines for visual context.
fieldA = rotateRectangle([165, 20], 520, 240, 0);
fieldBCentreXY = [735, 315];
fieldBLength_m = 540;
fieldBWidth_m = 250;
fieldBRotation_deg = -24;
fieldB = rotateRectangle(fieldBCentreXY, fieldBLength_m, fieldBWidth_m, fieldBRotation_deg);

% Last spray lane in the current field. The lane is placed along the side
% of the field and the transfer begins exactly at the lane end.
currentLaneStartXYZ = [-95, 110, flightHeight_m]; % [East North Up]
currentFieldEndXYZ = [425, 110, flightHeight_m];  % [East North Up]
currentHeading = deg2rad(90);                     % heading clockwise from North

% First spray lane in the next field. The transfer rejoins at the lane start
% on the field boundary, then the lane continues along the field's longest
% side.
fieldBEdgeOffset_m = 0.42 * fieldBWidth_m;
nextLaneStartXY = localPointOnRotatedRectangle( ...
    [-fieldBLength_m/2, fieldBEdgeOffset_m], fieldBCentreXY, fieldBRotation_deg);
nextLaneEndXY = localPointOnRotatedRectangle( ...
    [fieldBLength_m/2, fieldBEdgeOffset_m], fieldBCentreXY, fieldBRotation_deg);
nextFieldStartXYZ = [nextLaneStartXY, flightHeight_m]; % [East North Up]
nextLaneEndXYZ = [nextLaneEndXY, flightHeight_m];      % [East North Up]
nextFieldHeading = headingFromTwoPoints(nextFieldStartXYZ, nextLaneEndXYZ);

%% -------------------- BUILD TRANSFER GEOMETRY --------------------
codes = missionCodes();

uavConn = uavDubinsConnection;
uavConn.AirSpeed = uavAirSpeed;
uavConn.MaxRollAngle = maxRollAngle;
actualTurnRadius_m = uavConn.MinTurningRadius;

altDelta_m = max(transferAltitude_m - flightHeight_m, 0);
climbDistance_m = safeClimbDistance(altDelta_m, climbAngle_deg);
descentDistance_m = safeClimbDistance(altDelta_m, descentAngle_deg);

climbEndXYZ = offsetPointAlongHeading(currentFieldEndXYZ, currentHeading, climbDistance_m);
climbEndXYZ(3) = transferAltitude_m;

runInStartXYZ = offsetPointAlongHeading(nextFieldStartXYZ, nextFieldHeading, -preSprayLevelDistance_m);
runInStartXYZ(3) = flightHeight_m;

descentStartXYZ = offsetPointAlongHeading(runInStartXYZ, nextFieldHeading, -descentDistance_m);
descentStartXYZ(3) = transferAltitude_m;

preDescentStartXYZ = offsetPointAlongHeading(descentStartXYZ, nextFieldHeading, -preDescentLevelDistance_m);
preDescentStartXYZ(3) = transferAltitude_m;

[climbPts, ~, ~, climbSegTimes] = buildStraightChunk( ...
    currentFieldEndXYZ, climbEndXYZ, dsLine, uavAirSpeed, ...
    codes.mode.transfer, codes.stage.transferCruise, NaN, cruisePowerW);

[dubinsPts, ~, ~, dubinsSegTimes] = buildDubinsChunk( ...
    uavConn, climbEndXYZ, currentHeading, preDescentStartXYZ, nextFieldHeading, ...
    dsTurn, codes.mode.transfer, codes.stage.transferCruise, uavAirSpeed, NaN, cruisePowerW);

[levelPts, ~, ~, levelSegTimes] = buildStraightChunk( ...
    preDescentStartXYZ, descentStartXYZ, dsLine, uavAirSpeed, ...
    codes.mode.transfer, codes.stage.transferCruise, NaN, cruisePowerW);

[descentPts, ~, ~, descentSegTimes] = buildStraightChunk( ...
    descentStartXYZ, runInStartXYZ, dsLine, uavAirSpeed, ...
    codes.mode.transfer, codes.stage.transferRejoin, NaN, cruisePowerW);

[runInPts, ~, ~, runInSegTimes] = buildStraightChunk( ...
    runInStartXYZ, nextFieldStartXYZ, dsLine, uavAirSpeed, ...
    codes.mode.transfer, codes.stage.transferRejoin, NaN, cruisePowerW);

pathXYZ = joinPointBlocks(climbPts, dubinsPts, levelPts, descentPts, runInPts);
segmentTimes_s = [ ...
    sum(climbSegTimes); ...
    sum(dubinsSegTimes); ...
    sum(levelSegTimes); ...
    sum(descentSegTimes); ...
    sum(runInSegTimes)];

%% -------------------- PLOT --------------------
fig = figure('Color', 'w', 'Name', 'Dubins field transfer visual', 'NumberTitle', 'off', ...
    'Position', [80 80 1250 720]);

ax3 = axes('Parent', fig);
hold(ax3, 'on')
grid(ax3, 'on')
axis(ax3, 'equal')
daspect(ax3, [1 1 1/verticalExaggeration])
view(ax3, 38, 24)
set(ax3, 'FontSize', 10)
xlabel(ax3, 'East [m]')
ylabel(ax3, 'North [m]')
zlabel(ax3, 'Up [m]')
title(ax3, 'Dubins connection from one field spray lane to the next')

drawFieldPatch(ax3, fieldA, 'Current field');
drawFieldPatch(ax3, fieldB, 'Next field');

plotSprayLane(ax3, currentLaneStartXYZ, currentFieldEndXYZ, ...
    [0.00 0.48 0.18], 'Last spray lane');
plotChunk3(ax3, climbPts, [0.16 0.45 0.80], 'Climb-out');
plotChunk3(ax3, dubinsPts, [0.90 0.32 0.16], 'Elevated Dubins connection');
plotChunk3(ax3, levelPts, [0.45 0.45 0.45], 'Level section');
plotChunk3(ax3, descentPts, [0.20 0.62 0.34], 'Descent');
plotChunk3(ax3, runInPts, [0.50 0.20 0.68], 'Run-in');
plotSprayLane(ax3, nextFieldStartXYZ, nextLaneEndXYZ, ...
    [0.00 0.48 0.18], 'First spray lane');

plotHeadingArrow(ax3, currentFieldEndXYZ, currentHeading, 38, [0.05 0.20 0.50]);
plotHeadingArrow(ax3, nextFieldStartXYZ, nextFieldHeading, 38, [0.25 0.05 0.35]);

scatter3(ax3, currentFieldEndXYZ(1), currentFieldEndXYZ(2), currentFieldEndXYZ(3), ...
    86, [0.05 0.20 0.50], 'filled', 'DisplayName', 'Spray lane end');
scatter3(ax3, nextFieldStartXYZ(1), nextFieldStartXYZ(2), nextFieldStartXYZ(3), ...
    86, [0.25 0.05 0.35], 'filled', 'DisplayName', 'Next spray lane start');
scatter3(ax3, climbEndXYZ(1), climbEndXYZ(2), climbEndXYZ(3), ...
    60, [0.16 0.45 0.80], 'filled', 'DisplayName', 'Top of climb');
scatter3(ax3, descentStartXYZ(1), descentStartXYZ(2), descentStartXYZ(3), ...
    60, [0.20 0.62 0.34], 'filled', 'DisplayName', 'Descent start');
scatter3(ax3, runInStartXYZ(1), runInStartXYZ(2), runInStartXYZ(3), ...
    60, [0.50 0.20 0.68], 'filled', 'DisplayName', 'Run-in start');

labelPoint(ax3, currentFieldEndXYZ, 'Spray lane end', [0.05 0.20 0.50], [8 -20 7]);
labelPoint(ax3, nextFieldStartXYZ, 'First spray lane start', [0.25 0.05 0.35], [-85 -25 7]);
xlim(ax3, [min([fieldA(:,1); fieldB(:,1); pathXYZ(:,1); currentLaneStartXYZ(1); nextLaneEndXYZ(1)]) - 80, ...
    max([fieldA(:,1); fieldB(:,1); pathXYZ(:,1); currentLaneStartXYZ(1); nextLaneEndXYZ(1)]) + 80])
ylim(ax3, [min([fieldA(:,2); fieldB(:,2); pathXYZ(:,2); currentLaneStartXYZ(2); nextLaneEndXYZ(2)]) - 90, ...
    max([fieldA(:,2); fieldB(:,2); pathXYZ(:,2); currentLaneStartXYZ(2); nextLaneEndXYZ(2)]) + 110])
zlim(ax3, [0, transferAltitude_m + 8])
legend(ax3, 'Location', 'northeastoutside', 'FontSize', 8)

infoText = sprintf(['Airspeed %.1f m/s\nMax roll %.0f deg\nMinimum turn radius %.1f m\n' ...
    'Transfer altitude %.1f m\nVertical exaggeration %.1fx\nTotal transfer time %.1f s'], ...
    uavAirSpeed, rad2deg(maxRollAngle), actualTurnRadius_m, transferAltitude_m, ...
    verticalExaggeration, sum(segmentTimes_s));
annotation(fig, 'textbox', [0.06 0.12 0.24 0.16], 'String', infoText, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'w', 'EdgeColor', [0.3 0.3 0.3], ...
    'Interpreter', 'none', 'FontSize', 9);

fprintf('Dubins field transfer visual generated. Minimum turn radius: %.2f m\n', actualTurnRadius_m);
fprintf('Total transfer path length: %.2f m\n', cumulativeDistance(pathXYZ));
fprintf('Total transfer time: %.2f s\n', sum(segmentTimes_s));

if exportFigure
    exportgraphics(fig, outputFile, 'Resolution', 300);
    fprintf('Saved figure to: %s\n', outputFile);
end

%% -------------------- LOCAL FUNCTIONS --------------------
function distance_m = safeClimbDistance(altDelta_m, climbAngle_deg)
    if altDelta_m <= 0
        distance_m = 0;
    else
        distance_m = altDelta_m / tan(deg2rad(climbAngle_deg));
    end
end

function xy = rotateRectangle(centreXY, length_m, width_m, rotation_deg)
    local = 0.5 * [-length_m -width_m; length_m -width_m; length_m width_m; -length_m width_m];
    c = cosd(rotation_deg);
    s = sind(rotation_deg);
    rot = [c -s; s c];
    xy = local * rot.' + centreXY;
end

function globalXY = localPointOnRotatedRectangle(localXY, centreXY, rotation_deg)
    c = cosd(rotation_deg);
    s = sind(rotation_deg);
    rot = [c -s; s c];
    globalXY = localXY * rot.' + centreXY;
end

function heading = headingFromTwoPoints(startXYZ, endXYZ)
    dEast = endXYZ(1) - startXYZ(1);
    dNorth = endXYZ(2) - startXYZ(2);
    heading = atan2(dEast, dNorth);
end

function drawFieldPatch(ax, boundaryXY, labelText)
    patch(ax, boundaryXY(:,1), boundaryXY(:,2), zeros(size(boundaryXY,1),1), ...
        [0.58 0.86 0.58], 'FaceAlpha', 0.24, 'EdgeColor', [0.05 0.35 0.12], ...
        'LineWidth', 1.6, 'DisplayName', labelText);
    centreXY = mean(boundaryXY, 1);
    text(ax, centreXY(1), centreXY(2), 0.25, labelText, ...
        'FontWeight', 'bold', 'FontSize', 8, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'Color', [0.05 0.25 0.08]);
end

function plotChunk3(ax, pts, colour, displayName)
    if isempty(pts)
        return
    end
    plot3(ax, pts(:,1), pts(:,2), pts(:,3), '-', ...
        'Color', colour, 'LineWidth', 2.6, 'DisplayName', displayName);
end

function plotSprayLane(ax, startXYZ, endXYZ, colour, displayName)
    plot3(ax, [startXYZ(1), endXYZ(1)], [startXYZ(2), endXYZ(2)], [startXYZ(3), endXYZ(3)], ...
        '-', 'Color', colour, 'LineWidth', 4.0, 'DisplayName', displayName);
    quiver3(ax, startXYZ(1), startXYZ(2), startXYZ(3), ...
        endXYZ(1) - startXYZ(1), endXYZ(2) - startXYZ(2), endXYZ(3) - startXYZ(3), ...
        0, 'Color', colour, 'LineWidth', 1.5, 'MaxHeadSize', 0.18, ...
        'HandleVisibility', 'off');
end

function labelPoint(ax, pointXYZ, labelText, colour, offsetXYZ)
    text(ax, pointXYZ(1) + offsetXYZ(1), pointXYZ(2) + offsetXYZ(2), pointXYZ(3) + offsetXYZ(3), ...
        labelText, 'Color', colour, 'FontSize', 8, 'FontWeight', 'bold', ...
        'BackgroundColor', 'w', 'Margin', 2, 'EdgeColor', colour);
end

function plotHeadingArrow(ax, originXYZ, heading, length_m, colour)
    tipXYZ = offsetPointAlongHeading(originXYZ, heading, length_m);
    quiver3(ax, originXYZ(1), originXYZ(2), originXYZ(3), ...
        tipXYZ(1) - originXYZ(1), tipXYZ(2) - originXYZ(2), 0, ...
        0, 'Color', colour, 'LineWidth', 1.8, 'MaxHeadSize', 0.7, ...
        'HandleVisibility', 'off');
end

function pathXYZ = joinPointBlocks(varargin)
    pathXYZ = zeros(0,3);
    for iBlock = 1:nargin
        pts = varargin{iBlock};
        if isempty(pts)
            continue
        end
        if isempty(pathXYZ)
            pathXYZ = pts;
        elseif norm(pathXYZ(end,:) - pts(1,:)) <= 1e-9
            pathXYZ = [pathXYZ; pts(2:end,:)]; %#ok<AGROW>
        else
            pathXYZ = [pathXYZ; pts]; %#ok<AGROW>
        end
    end
end

function totalDistance_m = cumulativeDistance(pts)
    if size(pts,1) < 2
        totalDistance_m = 0;
    else
        totalDistance_m = sum(vecnorm(diff(pts,1,1),2,2));
    end
end

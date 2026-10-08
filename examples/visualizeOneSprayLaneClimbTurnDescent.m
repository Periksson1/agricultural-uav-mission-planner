%% visualizeOneSprayLaneClimbTurnDescent
% Explanatory 3D figure for the transition from the end of one spray lane
% to the start of the next spray lane.
%
% Coordinate system:
%   x = East [m], y = North [m], z = Up [m]
%
% Only the previous-lane end and next spray-start point are marked. The full
% spray lane is intentionally omitted so the climb/turn/descent geometry is
% the focus.

setupProject;
clc
clear
close all

%% Scenario values
sprayLaneReferenceLength_m = 100.0;
sprayLaneWidth_m = 6.0;
sprayHeight_m = 4.0;
turnAltitude_m = 15.0;
laneSpacing_m = 6.0;

uavAirSpeed_mps = 25.8;
maxRollAngle_deg = 30.0;
climbAngle_deg = 5.0;
descentAngle_deg = 5.0;
preSprayLevelDistance_m = 75.0;
preDescentLevelDistance_m = 20.0;
verticalExaggeration = 3.0;

dsLine_m = 1.0;
dsTurn_m = 1.0;

%% Label placement offsets
% Adjust these [East, North, Up_display] offsets to move individual labels
% around the figure without changing the flight-path geometry.
climbLabelOffset_m = [20.0, -34.0, 2.0];
runInLabelOffset_m = [10.0, 26.0, 4.0];

altitudeChange_m = turnAltitude_m - sprayHeight_m;
climbDistance_m = altitudeChange_m / tand(climbAngle_deg);
descentDistance_m = altitudeChange_m / tand(descentAngle_deg);

%% One-lane geometry
targetLaneStartXYZ_m = [-0.5 * sprayLaneReferenceLength_m, 0.0, sprayHeight_m];
targetHeading_rad = pi / 2;      % eastbound

previousLaneEndXYZ_m = [-0.5 * sprayLaneReferenceLength_m, -laneSpacing_m, sprayHeight_m];
previousHeading_rad = 3 * pi / 2; % westbound

climbEndXYZ_m = offsetPointAlongHeading(previousLaneEndXYZ_m, previousHeading_rad, climbDistance_m);
climbEndXYZ_m(3) = turnAltitude_m;

runInStartXYZ_m = offsetPointAlongHeading(targetLaneStartXYZ_m, targetHeading_rad, -preSprayLevelDistance_m);
runInStartXYZ_m(3) = sprayHeight_m;

descentStartXYZ_m = offsetPointAlongHeading(runInStartXYZ_m, targetHeading_rad, -descentDistance_m);
descentStartXYZ_m(3) = turnAltitude_m;

turnEndXYZ_m = offsetPointAlongHeading(descentStartXYZ_m, targetHeading_rad, -preDescentLevelDistance_m);
turnEndXYZ_m(3) = turnAltitude_m;

previousLaneX_m = [previousLaneEndXYZ_m(1) + 28.0, previousLaneEndXYZ_m(1)];
previousLaneY_m = previousLaneEndXYZ_m(2) * [1 1];
previousLaneZ_m = sprayHeight_m * [1 1];

%% Build path chunks
codes = missionCodes();
uavConn = uavDubinsConnection;
uavConn.AirSpeed = uavAirSpeed_mps;
uavConn.MaxRollAngle = deg2rad(maxRollAngle_deg);
turnRadius_m = uavConn.MinTurningRadius;
dummyPowerW = 0.0;
passNumber = 1;

[climbPts, climbRoll, climbGamma] = buildDisplayStraightChunk( ...
    previousLaneEndXYZ_m, climbEndXYZ_m, dsLine_m, uavAirSpeed_mps, ...
    codes.mode.interPassTurn, codes.stage.interPassClimbOut, passNumber, dummyPowerW);

[turnPts, turnRoll, turnGamma] = buildDisplayDubinsChunk( ...
    uavConn, climbEndXYZ_m, previousHeading_rad, turnEndXYZ_m, targetHeading_rad, ...
    dsTurn_m, codes.mode.interPassTurn, codes.stage.interPassElevatedTurn, ...
    uavAirSpeed_mps, passNumber, dummyPowerW);

[topLevelPts, topLevelRoll, topLevelGamma] = buildDisplayStraightChunk( ...
    turnEndXYZ_m, descentStartXYZ_m, dsLine_m, uavAirSpeed_mps, ...
    codes.mode.interPassTurn, codes.stage.interPassElevatedTurn, passNumber, dummyPowerW);

[descentPts, descentRoll, descentGamma] = buildDisplayStraightChunk( ...
    descentStartXYZ_m, runInStartXYZ_m, dsLine_m, uavAirSpeed_mps, ...
    codes.mode.interPassTurn, codes.stage.interPassDescent, passNumber, dummyPowerW);

[runInPts, runInRoll, runInGamma] = buildDisplayStraightChunk( ...
    runInStartXYZ_m, targetLaneStartXYZ_m, dsLine_m, uavAirSpeed_mps, ...
    codes.mode.interPassTurn, codes.stage.interPassRunIn, passNumber, dummyPowerW);

allPts = [climbPts; turnPts(2:end,:); topLevelPts(2:end,:); ...
    descentPts(2:end,:); runInPts(2:end,:)];

previousLaneDisplay_m = exaggerateZ([previousLaneX_m(:), previousLaneY_m(:), previousLaneZ_m(:)], verticalExaggeration);
climbDisplayPts = exaggerateZ(climbPts, verticalExaggeration);
turnDisplayPts = exaggerateZ(turnPts, verticalExaggeration);
topLevelDisplayPts = exaggerateZ(topLevelPts, verticalExaggeration);
descentDisplayPts = exaggerateZ(descentPts, verticalExaggeration);
runInDisplayPts = exaggerateZ(runInPts, verticalExaggeration);
allDisplayPts = exaggerateZ(allPts, verticalExaggeration);

targetLaneStartDisplayXYZ_m = exaggerateZ(targetLaneStartXYZ_m, verticalExaggeration);
previousLaneEndDisplayXYZ_m = exaggerateZ(previousLaneEndXYZ_m, verticalExaggeration);
climbEndDisplayXYZ_m = exaggerateZ(climbEndXYZ_m, verticalExaggeration);
runInStartDisplayXYZ_m = exaggerateZ(runInStartXYZ_m, verticalExaggeration);

%% Figure setup
fig = figure('Color', 'w', ...
    'Name', 'One Spray Lane Climb-Turn-Descent Geometry', ...
    'NumberTitle', 'off');
fig.Position(3:4) = [1550 860];

ax = axes('Parent', fig, 'Position', [0.065 0.11 0.72 0.80]);
hold(ax, 'on')
grid(ax, 'on')
axis(ax, 'equal')
view(ax, 88, 9)
camproj(ax, 'orthographic')
set(ax, 'FontName', 'Arial', 'FontSize', 11, ...
    'LineWidth', 0.9, 'GridColor', [0.78 0.78 0.78], ...
    'GridAlpha', 0.34, 'Box', 'on')
xlabel(ax, 'East [m]', 'FontName', 'Arial')
ylabel(ax, 'North [m]', 'FontName', 'Arial')
zlabel(ax, sprintf('Up [m] x %.1f visual scale', verticalExaggeration), 'FontName', 'Arial')
title(ax, 'Transition from previous lane end to next spray start', ...
    'FontName', 'Arial', 'FontSize', 13, 'FontWeight', 'bold')

%% Colours
fieldColor = [0.86 0.91 0.80];
previousColor = [0.52 0.52 0.52];
climbColor = [0.76 0.38 0.12];
turnColor = [0.42 0.22 0.62];
topLevelColor = [0.22 0.48 0.76];
descentColor = [0.05 0.24 0.56];
runInColor = [0.12 0.55 0.48];
sprayColor = [0.05 0.42 0.18];
markerColor = [0.95 0.72 0.18];

%% Plot limits and spray-lane edge markers
xMinField_m = min(allDisplayPts(:,1)) - 30.0;
xMaxField_m = max(targetLaneStartDisplayXYZ_m(1), max(allDisplayPts(:,1))) + 50.0;
yMinField_m = min(allDisplayPts(:,2)) - 55.0;
yMaxField_m = max(allDisplayPts(:,2)) + 55.0;

patch(ax, [xMinField_m xMaxField_m xMaxField_m xMinField_m], ...
    [yMinField_m yMinField_m yMaxField_m yMaxField_m], zeros(1,4), ...
    fieldColor, 'EdgeColor', [0.62 0.69 0.54], 'LineWidth', 0.9, ...
    'FaceAlpha', 0.52, 'DisplayName', 'Field reference plane');

sprayLaneEdgeDisplayXYZ_m = exaggerateZ([ ...
    targetLaneStartXYZ_m(1), -0.5 * sprayLaneWidth_m, sprayHeight_m; ...
    targetLaneStartXYZ_m(1),  0.5 * sprayLaneWidth_m, sprayHeight_m], verticalExaggeration);

%% Path segments
plot3(ax, previousLaneDisplay_m(:,1), previousLaneDisplay_m(:,2), previousLaneDisplay_m(:,3), '-', ...
    'Color', previousColor, 'LineWidth', 2.0, ...
    'DisplayName', 'Previous lane exit');
plot3(ax, climbDisplayPts(:,1), climbDisplayPts(:,2), climbDisplayPts(:,3), '-', ...
    'Color', climbColor, 'LineWidth', 3.0, ...
    'DisplayName', 'Climb-out');
plot3(ax, turnDisplayPts(:,1), turnDisplayPts(:,2), turnDisplayPts(:,3), '-', ...
    'Color', turnColor, 'LineWidth', 3.2, ...
    'DisplayName', 'Elevated turn');
plot3(ax, topLevelDisplayPts(:,1), topLevelDisplayPts(:,2), topLevelDisplayPts(:,3), '-', ...
    'Color', topLevelColor, 'LineWidth', 2.8, ...
    'DisplayName', 'Top level before descent');
plot3(ax, descentDisplayPts(:,1), descentDisplayPts(:,2), descentDisplayPts(:,3), '-', ...
    'Color', descentColor, 'LineWidth', 3.0, ...
    'DisplayName', 'Descent');
plot3(ax, runInDisplayPts(:,1), runInDisplayPts(:,2), runInDisplayPts(:,3), '-', ...
    'Color', runInColor, 'LineWidth', 2.8, ...
    'DisplayName', 'Level run-in');

plot3(ax, turnDisplayPts(:,1), turnDisplayPts(:,2), zeros(size(turnDisplayPts,1),1), ':', ...
    'Color', [0.34 0.30 0.38], 'LineWidth', 1.1, ...
    'HandleVisibility', 'off');
for iGuide = 1:12:size(turnDisplayPts,1)
    plot3(ax, turnDisplayPts(iGuide,1) * [1 1], turnDisplayPts(iGuide,2) * [1 1], ...
        [0 turnDisplayPts(iGuide,3)], ':', 'Color', [0.72 0.72 0.72], ...
        'LineWidth', 0.7, 'HandleVisibility', 'off');
end

[radiusPathDisplayXYZ_m, radiusCentreDisplayXYZ_m, radiusLabelDisplayXYZ_m] = ...
    buildTurnRadiusIndicator(turnDisplayPts, turnRadius_m);
plot3(ax, [radiusCentreDisplayXYZ_m(1), radiusPathDisplayXYZ_m(1)], ...
    [radiusCentreDisplayXYZ_m(2), radiusPathDisplayXYZ_m(2)], ...
    [radiusCentreDisplayXYZ_m(3), radiusPathDisplayXYZ_m(3)], '-', ...
    'Color', [0.20 0.20 0.20], 'LineWidth', 1.5, ...
    'DisplayName', 'Turn radius');
plot3(ax, radiusCentreDisplayXYZ_m(1), radiusCentreDisplayXYZ_m(2), radiusCentreDisplayXYZ_m(3), 'x', ...
    'MarkerSize', 9, 'Color', [0.20 0.20 0.20], ...
    'LineWidth', 1.4, 'HandleVisibility', 'off');

%% Markers and direction arrows
plot3(ax, previousLaneEndDisplayXYZ_m(1), previousLaneEndDisplayXYZ_m(2), previousLaneEndDisplayXYZ_m(3), 's', ...
    'MarkerSize', 9, 'MarkerFaceColor', [0.80 0.10 0.10], ...
    'MarkerEdgeColor', [0.12 0.12 0.12], 'LineWidth', 1.0, ...
    'DisplayName', 'Previous lane end');
plot3(ax, targetLaneStartDisplayXYZ_m(1), targetLaneStartDisplayXYZ_m(2), targetLaneStartDisplayXYZ_m(3), '^', ...
    'MarkerSize', 9, 'MarkerFaceColor', sprayColor, ...
    'MarkerEdgeColor', [0.12 0.12 0.12], 'LineWidth', 1.0, ...
    'DisplayName', 'Spray starts');
plot3(ax, sprayLaneEdgeDisplayXYZ_m(:,1), sprayLaneEdgeDisplayXYZ_m(:,2), sprayLaneEdgeDisplayXYZ_m(:,3), 's', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'w', ...
    'MarkerEdgeColor', sprayColor, 'LineWidth', 1.2, ...
    'LineStyle', 'none', 'DisplayName', 'Spray-lane edge markers');
plot3(ax, climbEndDisplayXYZ_m(1), climbEndDisplayXYZ_m(2), climbEndDisplayXYZ_m(3), 'o', ...
    'MarkerSize', 8, 'MarkerFaceColor', markerColor, ...
    'MarkerEdgeColor', [0.12 0.12 0.12], 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
plot3(ax, runInStartDisplayXYZ_m(1), runInStartDisplayXYZ_m(2), runInStartDisplayXYZ_m(3), 'd', ...
    'MarkerSize', 8, 'MarkerFaceColor', markerColor, ...
    'MarkerEdgeColor', [0.12 0.12 0.12], 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');

addPathArrow(ax, climbDisplayPts, climbColor, 0.55, 12.0);
addPathArrow(ax, turnDisplayPts, turnColor, 0.45, 13.0);
addPathArrow(ax, descentDisplayPts, descentColor, 0.55, 13.0);
addPathArrow(ax, runInDisplayPts, runInColor, 0.45, 11.0);

%% Direct labels
labelStyle = {'FontName', 'Arial', 'FontSize', 10.6, 'BackgroundColor', 'w', 'Margin', 2.4};
previousLabelXYZ_m = [previousLaneEndDisplayXYZ_m(1) - 48.0, previousLaneEndDisplayXYZ_m(2) - 34.0, previousLaneEndDisplayXYZ_m(3) + 9.0];
sprayStartLabelXYZ_m = [targetLaneStartDisplayXYZ_m(1) + 32.0, targetLaneStartDisplayXYZ_m(2) + 38.0, targetLaneStartDisplayXYZ_m(3) + 11.0];

plot3(ax, [previousLaneEndDisplayXYZ_m(1), previousLabelXYZ_m(1) + 10.0], ...
    [previousLaneEndDisplayXYZ_m(2), previousLabelXYZ_m(2) + 4.0], ...
    [previousLaneEndDisplayXYZ_m(3), previousLabelXYZ_m(3) - 0.8], ':', ...
    'Color', previousColor, 'LineWidth', 1.1, 'HandleVisibility', 'off');
plot3(ax, [targetLaneStartDisplayXYZ_m(1), sprayStartLabelXYZ_m(1) - 8.0], ...
    [targetLaneStartDisplayXYZ_m(2), sprayStartLabelXYZ_m(2) - 5.0], ...
    [targetLaneStartDisplayXYZ_m(3), sprayStartLabelXYZ_m(3) - 0.8], ':', ...
    'Color', sprayColor, 'LineWidth', 1.1, 'HandleVisibility', 'off');

text(ax, previousLabelXYZ_m(1), previousLabelXYZ_m(2), previousLabelXYZ_m(3), ...
    'Previous lane end', labelStyle{:}, 'Color', previousColor, 'HorizontalAlignment', 'right');
climbLabelXYZ_m = mean(climbDisplayPts, 1) + climbLabelOffset_m;
text(ax, climbLabelXYZ_m(1), climbLabelXYZ_m(2), climbLabelXYZ_m(3), ...
    sprintf('Climb at %.0f deg', climbAngle_deg), labelStyle{:}, 'Color', climbColor);
text(ax, mean(turnDisplayPts(:,1)) - 18.0, mean(turnDisplayPts(:,2)) + 18.0, turnAltitude_m * verticalExaggeration + 3.0, ...
    'Elevated turn path', labelStyle{:}, 'Color', turnColor);
text(ax, radiusLabelDisplayXYZ_m(1), radiusLabelDisplayXYZ_m(2), radiusLabelDisplayXYZ_m(3), ...
    sprintf('Turn radius = %.1f m', turnRadius_m), ...
    labelStyle{:}, 'Color', [0.20 0.20 0.20], 'HorizontalAlignment', 'center');
text(ax, mean(topLevelDisplayPts(:,1)) - 12.0, mean(topLevelDisplayPts(:,2)) + 14.0, turnAltitude_m * verticalExaggeration + 3.0, ...
    sprintf('Level at %.0f m altitude', turnAltitude_m), labelStyle{:}, 'Color', topLevelColor);
text(ax, mean(descentDisplayPts(:,1)) + 10.0, mean(descentDisplayPts(:,2)) + 14.0, mean(descentDisplayPts(:,3)) + 3.0, ...
    sprintf('Descent at %.0f deg', descentAngle_deg), labelStyle{:}, 'Color', descentColor);
runInLabelXYZ_m = [mean(runInDisplayPts(:,1:2), 1), sprayHeight_m * verticalExaggeration] + runInLabelOffset_m;
text(ax, runInLabelXYZ_m(1), runInLabelXYZ_m(2), runInLabelXYZ_m(3), ...
    sprintf('Level run-in %.0f m', preSprayLevelDistance_m), ...
    labelStyle{:}, 'Color', runInColor, 'HorizontalAlignment', 'right');
text(ax, sprayStartLabelXYZ_m(1), sprayStartLabelXYZ_m(2), sprayStartLabelXYZ_m(3), ...
    'Spraying starts', labelStyle{:}, 'Color', sprayColor);

%% Compact information panel
turnLength_m = pathLength(turnPts);
climbTime_s = pathLength(climbPts) / uavAirSpeed_mps;
turnTime_s = turnLength_m / uavAirSpeed_mps;
descentTime_s = pathLength(descentPts) / uavAirSpeed_mps;
runInTime_s = pathLength(runInPts) / uavAirSpeed_mps;

infoText = sprintf([ ...
    'Transition setup\n' ...
    'Spray lane width           %.1f m\n' ...
    'Spray height               %.1f m\n' ...
    'Turn altitude              %.1f m\n' ...
    'Min turn radius            %.1f m\n' ...
    'Climb distance             %.1f m\n' ...
    'Descent distance           %.1f m\n' ...
    'Level run-in before spray  %.1f m'], ...
    sprayLaneWidth_m, sprayHeight_m, turnAltitude_m, ...
    turnRadius_m, climbDistance_m, descentDistance_m, preSprayLevelDistance_m);

annotation(fig, 'textbox', [0.80 0.55 0.18 0.31], ...
    'String', infoText, ...
    'FontName', 'Arial', 'FontSize', 10.2, ...
    'BackgroundColor', 'w', ...
    'EdgeColor', [0.28 0.28 0.28], ...
    'LineWidth', 0.8, ...
    'Interpreter', 'none', ...
    'FitBoxToText', 'off');

timeText = sprintf([ ...
    'Segment times at %.1f m/s\n' ...
    'Climb       %.2f s\n' ...
    'Turn        %.2f s\n' ...
    'Descent     %.2f s\n' ...
    'Run-in      %.2f s'], ...
    uavAirSpeed_mps, climbTime_s, turnTime_s, descentTime_s, ...
    runInTime_s);

annotation(fig, 'textbox', [0.80 0.34 0.18 0.17], ...
    'String', timeText, ...
    'FontName', 'Arial', 'FontSize', 10.2, ...
    'BackgroundColor', [0.97 0.98 1.00], ...
    'EdgeColor', [0.58 0.65 0.78], ...
    'LineWidth', 0.8, ...
    'Interpreter', 'none', ...
    'FitBoxToText', 'off');

legend(ax, 'Location', 'northeastoutside', 'FontName', 'Arial', 'FontSize', 8.4)

xlim(ax, [xMinField_m, xMaxField_m])
ylim(ax, [yMinField_m, yMaxField_m])
zlim(ax, [0.0, verticalExaggeration * turnAltitude_m + 12.0])
daspect(ax, [1 1 1])

exportgraphics(fig, 'oneSprayLaneClimbTurnDescentVisual.png', 'Resolution', 300);

fprintf('Generated one-lane climb/turn/descent visualization.\n');
fprintf('Saved figure: oneSprayLaneClimbTurnDescentVisual.png\n');
fprintf('Climb distance: %.2f m, descent distance: %.2f m, turn radius: %.2f m\n', ...
    climbDistance_m, descentDistance_m, turnRadius_m);

%% Local helpers
function [pts, roll, gamma] = buildDisplayStraightChunk(startXYZ, endXYZ, ds, uavAirSpeed, modeCode, stageCode, passNumber, powerW)
    [pts, roll, gamma] = buildStraightChunk(startXYZ, endXYZ, ds, uavAirSpeed, modeCode, stageCode, passNumber, powerW);
end

function [pts, roll, gamma] = buildDisplayDubinsChunk(uavConn, startXYZ, startHeading, goalXYZ, goalHeading, ds, modeCode, stageCode, uavAirSpeed, passNumber, powerW)
    [pts, roll, gamma] = buildDubinsChunk(uavConn, startXYZ, startHeading, goalXYZ, goalHeading, ds, modeCode, stageCode, uavAirSpeed, passNumber, powerW);
end

function len = pathLength(pts)
    if size(pts,1) < 2
        len = 0.0;
    else
        len = sum(vecnorm(diff(pts,1,1),2,2));
    end
end

function ptsOut = exaggerateZ(ptsIn, verticalExaggeration)
    ptsOut = ptsIn;
    ptsOut(:,3) = verticalExaggeration * ptsOut(:,3);
end

function [pathPointXYZ, centreXYZ, labelXYZ] = buildTurnRadiusIndicator(turnPts, turnRadius_m)
    idx = max(3, min(size(turnPts,1) - 2, round(0.32 * size(turnPts,1))));
    pathPointXYZ = turnPts(idx,:);

    tangentXY = turnPts(idx+1,1:2) - turnPts(idx-1,1:2);
    tangentNorm = norm(tangentXY);
    if tangentNorm <= 0
        tangentXY = [1 0];
    else
        tangentXY = tangentXY / tangentNorm;
    end

    normalXY = [-tangentXY(2), tangentXY(1)];
    candidateCentreA = pathPointXYZ(1:2) + turnRadius_m * normalXY;
    candidateCentreB = pathPointXYZ(1:2) - turnRadius_m * normalXY;

    turnCentroidXY = mean(turnPts(:,1:2), 1);
    if norm(candidateCentreA - turnCentroidXY) <= norm(candidateCentreB - turnCentroidXY)
        centreXY = candidateCentreA;
    else
        centreXY = candidateCentreB;
    end

    centreXYZ = [centreXY, pathPointXYZ(3)];
    labelXYZ = 0.5 * (pathPointXYZ + centreXYZ);
    labelXYZ(3) = labelXYZ(3) + 4.0;
end

function addPathArrow(ax, pts, arrowColor, fractionAlongPath, arrowLength)
    if size(pts,1) < 3
        return
    end

    idx = max(1, min(size(pts,1) - 1, round(fractionAlongPath * size(pts,1))));
    direction = pts(idx+1,:) - pts(idx,:);
    directionNorm = norm(direction);
    if directionNorm <= 0
        return
    end
    direction = direction / directionNorm;
    quiver3(ax, pts(idx,1), pts(idx,2), pts(idx,3), ...
        arrowLength * direction(1), arrowLength * direction(2), arrowLength * direction(3), ...
        0.0, 'Color', arrowColor, 'LineWidth', 1.45, ...
        'MaxHeadSize', 0.9, 'HandleVisibility', 'off');
end

%% visualizeRectangularSprayLaneGeneration
% 3D explanatory figure for spray-lane generation in a 30 m x 100 m field.
%
% The figure shows:
%   - the rectangular field footprint
%   - each spray-lane width as a shaded strip
%   - straight centrelines joining each lane start and end point
%   - alternating lane direction across the full field width
%   - green triangle start markers and red square end markers

setupProject;
clc
clear
close all

%% Geometry
fieldWidth_m = 30.0;
fieldLength_m = 100.0;
sprayLaneWidth_m = 6.0;
sprayHeight_m = 4.0;

fieldHalfLength_m = fieldLength_m / 2;
fieldHalfWidth_m = fieldWidth_m / 2;
sprayLaneHalfWidth_m = sprayLaneWidth_m / 2;

laneY_m = (-fieldHalfWidth_m + sprayLaneHalfWidth_m):sprayLaneWidth_m:(fieldHalfWidth_m - sprayLaneHalfWidth_m);
nLanes = numel(laneY_m);

fieldX_m = [-fieldHalfLength_m, fieldHalfLength_m, fieldHalfLength_m, -fieldHalfLength_m];
fieldY_m = [-fieldHalfWidth_m, -fieldHalfWidth_m, fieldHalfWidth_m, fieldHalfWidth_m];
fieldZ_m = zeros(1,4);

laneStartXYZ_m = zeros(nLanes, 3);
laneEndXYZ_m = zeros(nLanes, 3);
laneLineX_m = zeros(nLanes, 2);
laneLineY_m = zeros(nLanes, 2);
laneLineZ_m = sprayHeight_m * ones(nLanes, 2);

for iLane = 1:nLanes
    if mod(iLane, 2) == 1
        xStart_m = -fieldHalfLength_m;
        xEnd_m = fieldHalfLength_m;
    else
        xStart_m = fieldHalfLength_m;
        xEnd_m = -fieldHalfLength_m;
    end

    laneStartXYZ_m(iLane,:) = [xStart_m, laneY_m(iLane), sprayHeight_m];
    laneEndXYZ_m(iLane,:) = [xEnd_m, laneY_m(iLane), sprayHeight_m];
    laneLineX_m(iLane,:) = [xStart_m, xEnd_m];
    laneLineY_m(iLane,:) = laneY_m(iLane) * [1 1];
end

widthMarkerX_m = -fieldHalfLength_m + 5.0;
widthMarkerY_m = [laneY_m(1) - sprayLaneHalfWidth_m, laneY_m(1) + sprayLaneHalfWidth_m];
widthMarkerZ_m = 0.35 * ones(1,2);

%% Figure setup
fig = figure('Color', 'w', ...
    'Name', 'Rectangular Field Spray Lane Generation', ...
    'NumberTitle', 'off');
fig.Position(3:4) = [1500 850];

ax = axes('Parent', fig, 'Position', [0.08 0.12 0.78 0.78]);
hold(ax, 'on')
grid(ax, 'on')
axis(ax, 'equal')
view(ax, 34, 19)
camproj(ax, 'orthographic')
set(ax, 'FontName', 'Arial', 'FontSize', 11, ...
    'LineWidth', 0.9, 'GridColor', [0.78 0.78 0.78], ...
    'GridAlpha', 0.34, 'Box', 'on')

xlabel(ax, 'East [m]', 'FontName', 'Arial')
ylabel(ax, 'North [m]', 'FontName', 'Arial')
zlabel(ax, 'Up [m]', 'FontName', 'Arial')
title(ax, 'Spray-lane generation for a 30 m x 100 m rectangular field', ...
    'FontName', 'Arial', 'FontSize', 13, 'FontWeight', 'bold')

%% Colours
fieldColor = [0.86 0.91 0.80];
fieldEdgeColor = [0.62 0.69 0.54];
laneColor = [0.47 0.70 0.49];
laneEdgeColor = [0.14 0.38 0.17];
centrelineColor = [0.05 0.42 0.18];
laneChangeColor = [0.38 0.38 0.38];
startMarkerColor = [0.10 0.64 0.20];
endMarkerColor = [0.82 0.08 0.08];
widthColor = [0.06 0.23 0.55];

%% Field, lane footprints, and centrelines
hField = patch(ax, fieldX_m, fieldY_m, fieldZ_m, fieldColor, ...
    'EdgeColor', fieldEdgeColor, 'LineWidth', 1.2, ...
    'FaceAlpha', 0.62, 'DisplayName', '30 m x 100 m field');

hLanePatch = gobjects(1,1);
hLaneLine = gobjects(1,1);
for iLane = 1:nLanes
    lanePatchX_m = [-fieldHalfLength_m, fieldHalfLength_m, fieldHalfLength_m, -fieldHalfLength_m];
    lanePatchY_m = [laneY_m(iLane) - sprayLaneHalfWidth_m, laneY_m(iLane) - sprayLaneHalfWidth_m, ...
        laneY_m(iLane) + sprayLaneHalfWidth_m, laneY_m(iLane) + sprayLaneHalfWidth_m];
    lanePatchZ_m = 0.05 * ones(1,4);

    if iLane == 1
        hLanePatch = patch(ax, lanePatchX_m, lanePatchY_m, lanePatchZ_m, laneColor, ...
            'EdgeColor', laneEdgeColor, 'LineWidth', 1.0, ...
            'FaceAlpha', 0.36, 'DisplayName', 'Spray-lane width');
        hLaneLine = plot3(ax, laneLineX_m(iLane,:), laneLineY_m(iLane,:), laneLineZ_m(iLane,:), '-', ...
            'Color', centrelineColor, 'LineWidth', 2.5, ...
            'DisplayName', 'Spray-lane centreline');
    else
        patch(ax, lanePatchX_m, lanePatchY_m, lanePatchZ_m, laneColor, ...
            'EdgeColor', laneEdgeColor, 'LineWidth', 1.0, ...
            'FaceAlpha', 0.36, 'HandleVisibility', 'off');
        plot3(ax, laneLineX_m(iLane,:), laneLineY_m(iLane,:), laneLineZ_m(iLane,:), '-', ...
            'Color', centrelineColor, 'LineWidth', 2.5, ...
            'HandleVisibility', 'off');
    end

    directionVector_m = laneEndXYZ_m(iLane,1:2) - laneStartXYZ_m(iLane,1:2);
    directionVector_m = directionVector_m / norm(directionVector_m);
    quiver3(ax, laneStartXYZ_m(iLane,1) + 0.35 * fieldLength_m * directionVector_m(1), ...
        laneStartXYZ_m(iLane,2), sprayHeight_m, ...
        12.0 * directionVector_m(1), 0.0, 0.0, 0.0, ...
        'Color', centrelineColor, 'LineWidth', 1.4, 'MaxHeadSize', 0.8, ...
        'HandleVisibility', 'off');
end

hLaneChange = plot3(ax, NaN, NaN, NaN, '--', ...
    'Color', laneChangeColor, 'LineWidth', 1.2, ...
    'DisplayName', 'Lane change');
for iLane = 1:(nLanes-1)
    plot3(ax, [laneEndXYZ_m(iLane,1), laneStartXYZ_m(iLane+1,1)], ...
        [laneEndXYZ_m(iLane,2), laneStartXYZ_m(iLane+1,2)], ...
        [sprayHeight_m, sprayHeight_m], '--', ...
        'Color', laneChangeColor, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off');
end

hStart = plot3(ax, laneStartXYZ_m(:,1), laneStartXYZ_m(:,2), laneStartXYZ_m(:,3), '^', ...
    'MarkerSize', 9, 'MarkerFaceColor', startMarkerColor, ...
    'MarkerEdgeColor', [0.12 0.12 0.12], 'LineWidth', 1.0, ...
    'LineStyle', 'none', 'DisplayName', 'Lane start');
hEnd = plot3(ax, laneEndXYZ_m(:,1), laneEndXYZ_m(:,2), laneEndXYZ_m(:,3), 's', ...
    'MarkerSize', 9, 'MarkerFaceColor', endMarkerColor, ...
    'MarkerEdgeColor', [0.12 0.12 0.12], 'LineWidth', 1.0, ...
    'LineStyle', 'none', 'DisplayName', 'Lane end');

%% Spray-width indicator
plot3(ax, widthMarkerX_m * [1 1], widthMarkerY_m, widthMarkerZ_m, '-', ...
    'Color', widthColor, 'LineWidth', 1.8, 'HandleVisibility', 'off');
plot3(ax, widthMarkerX_m, widthMarkerY_m(1), widthMarkerZ_m(1), 'v', ...
    'Color', widthColor, 'MarkerFaceColor', widthColor, ...
    'MarkerSize', 8, 'HandleVisibility', 'off');
plot3(ax, widthMarkerX_m, widthMarkerY_m(2), widthMarkerZ_m(2), '^', ...
    'Color', widthColor, 'MarkerFaceColor', widthColor, ...
    'MarkerSize', 8, 'HandleVisibility', 'off');

%% Direct labels
labelStyle = {'FontName', 'Arial', 'FontSize', 9.0, 'BackgroundColor', 'w', 'Margin', 1.7};
for iLane = 1:nLanes
    startIsLeft = laneStartXYZ_m(iLane,1) < 0;
    endIsLeft = laneEndXYZ_m(iLane,1) < 0;

    if startIsLeft
        startDx_m = -2.8;
        startAlign = 'right';
    else
        startDx_m = 2.8;
        startAlign = 'left';
    end

    if endIsLeft
        endDx_m = -2.8;
        endAlign = 'right';
    else
        endDx_m = 2.8;
        endAlign = 'left';
    end

    text(ax, laneStartXYZ_m(iLane,1) + startDx_m, laneStartXYZ_m(iLane,2), ...
        sprayHeight_m + 0.65, sprintf('Lane %d start', iLane), ...
        labelStyle{:}, 'Color', startMarkerColor, 'HorizontalAlignment', startAlign);
    text(ax, laneEndXYZ_m(iLane,1) + endDx_m, laneEndXYZ_m(iLane,2), ...
        sprayHeight_m + 0.65, sprintf('Lane %d end', iLane), ...
        labelStyle{:}, 'Color', endMarkerColor, 'HorizontalAlignment', endAlign);
end

text(ax, widthMarkerX_m - 2.0, mean(widthMarkerY_m), 0.55, ...
    sprintf('Spray lane width = %.1f m', sprayLaneWidth_m), ...
    'FontName', 'Arial', 'FontSize', 10.5, 'BackgroundColor', 'w', ...
    'Margin', 2.0, 'Color', widthColor, 'HorizontalAlignment', 'right');
text(ax, -18.0, max(laneY_m) + sprayLaneHalfWidth_m + 2.2, 0.5, ...
    'Five adjacent passes cover the full field width', ...
    'FontName', 'Arial', 'FontSize', 10.5, 'BackgroundColor', 'w', ...
    'Margin', 2.0, 'Color', laneEdgeColor);

%% Axes limits and legend
xlim(ax, [-64 64])
ylim(ax, [-22 22])
zlim(ax, [0 9])
daspect(ax, [1 1 1])

legend(ax, [hField, hLanePatch, hLaneLine, hLaneChange, hStart, hEnd], ...
    'Location', 'northeastoutside', 'FontName', 'Arial', 'FontSize', 10)

fprintf('Generated 30 m x 100 m rectangular spray-lane figure.\n');
fprintf('Spray lane width: %.2f m\n', sprayLaneWidth_m);
fprintf('Number of lanes: %d\n', nLanes);
for iLane = 1:nLanes
    fprintf('Lane %d centreline: start [%.1f, %.1f, %.1f] m, end [%.1f, %.1f, %.1f] m\n', ...
        iLane, laneStartXYZ_m(iLane,:), laneEndXYZ_m(iLane,:));
end

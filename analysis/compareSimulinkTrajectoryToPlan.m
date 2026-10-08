%% compareSimulinkTrajectoryToPlan
% Compare one Simulink sortie trajectory against the planned sortie route.
%
% Expected workflow:
% 1. Run runMission.m so missionCfg,
%    planningData, and simulationData exist in the workspace.
% 2. Run your Simulink sortie model so that either:
%       out.Position.Data / out.Position.Time
%    or:
%       pos / t
%    are available in the workspace.
% 3. Run this script.
%
% This script assumes:
% - each Simulink run represents one sortie only
% - Simulink position is reported in NED [North East Down]
% - Simulink time starts at the start of the selected sortie

%% -------------------- USER SETTINGS --------------------
% Set the sortie to open immediately when the figure is created.
if ~exist('selectedSortie', 'var') || isempty(selectedSortie)
    selectedSortie = 1;
end
actualTimeOffset_s = 0.0;
actualPositionFrame = 'NED';
% 0 m keeps the ABNED leg switching driven by finish-line crossing only.
crossTrackWaypointRadius_m = 0.0;

%% -------------------- LOAD REQUIRED DATA --------------------
codes = missionCodes();

assert(exist('missionCfg', 'var') == 1, ...
    'missionCfg was not found. Run runMission.m first.');
assert(exist('planningData', 'var') == 1, ...
    'planningData was not found. Run runMission.m first.');
assert(exist('simulationData', 'var') == 1, ...
    'simulationData was not found. Run runMission.m first.');

validateMissionCfg(missionCfg);
validatePlanningData(planningData);
validateSimulationData(simulationData);

if exist('out', 'var') == 1 && isprop(out, 'Position')
    actualPosRaw = out.Position.Data;
    actualTime_s = out.Position.Time;
elseif exist('pos', 'var') == 1 && exist('t', 'var') == 1
    actualPosRaw = pos;
    actualTime_s = t;
else
    error(['Simulink trajectory not found. Expected either out.Position.Data / out.Position.Time ' ...
           'or workspace variables pos and t.']);
end

assert(size(actualPosRaw,2) >= 3, 'Simulink position data must have at least 3 columns.');
actualTime_s = actualTime_s(:) - actualTimeOffset_s;
actualXYZ = convertActualPositionToENU(actualPosRaw(:,1:3), actualPositionFrame);

validActual = all(isfinite(actualXYZ), 2) & isfinite(actualTime_s);
actualXYZ = actualXYZ(validActual,:);
actualTime_s = actualTime_s(validActual);

sortieList = unique(simulationData.segmentSortie(:));
sortieList = sortieList(isfinite(sortieList) & sortieList >= 1);
sortieList = sortieList(:).';
assert(~isempty(sortieList), 'No sortie information was found in simulationData.');

selectedSortieIdx = find(sortieList == selectedSortie, 1, 'first');
assert(~isempty(selectedSortieIdx), ...
    'selectedSortie %d was not found. Available sorties are: %s', ...
    selectedSortie, mat2str(sortieList));

%% -------------------- FIGURE --------------------
fig = figure('Color', 'w', ...
    'Name', 'Planned vs Simulink Sortie Comparison', 'NumberTitle', 'off');

axRoute = axes('Parent', fig);
set(axRoute, 'Position', [0.08 0.20 0.68 0.69]);
hold(axRoute, 'on')
grid(axRoute, 'on')
axis(axRoute, 'equal')
daspect(axRoute, [1 1 1/max(missionCfg.verticalExaggeration, eps)])
xlabel(axRoute, 'East [m]')
ylabel(axRoute, 'North [m]')
zlabel(axRoute, 'Up [m]')
title(axRoute, 'Planned vs actual sortie trajectory')

hStats = annotation(fig, 'textbox', [0.79 0.12 0.19 0.76], ...
    'String', '', 'FitBoxToText', 'off', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'FontSize', 10, 'Interpreter', 'none');

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.31 0.145 0.10 0.03], 'String', 'Sortie', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'left', ...
    'FontSize', 10, 'FontWeight', 'bold');

hSortiePopup = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.39 0.143 0.18 0.04], ...
    'String', arrayfun(@(s) sprintf('Sortie %d', s), sortieList, 'UniformOutput', false), ...
    'Value', selectedSortieIdx, ...
    'BackgroundColor', 'w', 'FontSize', 10);

hViewToggle = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.59 0.143 0.12 0.04], 'String', 'Switch to 2D', ...
    'BackgroundColor', 'w', 'FontSize', 10, 'FontWeight', 'bold');

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.31 0.095 0.08 0.025], 'String', '0 s', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'left', ...
    'FontSize', 9);

hTimeEndLabel = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.66 0.095 0.12 0.025], 'String', '', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'right', ...
    'FontSize', 9);

hTimeSlider = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.31 0.065 0.37 0.035], 'Min', 0, 'Max', 1, 'Value', 1, ...
    'SliderStep', [0.01 0.10], 'Enable', 'off');

hTimeReadout = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.31 0.035 0.47 0.025], 'String', '', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'left', ...
    'FontSize', 9.5, 'FontWeight', 'bold');

fieldFaceColor = [0.60 0.90 0.60];
groundStationFaceColor = [1.00 0.83 0.25];
plannedSortieColor = [0.20 0.40 0.95];
plannedSprayColor = [0.10 0.70 0.10];
actualSortieColor = [0.90 0.20 0.20];

hFieldLegend = patch(axRoute, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
    'FaceColor', fieldFaceColor, 'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.22, ...
    'DisplayName', 'Fields');
hGroundStation = fill3(axRoute, planningData.groundStationX, planningData.groundStationY, planningData.groundStationZ, ...
    groundStationFaceColor, 'EdgeColor', [0.60 0.40 0.00], 'LineWidth', 2.0, 'FaceAlpha', 0.80, ...
    'DisplayName', 'Ground station');
hPlannedSortie = plot3(axRoute, NaN, NaN, NaN, '-', 'Color', plannedSortieColor, 'LineWidth', 2.1, ...
    'DisplayName', 'Selected planned sortie');
hPlannedSpray = plot3(axRoute, NaN, NaN, NaN, '-', 'Color', plannedSprayColor, 'LineWidth', 3.0, ...
    'DisplayName', 'Planned spray legs');
hActualSortie = plot3(axRoute, NaN, NaN, NaN, '-', 'Color', actualSortieColor, 'LineWidth', 2.0, ...
    'DisplayName', 'Simulink sortie');
hPlannedStart = plot3(axRoute, NaN, NaN, NaN, 'o', 'Color', plannedSortieColor, 'MarkerFaceColor', plannedSortieColor, ...
    'MarkerSize', 8, 'DisplayName', 'Planned start');
hActualStart = plot3(axRoute, NaN, NaN, NaN, 'o', 'Color', actualSortieColor, 'MarkerFaceColor', actualSortieColor, ...
    'MarkerSize', 8, 'DisplayName', 'Actual start');
hPlannedEnd = plot3(axRoute, NaN, NaN, NaN, 's', 'Color', plannedSortieColor, 'MarkerFaceColor', 'w', ...
    'MarkerSize', 7, 'DisplayName', 'Planned end');
hActualEnd = plot3(axRoute, NaN, NaN, NaN, 's', 'Color', actualSortieColor, 'MarkerFaceColor', 'w', ...
    'MarkerSize', 7, 'DisplayName', 'Actual current');

drawMissionContext(axRoute, planningData, fieldFaceColor);
text(axRoute, planningData.groundStationCentreXY(1), planningData.groundStationCentreXY(2), 0, ...
    '  Ground station', 'FontSize', 10, 'FontWeight', 'bold', ...
    'Color', [0.50 0.30 0.00], 'HitTest', 'off');

legend(axRoute, ...
    [hFieldLegend, hGroundStation, hPlannedSortie, hPlannedSpray, ...
     hActualSortie, hPlannedStart, hActualStart, hPlannedEnd, hActualEnd], ...
    'Location', 'northeast');

setappdata(fig, 'axRoute', axRoute);
setappdata(fig, 'hPlannedSortie', hPlannedSortie);
setappdata(fig, 'hPlannedSpray', hPlannedSpray);
setappdata(fig, 'hActualSortie', hActualSortie);
setappdata(fig, 'hPlannedStart', hPlannedStart);
setappdata(fig, 'hActualStart', hActualStart);
setappdata(fig, 'hPlannedEnd', hPlannedEnd);
setappdata(fig, 'hActualEnd', hActualEnd);
setappdata(fig, 'hStats', hStats);
setappdata(fig, 'hViewToggle', hViewToggle);
setappdata(fig, 'hTimeSlider', hTimeSlider);
setappdata(fig, 'hTimeReadout', hTimeReadout);
setappdata(fig, 'hTimeEndLabel', hTimeEndLabel);
setappdata(fig, 'sortieList', sortieList);
setappdata(fig, 'missionCfg', missionCfg);
setappdata(fig, 'planningData', planningData);
setappdata(fig, 'simulationData', simulationData);
setappdata(fig, 'actualXYZ', actualXYZ);
setappdata(fig, 'actualTime_s', actualTime_s);
setappdata(fig, 'crossTrackWaypointRadius_m', crossTrackWaypointRadius_m);
setappdata(fig, 'viewAzimuth_deg', missionCfg.viewAzimuth_deg);
setappdata(fig, 'viewElevation_deg', missionCfg.viewElevation_deg);
setappdata(fig, 'verticalExaggeration', missionCfg.verticalExaggeration);
setappdata(fig, 'isTopDownView', false);
setappdata(fig, 'currentSortieNumber', selectedSortie);
setappdata(fig, 'currentCompareData', []);
setappdata(fig, 'fullRouteLimits', []);
setappdata(fig, 'topDownFocusLimits', []);

set(hSortiePopup, 'Callback', @(src,~) refreshComparisonFigure(fig, sortieList(get(src, 'Value'))));
hViewToggle.Callback = @(~,~) toggleComparisonView(fig);
hTimeSlider.Callback = @(~,~) updateComparisonTimeFrame(fig);
sliderListener = addlistener(hTimeSlider, 'ContinuousValueChange', @(~,~) updateComparisonTimeFrame(fig));
setappdata(fig, 'sliderListener', sliderListener);
refreshComparisonFigure(fig, selectedSortie);
applyComparisonView(fig);

%% -------------------- LOCAL FUNCTIONS --------------------
function refreshComparisonFigure(figHandle, sortieNumber)
    codesLocal = missionCodes();

    axRouteLocal = getappdata(figHandle, 'axRoute');
    hPlannedSortieLocal = getappdata(figHandle, 'hPlannedSortie');
    hPlannedSprayLocal = getappdata(figHandle, 'hPlannedSpray');
    hActualSortieLocal = getappdata(figHandle, 'hActualSortie');
    hPlannedStartLocal = getappdata(figHandle, 'hPlannedStart');
    hActualStartLocal = getappdata(figHandle, 'hActualStart');
    hPlannedEndLocal = getappdata(figHandle, 'hPlannedEnd');
    hActualEndLocal = getappdata(figHandle, 'hActualEnd');
    hStatsLocal = getappdata(figHandle, 'hStats');
    hTimeSliderLocal = getappdata(figHandle, 'hTimeSlider');
    hTimeReadoutLocal = getappdata(figHandle, 'hTimeReadout');
    hTimeEndLabelLocal = getappdata(figHandle, 'hTimeEndLabel');

    missionCfgLocal = getappdata(figHandle, 'missionCfg');
    planningDataLocal = getappdata(figHandle, 'planningData');
    simulationDataLocal = getappdata(figHandle, 'simulationData');
    actualXYZLocal = getappdata(figHandle, 'actualXYZ');
    actualTimeLocal_s = getappdata(figHandle, 'actualTime_s');
    crossTrackWaypointRadiusLocal_m = getappdata(figHandle, 'crossTrackWaypointRadius_m');
    setappdata(figHandle, 'currentSortieNumber', sortieNumber);

    sortieData = extractSelectedSortieData(simulationDataLocal, sortieNumber, codesLocal);

    overlapMask = actualTimeLocal_s >= sortieData.compareTime_s(1) & ...
        actualTimeLocal_s <= sortieData.compareTime_s(end);
    actualTimeCompare_s = actualTimeLocal_s(overlapMask);
    actualXYZCompare = actualXYZLocal(overlapMask,:);

    if isempty(actualTimeCompare_s)
        set(hPlannedSortieLocal, 'XData', sortieData.compareXYZ(:,1), 'YData', sortieData.compareXYZ(:,2), 'ZData', sortieData.compareXYZ(:,3));
        set(hPlannedSprayLocal, 'XData', sortieData.sprayXYZ(:,1), 'YData', sortieData.sprayXYZ(:,2), 'ZData', sortieData.sprayXYZ(:,3));
        set(hActualSortieLocal, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set(hPlannedStartLocal, 'XData', sortieData.compareXYZ(1,1), 'YData', sortieData.compareXYZ(1,2), 'ZData', sortieData.compareXYZ(1,3));
        set(hPlannedEndLocal, 'XData', sortieData.compareXYZ(end,1), 'YData', sortieData.compareXYZ(end,2), 'ZData', sortieData.compareXYZ(end,3));
        set(hActualStartLocal, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set(hActualEndLocal, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set(hStatsLocal, 'String', sprintf(['Sortie %d\n\nNo overlapping actual samples were found.\n' ...
            'Check the loaded actual trajectory data.'], sortieNumber));
        set(hTimeSliderLocal, 'Min', 0, 'Max', 1, 'Value', 1, 'Enable', 'off');
        set(hTimeReadoutLocal, 'String', 'No actual samples available');
        set(hTimeEndLabelLocal, 'String', '');
        setappdata(figHandle, 'currentCompareData', []);
        setappdata(figHandle, 'topDownFocusLimits', ...
            computeTopDownFocusLimits(planningDataLocal, sortieData, zeros(0,3), false(0,1)));
        updateRouteAxes(axRouteLocal, missionCfgLocal, planningDataLocal, sortieData.compareXYZ, zeros(0,3));
        applyComparisonView(figHandle);
        return
    end

    plannedXYZCompare = interpPlannedXYZ(sortieData.compareTime_s, sortieData.compareXYZ, actualTimeCompare_s);
    deltaXYZ = actualXYZCompare - plannedXYZCompare;
    lineDistance_m = vecnorm(deltaXYZ, 2, 2);
    sampleStages = sampleStageAtTimes(sortieData.compareTime_s, sortieData.compareStages, actualTimeCompare_s);
    sprayMask = sampleStages == codesLocal.stage.sprayLeg;
    crossTrackMetrics = computeSortieCrossTrackMetrics( ...
        actualXYZCompare, actualTimeCompare_s, sortieData.compareXYZ, ...
        sprayMask, crossTrackWaypointRadiusLocal_m);

    meanLineDistanceAll_m = mean(lineDistance_m, 'omitnan');
    meanLineDistanceSpray_m = meanOrNaN(lineDistance_m(sprayMask));

    set(hPlannedSortieLocal, 'XData', sortieData.compareXYZ(:,1), 'YData', sortieData.compareXYZ(:,2), 'ZData', sortieData.compareXYZ(:,3));
    set(hPlannedSprayLocal, 'XData', sortieData.sprayXYZ(:,1), 'YData', sortieData.sprayXYZ(:,2), 'ZData', sortieData.sprayXYZ(:,3));
    set(hActualSortieLocal, 'XData', actualXYZCompare(:,1), 'YData', actualXYZCompare(:,2), 'ZData', actualXYZCompare(:,3));
    set(hPlannedStartLocal, 'XData', sortieData.compareXYZ(1,1), 'YData', sortieData.compareXYZ(1,2), 'ZData', sortieData.compareXYZ(1,3));
    set(hActualStartLocal, 'XData', actualXYZCompare(1,1), 'YData', actualXYZCompare(1,2), 'ZData', actualXYZCompare(1,3));
    set(hPlannedEndLocal, 'XData', sortieData.compareXYZ(end,1), 'YData', sortieData.compareXYZ(end,2), 'ZData', sortieData.compareXYZ(end,3));
    set(hActualEndLocal, 'XData', actualXYZCompare(end,1), 'YData', actualXYZCompare(end,2), 'ZData', actualXYZCompare(end,3));

    updateRouteAxes(axRouteLocal, missionCfgLocal, planningDataLocal, sortieData.compareXYZ, actualXYZCompare);

    compareData = struct();
    compareData.sortieData = sortieData;
    compareData.actualTimeCompare_s = actualTimeCompare_s;
    compareData.actualXYZCompare = actualXYZCompare;
    compareData.plannedXYZCompare = plannedXYZCompare;
    compareData.lineDistance_m = lineDistance_m;
    compareData.sprayMask = sprayMask;
    compareData.crossTrackMetrics = crossTrackMetrics;
    setappdata(figHandle, 'currentCompareData', compareData);
    setappdata(figHandle, 'topDownFocusLimits', ...
        computeTopDownFocusLimits(planningDataLocal, sortieData, actualXYZCompare, sprayMask));

    configureComparisonTimeSlider(figHandle, actualTimeCompare_s);
    updateComparisonTimeFrame(figHandle);
    applyComparisonView(figHandle);

    statsLines = {
        sprintf('Sortie %d', sortieNumber)
        ''
        sprintf('Compared samples: %d', numel(lineDistance_m))
        sprintf('Compared duration: %.2f s', actualTimeCompare_s(end) - actualTimeCompare_s(1))
        ''
        sprintf('Average |cross-track| ABNED (all): %s', formatStatValue(crossTrackMetrics.meanAbsAll_m, '%.3f m'))
        sprintf('Average |cross-track| ABNED (spraying): %s', formatStatValue(crossTrackMetrics.meanAbsSpray_m, '%.3f m'))
        sprintf('Cumulative |cross-track| ABNED (all): %s', formatStatValue(crossTrackMetrics.totalAbsAll_m_s, '%.3f m*s'))
        sprintf('Cumulative |cross-track| ABNED (spraying): %s', formatStatValue(crossTrackMetrics.totalAbsSpray_m_s, '%.3f m*s'))
        ''
        sprintf('Average 3D error (all): %.3f m', meanLineDistanceAll_m)
        sprintf('Average 3D error (spraying): %s', formatStatValue(meanLineDistanceSpray_m, '%.3f m'))
        ''
        sprintf('Max |cross-track| ABNED: %.3f m', max(crossTrackMetrics.crossTrackAbs_m))
        sprintf('Max 3D error: %.3f m', max(lineDistance_m))
        sprintf('Spraying samples used: %d', sum(sprayMask))
        };
    set(hStatsLocal, 'String', strjoin(statsLines, newline));
end

function configureComparisonTimeSlider(figHandle, actualTimeCompare_s)
    hTimeSliderLocal = getappdata(figHandle, 'hTimeSlider');
    hTimeEndLabelLocal = getappdata(figHandle, 'hTimeEndLabel');

    sliderMax = max(actualTimeCompare_s(end), eps);
    if numel(actualTimeCompare_s) <= 1
        sliderStep = [1 1];
    else
        sliderStep = [1/max(numel(actualTimeCompare_s)-1, 1), min(20/max(numel(actualTimeCompare_s)-1, 1), 1)];
    end

    set(hTimeSliderLocal, 'Min', 0, 'Max', sliderMax, 'Value', sliderMax, ...
        'SliderStep', sliderStep, 'Enable', 'on');
    set(hTimeEndLabelLocal, 'String', sprintf('%.1f s', actualTimeCompare_s(end)));
end

function updateComparisonTimeFrame(figHandle)
    compareData = getappdata(figHandle, 'currentCompareData');
    if isempty(compareData)
        return
    end

    hTimeSliderLocal = getappdata(figHandle, 'hTimeSlider');
    hTimeReadoutLocal = getappdata(figHandle, 'hTimeReadout');
    hActualSortieLocal = getappdata(figHandle, 'hActualSortie');
    hActualStartLocal = getappdata(figHandle, 'hActualStart');
    hActualEndLocal = getappdata(figHandle, 'hActualEnd');

    currentTime_s = min(max(get(hTimeSliderLocal, 'Value'), compareData.actualTimeCompare_s(1)), compareData.actualTimeCompare_s(end));
    idxCurrent = find(compareData.actualTimeCompare_s <= currentTime_s + 1e-12, 1, 'last');
    if isempty(idxCurrent)
        idxCurrent = 1;
    end

    actualXYZShown = compareData.actualXYZCompare(1:idxCurrent,:);
    set(hActualSortieLocal, 'XData', actualXYZShown(:,1), 'YData', actualXYZShown(:,2), 'ZData', actualXYZShown(:,3));
    set(hActualStartLocal, 'XData', compareData.actualXYZCompare(1,1), 'YData', compareData.actualXYZCompare(1,2), 'ZData', compareData.actualXYZCompare(1,3));
    set(hActualEndLocal, 'XData', compareData.actualXYZCompare(idxCurrent,1), 'YData', compareData.actualXYZCompare(idxCurrent,2), 'ZData', compareData.actualXYZCompare(idxCurrent,3));

    currentTime_s = compareData.actualTimeCompare_s(idxCurrent);
    set(hTimeReadoutLocal, 'String', sprintf( ...
        'Actual time %.2f s | 3D error %.3f m | |cross-track| %.3f m | Active leg %d', ...
        currentTime_s, ...
        compareData.lineDistance_m(idxCurrent), ...
        compareData.crossTrackMetrics.crossTrackAbs_m(idxCurrent), ...
        compareData.crossTrackMetrics.activeLegIdx(idxCurrent)));
end

function drawMissionContext(axHandle, planningDataIn, fieldFaceColor)
    fieldCountIn = planningDataIn.fieldCount;
    fieldVisitOrderIn = planningDataIn.fieldVisitOrder;
    fieldInfoIn = planningDataIn.fieldInfo;

    for visitIdx = 1:fieldCountIn
        fIdx = fieldVisitOrderIn(visitIdx);
        displayLoops = fieldInfoIn(fIdx).displayBoundaryLoops;

        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy) || size(cxy,1) < 3
                continue
            end
            fill3(axHandle, cxy(:,1), cxy(:,2), zeros(size(cxy,1),1), fieldFaceColor, ...
                'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.22, 'HandleVisibility', 'off');
        end

        text(axHandle, fieldInfoIn(fIdx).centreXY(1), fieldInfoIn(fIdx).centreXY(2), 0, ...
            sprintf('  %s', fieldInfoIn(fIdx).name), 'FontSize', 10, ...
            'FontWeight', 'bold', 'Color', [0 0 0], 'HitTest', 'off');
    end
end

function updateRouteAxes(axHandle, missionCfgIn, planningDataIn, plannedXYZIn, actualXYZIn)
    fieldXY = collectFieldBoundaryXY(planningDataIn);

    xCandidates = [fieldXY(:,1); planningDataIn.groundStationX(:); plannedXYZIn(:,1); actualXYZIn(:,1)];
    yCandidates = [fieldXY(:,2); planningDataIn.groundStationY(:); plannedXYZIn(:,2); actualXYZIn(:,2)];
    zCandidates = [plannedXYZIn(:,3); actualXYZIn(:,3); 0];

    xRange = max(xCandidates) - min(xCandidates);
    yRange = max(yCandidates) - min(yCandidates);
    zRange = max(zCandidates) - min(zCandidates);
    xMargin = max(5, 0.03 * max(xRange, 1));
    yMargin = max(5, 0.03 * max(yRange, 1));
    zMargin = max(1, 0.20 * max(zRange, missionCfgIn.vtolAltitude_m));

    xlim(axHandle, [min(xCandidates) - xMargin, max(xCandidates) + xMargin])
    ylim(axHandle, [min(yCandidates) - yMargin, max(yCandidates) + yMargin])
    zlim(axHandle, [max(0, min(zCandidates) - zMargin), max(zCandidates) + zMargin])

    figHandle = ancestor(axHandle, 'figure');
    if ~isempty(figHandle)
        fullRouteLimits = struct( ...
            'XLim', xlim(axHandle), ...
            'YLim', ylim(axHandle), ...
            'ZLim', zlim(axHandle));
        setappdata(figHandle, 'fullRouteLimits', fullRouteLimits);
    end
end

function toggleComparisonView(figHandle)
    setappdata(figHandle, 'isTopDownView', ~getappdata(figHandle, 'isTopDownView'));
    applyComparisonView(figHandle);
end

function applyComparisonView(figHandle)
    axHandle = getappdata(figHandle, 'axRoute');
    hViewToggleLocal = getappdata(figHandle, 'hViewToggle');
    currentSortieNumber = getappdata(figHandle, 'currentSortieNumber');
    isTopDownView = getappdata(figHandle, 'isTopDownView');

    if isTopDownView
        view(axHandle, 2);
        camproj(axHandle, 'orthographic');
        title(axHandle, sprintf('Planned vs actual sortie trajectory: sortie %d (Top-down 2D)', currentSortieNumber));
        set(hViewToggleLocal, 'String', 'Switch to 3D');
    else
        view(axHandle, getappdata(figHandle, 'viewAzimuth_deg'), getappdata(figHandle, 'viewElevation_deg'));
        camproj(axHandle, 'perspective');
        title(axHandle, sprintf('Planned vs actual sortie trajectory: sortie %d (3D)', currentSortieNumber));
        set(hViewToggleLocal, 'String', 'Switch to 2D');
    end

    daspect(axHandle, [1 1 1/max(getappdata(figHandle, 'verticalExaggeration'), eps)])
    if isTopDownView
        applyTopDownFocusLimits(axHandle, getappdata(figHandle, 'topDownFocusLimits'));
    else
        applyStoredRouteLimits(axHandle, getappdata(figHandle, 'fullRouteLimits'));
    end
end

function applyStoredRouteLimits(axHandle, routeLimits)
    if isempty(routeLimits)
        return
    end

    xlim(axHandle, routeLimits.XLim);
    ylim(axHandle, routeLimits.YLim);
    zlim(axHandle, routeLimits.ZLim);
end

function applyTopDownFocusLimits(axHandle, focusLimits)
    if isempty(focusLimits)
        return
    end

    xlim(axHandle, focusLimits.XLim);
    ylim(axHandle, focusLimits.YLim);
end

function focusLimits = computeTopDownFocusLimits(planningDataIn, sortieData, actualXYZIn, sprayMaskIn)
    plannedFocusXYZ = sortieData.sprayXYZ;
    plannedFocusXYZ = plannedFocusXYZ(all(isfinite(plannedFocusXYZ), 2), :);
    if isempty(plannedFocusXYZ)
        plannedFocusXYZ = sortieData.compareXYZ;
    end

    xBase = plannedFocusXYZ(:,1);
    yBase = plannedFocusXYZ(:,2);
    xRange = max(xBase) - min(xBase);
    yRange = max(yBase) - min(yBase);
    fieldSearchMargin = max(30, 0.25 * max([xRange, yRange, 1]));
    searchXLim = [min(xBase) - fieldSearchMargin, max(xBase) + fieldSearchMargin];
    searchYLim = [min(yBase) - fieldSearchMargin, max(yBase) + fieldSearchMargin];

    fieldXY = collectIntersectingFieldBoundaryXY(planningDataIn, searchXLim, searchYLim);
    if isempty(fieldXY)
        fieldXY = collectFieldBoundaryXY(planningDataIn);
    end

    xCandidates = [xBase; fieldXY(:,1)];
    yCandidates = [yBase; fieldXY(:,2)];

    if ~isempty(actualXYZIn)
        if numel(sprayMaskIn) == size(actualXYZIn, 1)
            actualCandidateXYZ = actualXYZIn(sprayMaskIn, :);
        else
            actualCandidateXYZ = actualXYZIn;
        end

        actualCandidateXYZ = actualCandidateXYZ(all(isfinite(actualCandidateXYZ), 2), :);
        xLimForActual = [min(xCandidates), max(xCandidates)];
        yLimForActual = [min(yCandidates), max(yCandidates)];
        actualMargin = max(50, 0.20 * max([diff(xLimForActual), diff(yLimForActual), 1]));
        actualInFocus = actualCandidateXYZ( ...
            actualCandidateXYZ(:,1) >= xLimForActual(1) - actualMargin & ...
            actualCandidateXYZ(:,1) <= xLimForActual(2) + actualMargin & ...
            actualCandidateXYZ(:,2) >= yLimForActual(1) - actualMargin & ...
            actualCandidateXYZ(:,2) <= yLimForActual(2) + actualMargin, :);

        xCandidates = [xCandidates; actualInFocus(:,1)];
        yCandidates = [yCandidates; actualInFocus(:,2)];
    end

    xRange = max(xCandidates) - min(xCandidates);
    yRange = max(yCandidates) - min(yCandidates);
    focusMargin = max(15, 0.06 * max([xRange, yRange, 1]));
    focusLimits = struct( ...
        'XLim', [min(xCandidates) - focusMargin, max(xCandidates) + focusMargin], ...
        'YLim', [min(yCandidates) - focusMargin, max(yCandidates) + focusMargin]);
end

function fieldXY = collectFieldBoundaryXY(planningDataIn)
    fieldXY = zeros(0,2);
    for iField = 1:planningDataIn.fieldCount
        displayLoops = planningDataIn.fieldInfo(iField).displayBoundaryLoops;
        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy)
                continue
            end
            fieldXY = [fieldXY; cxy(:,1:2)]; %#ok<AGROW>
        end
    end
end

function fieldXY = collectIntersectingFieldBoundaryXY(planningDataIn, xLimIn, yLimIn)
    fieldXY = zeros(0,2);
    for iField = 1:planningDataIn.fieldCount
        displayLoops = planningDataIn.fieldInfo(iField).displayBoundaryLoops;
        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy)
                continue
            end

            loopIntersectsFocus = max(cxy(:,1)) >= xLimIn(1) && min(cxy(:,1)) <= xLimIn(2) && ...
                max(cxy(:,2)) >= yLimIn(1) && min(cxy(:,2)) <= yLimIn(2);
            if loopIntersectsFocus
                fieldXY = [fieldXY; cxy(:,1:2)]; %#ok<AGROW>
            end
        end
    end
end

function sortieData = extractSelectedSortieData(simulationDataIn, sortieNumber, codesIn)
    segmentSortieIn = simulationDataIn.segmentSortie;
    segmentStagesIn = simulationDataIn.segmentStages;
    pathXYZIn = simulationDataIn.pathXYZ;
    pathTimeIn = simulationDataIn.pathTime;

    compareSegMask = segmentSortieIn == sortieNumber & ...
        ~ismember(segmentStagesIn, codesIn.stageGroups.cruiseOnlyExcluded);
    compareSegIdx = find(compareSegMask);
    assert(~isempty(compareSegIdx), ...
        'Sortie %d has no cruise-only path available for comparison.', sortieNumber);

    compareStartIdx = compareSegIdx(1);
    compareEndIdx = compareSegIdx(end);

    sortieData.compareXYZ = pathXYZIn(compareStartIdx:compareEndIdx+1,:);
    sortieData.compareTime_s = pathTimeIn(compareStartIdx:compareEndIdx+1) - pathTimeIn(compareStartIdx);
    sortieData.compareStages = segmentStagesIn(compareStartIdx:compareEndIdx);
    sortieData.sprayXYZ = buildSprayOnlyPolyline(sortieData.compareXYZ, sortieData.compareStages, codesIn.stage.sprayLeg);
    sortieData.cumulativeDistance_m = [0; cumsum(vecnorm(diff(sortieData.compareXYZ,1,1), 2, 2))];
end

function interpXYZ = interpPlannedXYZ(plannedTime_s, plannedXYZ, queryTime_s)
    interpXYZ = [ ...
        interp1(plannedTime_s, plannedXYZ(:,1), queryTime_s, 'linear'), ...
        interp1(plannedTime_s, plannedXYZ(:,2), queryTime_s, 'linear'), ...
        interp1(plannedTime_s, plannedXYZ(:,3), queryTime_s, 'linear')];
end

function stageCode = sampleStageAtTimes(plannedTime_s, segmentStages, queryTime_s)
    nSegments = numel(segmentStages);
    stageCode = nan(size(queryTime_s));
    segIdx = 1;

    for iQuery = 1:numel(queryTime_s)
        while segIdx < nSegments && queryTime_s(iQuery) > plannedTime_s(segIdx + 1)
            segIdx = segIdx + 1;
        end
        stageCode(iQuery) = segmentStages(segIdx);
    end
end

function actualENU = convertActualPositionToENU(posIn, frameName)
    frameName = upper(strtrim(frameName));
    switch frameName
        case 'NED'
            actualENU = [posIn(:,2), posIn(:,1), -posIn(:,3)];
        case 'ENU'
            actualENU = posIn;
        otherwise
            error('Unsupported actualPositionFrame "%s". Use ''NED'' or ''ENU''.', frameName);
    end
end

function polylineXYZ = buildSprayOnlyPolyline(pathXYZIn, segmentStagesIn, sprayStageCode)
    polylineXYZ = nan(0,3);
    for i = 1:numel(segmentStagesIn)
        if segmentStagesIn(i) ~= sprayStageCode
            continue
        end

        if isempty(polylineXYZ)
            polylineXYZ = [pathXYZIn(i,:); pathXYZIn(i+1,:)];
        else
            polylineXYZ = [polylineXYZ; NaN NaN NaN; pathXYZIn(i,:); pathXYZIn(i+1,:)]; %#ok<AGROW>
        end
    end
end

function valueOut = meanOrNaN(valueIn)
    if isempty(valueIn)
        valueOut = NaN;
    else
        valueOut = mean(valueIn, 'omitnan');
    end
end

function outStr = formatStatValue(valueIn, fmt)
    if isnan(valueIn)
        outStr = 'n/a';
    else
        outStr = sprintf(fmt, valueIn);
    end
end

function metrics = computeSortieCrossTrackMetrics(actualXYZIn, actualTimeIn_s, plannedXYZIn, ...
        sprayMaskIn, waypointRadius_m)
    sampleCount = size(actualXYZIn, 1);

    metrics = struct();
    metrics.crossTrackSigned_m = zeros(sampleCount,1);
    metrics.crossTrackAbs_m = zeros(sampleCount,1);
    metrics.crossTrackRate_mps = zeros(sampleCount,1);
    metrics.alongTrackRate_mps = zeros(sampleCount,1);
    metrics.activeLegIdx = zeros(sampleCount,1);
    metrics.distanceToWaypointB_m = zeros(sampleCount,1);
    metrics.closestPointXYZ = zeros(sampleCount,3);
    metrics.cumulativeAbsAll_m_s = zeros(sampleCount,1);
    metrics.cumulativeAbsSpray_m_s = zeros(sampleCount,1);
    metrics.meanAbsAll_m = NaN;
    metrics.meanAbsSpray_m = NaN;
    metrics.totalAbsAll_m_s = NaN;
    metrics.totalAbsSpray_m_s = NaN;

    if sampleCount == 0 || size(plannedXYZIn, 1) < 2
        return
    end

    actualTime_s = actualTimeIn_s(:);
    sprayMask = false(sampleCount, 1);
    nMask = min(numel(sprayMaskIn), sampleCount);
    sprayMask(1:nMask) = logical(sprayMaskIn(1:nMask));

    actualPosNED = convertXYZENUToNED(actualXYZIn);
    plannedWaypointsNED = convertXYZENUToNED(plannedXYZIn);
    actualVelNED = estimateSampleVelocityNED(actualPosNED, actualTime_s);

    activeLegIdx = 1;
    closestPointNED = zeros(sampleCount,3);
    for iSample = 1:sampleCount
        [metrics.crossTrackSigned_m(iSample), ...
            metrics.crossTrackRate_mps(iSample), ...
            metrics.alongTrackRate_mps(iSample), ...
            activeLegIdx, ...
            ~, ...
            metrics.distanceToWaypointB_m(iSample), ...
            closestPointNED(iSample,:)] = ...
            crossTrackErrorABNED(actualPosNED(iSample,:), actualVelNED(iSample,:), ...
            plannedWaypointsNED, activeLegIdx, waypointRadius_m);

        metrics.activeLegIdx(iSample) = activeLegIdx;
    end

    metrics.crossTrackAbs_m = abs(metrics.crossTrackSigned_m);
    metrics.closestPointXYZ = convertNEDToXYZENU(closestPointNED);
    metrics.cumulativeAbsAll_m_s = cumtrapz(actualTime_s, metrics.crossTrackAbs_m);
    metrics.cumulativeAbsSpray_m_s = cumtrapz(actualTime_s, metrics.crossTrackAbs_m .* double(sprayMask));
    metrics.meanAbsAll_m = mean(metrics.crossTrackAbs_m, 'omitnan');
    metrics.meanAbsSpray_m = meanOrNaN(metrics.crossTrackAbs_m(sprayMask));
    metrics.totalAbsAll_m_s = metrics.cumulativeAbsAll_m_s(end);
    metrics.totalAbsSpray_m_s = metrics.cumulativeAbsSpray_m_s(end);
end

function posNED = convertXYZENUToNED(posXYZ)
    posNED = [posXYZ(:,2), posXYZ(:,1), -posXYZ(:,3)];
end

function posXYZ = convertNEDToXYZENU(posNED)
    posXYZ = [posNED(:,2), posNED(:,1), -posNED(:,3)];
end

function velNED = estimateSampleVelocityNED(posNED, time_s)
    sampleCount = size(posNED, 1);
    velNED = zeros(sampleCount, 3);

    if sampleCount <= 1
        return
    end

    for iSample = 1:sampleCount
        if iSample == 1
            idxA = 1;
            idxB = 2;
        elseif iSample == sampleCount
            idxA = sampleCount - 1;
            idxB = sampleCount;
        else
            idxA = iSample - 1;
            idxB = iSample + 1;
        end

        dt = time_s(idxB) - time_s(idxA);
        if isfinite(dt) && abs(dt) > eps
            velNED(iSample,:) = (posNED(idxB,:) - posNED(idxA,:)) / dt;
        end
    end
end

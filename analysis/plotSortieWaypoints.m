%% plotSortieWaypoints
% Plot the selected planned sortie in 3D, together with waypoint markers
% sampled every fixed distance along the sortie path. The waypoint polyline
% is drawn as straight segments between consecutive sampled waypoints.
%
% Expected workflow:
% 1. Run runMission.m so missionCfg,
%    planningData, and simulationData exist in the workspace.
% 2. Run this script.
%
% Default behaviour:
% - samples waypoint markers from the full selected sortie path
% - includes all path segments belonging to that sortie
% - keeps only the bottom/top endpoints of climb and descent sections
% - draws straight 3D line segments between those waypoint markers
% - exports selected-sortie waypoints in NED for Simulink
% - keeps the field outlines and ground station visible

%% -------------------- USER SETTINGS --------------------
if ~exist('selectedSortie', 'var') || isempty(selectedSortie)
    selectedSortie = 1;
end
waypointSpacing_m = 25.0;
removeFinalWaypoint = false;
exportWaypointsToBaseWorkspace = true;

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

assert(isfinite(waypointSpacing_m) && waypointSpacing_m > 0, ...
    'waypointSpacing_m must be a finite value greater than zero.');

sortieList = unique(simulationData.segmentSortie(:));
sortieList = sortieList(isfinite(sortieList) & sortieList >= 1);
sortieList = sortieList(:).';
assert(~isempty(sortieList), 'No sortie information was found in simulationData.');

if isempty(find(sortieList == selectedSortie, 1))
    selectedSortie = sortieList(1);
end

%% -------------------- FIGURE --------------------
fig = figure('Color', 'w', ...
    'Name', 'Sortie Waypoint Polyline', 'NumberTitle', 'off');

axRoute = axes('Parent', fig);
set(axRoute, 'Position', [0.22 0.12 0.74 0.78]);
hold(axRoute, 'on')
grid(axRoute, 'on')
axis(axRoute, 'equal')
daspect(axRoute, [1 1 1/max(missionCfg.verticalExaggeration, eps)])
xlabel(axRoute, 'East [m]')
ylabel(axRoute, 'North [m]')
zlabel(axRoute, 'Up [m]')
title(axRoute, 'Selected sortie waypoint polyline')

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.04 0.86 0.10 0.03], 'String', 'Sortie', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'left', ...
    'FontSize', 10, 'FontWeight', 'bold');

hSortiePopup = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.10 0.858 0.12 0.04], ...
    'String', arrayfun(@(s) sprintf('Sortie %d', s), sortieList, 'UniformOutput', false), ...
    'Value', find(sortieList == selectedSortie, 1, 'first'), ...
    'BackgroundColor', 'w', 'FontSize', 10);

hStats = annotation(fig, 'textbox', [0.03 0.18 0.17 0.62], ...
    'String', '', 'FitBoxToText', 'off', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'FontSize', 10, 'Interpreter', 'none');

fieldFaceColor = [0.60 0.90 0.60];
groundStationFaceColor = [1.00 0.83 0.25];
waypointLineColor = [0.93 0.45 0.12];

hFieldLegend = patch(axRoute, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
    'FaceColor', fieldFaceColor, 'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.22, ...
    'DisplayName', 'Fields');
hGroundStation = fill3(axRoute, planningData.groundStationX, planningData.groundStationY, planningData.groundStationZ, ...
    groundStationFaceColor, 'EdgeColor', [0.60 0.40 0.00], 'LineWidth', 2.0, 'FaceAlpha', 0.80, ...
    'DisplayName', 'Ground station');
hWaypointLine = plot3(axRoute, NaN, NaN, NaN, '-', 'Color', waypointLineColor, 'LineWidth', 2.0, ...
    'DisplayName', 'Waypoint polyline');
hWaypointMarkers = plot3(axRoute, NaN, NaN, NaN, 'o', 'Color', waypointLineColor, ...
    'MarkerFaceColor', 'w', 'MarkerSize', 6.5, 'LineWidth', 1.2, ...
    'DisplayName', 'Waypoint markers');
hSprayStartMarkers = plot3(axRoute, NaN, NaN, NaN, '^', 'Color', [0.00 0.60 0.20], ...
    'MarkerFaceColor', [0.55 0.95 0.55], 'MarkerSize', 12.0, 'LineWidth', 1.8, ...
    'DisplayName', 'Spray lane starts');
hSprayEndMarkers = plot3(axRoute, NaN, NaN, NaN, 's', 'Color', [0.80 0.10 0.10], ...
    'MarkerFaceColor', [1.00 0.75 0.75], 'MarkerSize', 11.5, 'LineWidth', 1.8, ...
    'DisplayName', 'Spray lane ends');
hSprayBoundaryWaypointMarkers = plot3(axRoute, NaN, NaN, NaN, 'o', 'Color', waypointLineColor, ...
    'MarkerFaceColor', 'none', 'MarkerSize', 5.2, 'LineWidth', 1.2, ...
    'HandleVisibility', 'off');

drawMissionContext(axRoute, planningData, fieldFaceColor);
text(axRoute, planningData.groundStationCentreXY(1), planningData.groundStationCentreXY(2), 0, ...
    '  Ground station', 'FontSize', 10, 'FontWeight', 'bold', ...
    'Color', [0.50 0.30 0.00], 'HitTest', 'off');

legend(axRoute, ...
    [hFieldLegend, hGroundStation, hWaypointLine, hWaypointMarkers, hSprayStartMarkers, hSprayEndMarkers], ...
    'Location', 'northeast');

setappdata(fig, 'axRoute', axRoute);
setappdata(fig, 'hStats', hStats);
setappdata(fig, 'missionCfg', missionCfg);
setappdata(fig, 'planningData', planningData);
setappdata(fig, 'simulationData', simulationData);
setappdata(fig, 'sortieList', sortieList);
setappdata(fig, 'waypointSpacing_m', waypointSpacing_m);
setappdata(fig, 'removeFinalWaypoint', removeFinalWaypoint);
setappdata(fig, 'exportWaypointsToBaseWorkspace', exportWaypointsToBaseWorkspace);
setappdata(fig, 'hWaypointLine', hWaypointLine);
setappdata(fig, 'hWaypointMarkers', hWaypointMarkers);
setappdata(fig, 'hSprayStartMarkers', hSprayStartMarkers);
setappdata(fig, 'hSprayEndMarkers', hSprayEndMarkers);
setappdata(fig, 'hSprayBoundaryWaypointMarkers', hSprayBoundaryWaypointMarkers);
setappdata(fig, 'waypointLabels', gobjects(0,1));
setappdata(fig, 'sprayBoundaryLabels', gobjects(0,1));

set(hSortiePopup, 'Callback', @(src,~) refreshSortieWaypointFigure(fig, sortieList(get(src, 'Value')), codes));
refreshSortieWaypointFigure(fig, selectedSortie, codes);

%% -------------------- LOCAL FUNCTIONS --------------------
function refreshSortieWaypointFigure(figHandle, sortieNumber, codesIn)
    axRouteLocal = getappdata(figHandle, 'axRoute');
    hStatsLocal = getappdata(figHandle, 'hStats');
    missionCfgLocal = getappdata(figHandle, 'missionCfg');
    planningDataLocal = getappdata(figHandle, 'planningData');
    simulationDataLocal = getappdata(figHandle, 'simulationData');
    waypointSpacingLocal_m = getappdata(figHandle, 'waypointSpacing_m');
    removeFinalWaypointLocal = getappdata(figHandle, 'removeFinalWaypoint');
    exportWaypointsToBaseWorkspaceLocal = getappdata(figHandle, 'exportWaypointsToBaseWorkspace');

    hWaypointLineLocal = getappdata(figHandle, 'hWaypointLine');
    hWaypointMarkersLocal = getappdata(figHandle, 'hWaypointMarkers');
    hSprayStartMarkersLocal = getappdata(figHandle, 'hSprayStartMarkers');
    hSprayEndMarkersLocal = getappdata(figHandle, 'hSprayEndMarkers');
    hSprayBoundaryWaypointMarkersLocal = getappdata(figHandle, 'hSprayBoundaryWaypointMarkers');

    oldLabels = getappdata(figHandle, 'waypointLabels');
    if ~isempty(oldLabels)
        delete(oldLabels(ishghandle(oldLabels)));
    end
    oldSprayLabels = getappdata(figHandle, 'sprayBoundaryLabels');
    if ~isempty(oldSprayLabels)
        delete(oldSprayLabels(ishghandle(oldSprayLabels)));
    end

    sortieData = extractSelectedSortieWaypointPath(simulationDataLocal, sortieNumber, codesIn);
    waypointData = sampleWaypointsAlongPath(sortieData.xyz, sortieData.time_s, waypointSpacingLocal_m);
    if removeFinalWaypointLocal
        waypointData = removeFinalWaypointFromData(waypointData);
    end
    sprayBoundaryData = buildSprayBoundaryWaypointData(sortieData, codesIn.stage.sprayLeg);
    waypointData = includeSprayBoundaryCheckpoints(waypointData, sprayBoundaryData);
    waypointData.ned = enuToNed(waypointData.xyz);
    sprayBoundaryData.ned = enuToNed(sprayBoundaryData.xyz);

    if exportWaypointsToBaseWorkspaceLocal
        allSortieWaypointData = buildAllSortieWaypointExports(simulationDataLocal, getappdata(figHandle, 'sortieList'), ...
            codesIn, waypointSpacingLocal_m, removeFinalWaypointLocal);
        exportWaypointDataToBaseWorkspace(sortieNumber, waypointData, sprayBoundaryData, ...
            waypointSpacingLocal_m, removeFinalWaypointLocal, allSortieWaypointData);
    end

    set(hWaypointLineLocal, 'XData', waypointData.xyz(:,1), 'YData', waypointData.xyz(:,2), 'ZData', waypointData.xyz(:,3));
    set(hWaypointMarkersLocal, 'XData', waypointData.xyz(:,1), 'YData', waypointData.xyz(:,2), 'ZData', waypointData.xyz(:,3));
    set(hSprayStartMarkersLocal, 'XData', sprayBoundaryData.xyz(sprayBoundaryData.isStart,1), ...
        'YData', sprayBoundaryData.xyz(sprayBoundaryData.isStart,2), ...
        'ZData', sprayBoundaryData.xyz(sprayBoundaryData.isStart,3));
    set(hSprayEndMarkersLocal, 'XData', sprayBoundaryData.xyz(sprayBoundaryData.isEnd,1), ...
        'YData', sprayBoundaryData.xyz(sprayBoundaryData.isEnd,2), ...
        'ZData', sprayBoundaryData.xyz(sprayBoundaryData.isEnd,3));
    set(hSprayBoundaryWaypointMarkersLocal, 'XData', sprayBoundaryData.xyz(:,1), ...
        'YData', sprayBoundaryData.xyz(:,2), ...
        'ZData', sprayBoundaryData.xyz(:,3));

    waypointLabels = gobjects(size(waypointData.xyz,1),1);
    labelOffset = max(1.0, 0.02 * max(waypointData.totalDistance_m, 1));
    for iWaypoint = 1:size(waypointData.xyz,1)
        waypointLabels(iWaypoint) = text(axRouteLocal, ...
            waypointData.xyz(iWaypoint,1), waypointData.xyz(iWaypoint,2), waypointData.xyz(iWaypoint,3) + labelOffset, ...
            sprintf('W%d', iWaypoint), 'FontSize', 8, 'Color', [0.35 0.18 0.05], ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Clipping', 'on');
    end
    setappdata(figHandle, 'waypointLabels', waypointLabels);

    sprayBoundaryLabels = gobjects(size(sprayBoundaryData.xyz,1),1);
    sprayLabelOffset = 1.5 * labelOffset;
    for iBoundary = 1:size(sprayBoundaryData.xyz,1)
        if sprayBoundaryData.isStart(iBoundary)
            labelColor = [0.00 0.45 0.15];
        else
            labelColor = [0.70 0.05 0.05];
        end
        sprayBoundaryLabels(iBoundary) = text(axRouteLocal, ...
            sprayBoundaryData.xyz(iBoundary,1), sprayBoundaryData.xyz(iBoundary,2), ...
            sprayBoundaryData.xyz(iBoundary,3) + sprayLabelOffset, ...
            sprayBoundaryData.labels{iBoundary}, 'FontSize', 8.5, 'Color', labelColor, ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Clipping', 'on');
    end
    setappdata(figHandle, 'sprayBoundaryLabels', sprayBoundaryLabels);

    updateRouteAxes(axRouteLocal, missionCfgLocal, planningDataLocal, sortieData.xyz, waypointData.xyz);
    view(axRouteLocal, missionCfgLocal.viewAzimuth_deg, missionCfgLocal.viewElevation_deg);
    camproj(axRouteLocal, 'perspective');
    daspect(axRouteLocal, [1 1 1/max(missionCfgLocal.verticalExaggeration, eps)])
    title(axRouteLocal, sprintf('Selected sortie waypoint polyline with %.0f m spacing: sortie %d (3D)', ...
        waypointSpacingLocal_m, sortieNumber));

    statsLines = {
        sprintf('Sortie %d', sortieNumber)
        ''
        'Displayed route: waypoint polyline from full sortie path'
        sprintf('Waypoint spacing: %.1f m', waypointSpacingLocal_m)
        'Waypoint markers sampled from full sortie path'
        'Spray lane starts/ends included as waypoints'
        'Climb/descent sampling: endpoints only'
        sprintf('Waypoint count: %d', size(waypointData.xyz,1))
        sprintf('Spray boundary count: %d', size(sprayBoundaryData.xyz,1))
        sprintf('Spray lane count: %d', sprayBoundaryData.laneCount)
        sprintf('Final endpoint removed: %s', logicalToYesNo(removeFinalWaypointLocal))
        sprintf('Exported NED: %s', logicalToYesNo(exportWaypointsToBaseWorkspaceLocal))
        sprintf('Separate spray boundary export: %s', logicalToYesNo(exportWaypointsToBaseWorkspaceLocal))
        sprintf('Sortie samples: %d', size(sortieData.xyz,1))
        ''
        sprintf('Sortie duration: %.2f s', sortieData.time_s(end) - sortieData.time_s(1))
        sprintf('Sortie distance: %.2f m', waypointData.totalDistance_m)
        sprintf('Spray distance: %.2f m', sortieData.sprayDistance_m)
        ''
        sprintf('Start: [%.1f, %.1f, %.1f] m', sortieData.xyz(1,1), sortieData.xyz(1,2), sortieData.xyz(1,3))
        sprintf('End: [%.1f, %.1f, %.1f] m', sortieData.xyz(end,1), sortieData.xyz(end,2), sortieData.xyz(end,3))
        ''
        sprintf('Last waypoint distance: %s', formatLastDistance(waypointData.sampleDistance_m))
        };
    set(hStatsLocal, 'String', strjoin(statsLines, newline));
end

function drawMissionContext(axHandle, planningDataIn, fieldFaceColor)
    for visitIdx = 1:planningDataIn.fieldCount
        fIdx = planningDataIn.fieldVisitOrder(visitIdx);
        displayLoops = planningDataIn.fieldInfo(fIdx).displayBoundaryLoops;

        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy) || size(cxy,1) < 3
                continue
            end
            fill3(axHandle, cxy(:,1), cxy(:,2), zeros(size(cxy,1),1), fieldFaceColor, ...
                'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.22, 'HandleVisibility', 'off');
        end

        text(axHandle, planningDataIn.fieldInfo(fIdx).centreXY(1), ...
            planningDataIn.fieldInfo(fIdx).centreXY(2), 0, ...
            sprintf('  %s', planningDataIn.fieldInfo(fIdx).name), 'FontSize', 10, ...
            'FontWeight', 'bold', 'Color', [0 0 0], 'HitTest', 'off');
    end
end

function updateRouteAxes(axHandle, missionCfgIn, planningDataIn, sortieXYZIn, waypointXYZIn)
    allFieldX = [];
    allFieldY = [];
    for iField = 1:planningDataIn.fieldCount
        displayLoops = planningDataIn.fieldInfo(iField).displayBoundaryLoops;
        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy)
                continue
            end
            allFieldX = [allFieldX; cxy(:,1)]; %#ok<AGROW>
            allFieldY = [allFieldY; cxy(:,2)]; %#ok<AGROW>
        end
    end

    xCandidates = [allFieldX(:); planningDataIn.groundStationX(:); sortieXYZIn(:,1); waypointXYZIn(:,1)];
    yCandidates = [allFieldY(:); planningDataIn.groundStationY(:); sortieXYZIn(:,2); waypointXYZIn(:,2)];
    zCandidates = [sortieXYZIn(:,3); waypointXYZIn(:,3); 0];

    xRange = max(xCandidates) - min(xCandidates);
    yRange = max(yCandidates) - min(yCandidates);
    zRange = max(zCandidates) - min(zCandidates);
    xMargin = max(5, 0.03 * max(xRange, 1));
    yMargin = max(5, 0.03 * max(yRange, 1));
    zMargin = max(1, 0.20 * max(zRange, missionCfgIn.vtolAltitude_m));

    xlim(axHandle, [min(xCandidates) - xMargin, max(xCandidates) + xMargin])
    ylim(axHandle, [min(yCandidates) - yMargin, max(yCandidates) + yMargin])
    zlim(axHandle, [max(0, min(zCandidates) - zMargin), max(zCandidates) + zMargin])
end

function sortieData = extractSelectedSortieWaypointPath(simulationDataIn, sortieNumber, codesIn)
    idxSeg = find(simulationDataIn.segmentSortie == sortieNumber);
    assert(~isempty(idxSeg), 'Sortie %d was not found in simulationData.', sortieNumber);

    firstSeg = idxSeg(1);
    lastSeg = idxSeg(end);

    sortieData.xyz = simulationDataIn.pathXYZ(firstSeg:lastSeg+1,:);
    sortieData.time_s = simulationDataIn.pathTime(firstSeg:lastSeg+1);
    sortieData.time_s = sortieData.time_s - sortieData.time_s(1);
    sortieData.stages = simulationDataIn.segmentStages(firstSeg:lastSeg);
    sortieData.sprayXYZ = buildSprayOnlyPolyline(sortieData.xyz, sortieData.stages, codesIn.stage.sprayLeg);

    if size(sortieData.xyz,1) >= 2
        sortieData.segmentLengths_m = vecnorm(diff(sortieData.xyz,1,1), 2, 2);
        sortieData.cumulativeDistance_m = [0; cumsum(sortieData.segmentLengths_m)];
        sortieData.totalDistance_m = sum(sortieData.segmentLengths_m);
    else
        sortieData.segmentLengths_m = zeros(0,1);
        sortieData.cumulativeDistance_m = 0;
        sortieData.totalDistance_m = 0;
    end

    sprayMask = sortieData.stages == codesIn.stage.sprayLeg;
    if isempty(sortieData.segmentLengths_m)
        sortieData.sprayDistance_m = 0;
    else
        sortieData.sprayDistance_m = sum(sortieData.segmentLengths_m(sprayMask));
    end
end

function sprayBoundaryData = buildSprayBoundaryWaypointData(sortieDataIn, sprayStageCode)
    sprayBoundaryData = struct();
    sprayBoundaryData.xyz = zeros(0,3);
    sprayBoundaryData.time_s = zeros(0,1);
    sprayBoundaryData.sampleDistance_m = zeros(0,1);
    sprayBoundaryData.labels = cell(0,1);
    sprayBoundaryData.isStart = false(0,1);
    sprayBoundaryData.isEnd = false(0,1);
    sprayBoundaryData.laneNumber = zeros(0,1);
    sprayBoundaryData.laneCount = 0;

    if isempty(sortieDataIn.stages) || size(sortieDataIn.xyz,1) < 2
        return
    end

    sprayMask = sortieDataIn.stages == sprayStageCode;
    if ~any(sprayMask)
        return
    end

    laneStartSegIdx = find(sprayMask & [true; ~sprayMask(1:end-1)]);
    laneEndSegIdx = find(sprayMask & [~sprayMask(2:end); true]);
    laneCount = numel(laneStartSegIdx);

    xyz = zeros(2 * laneCount, 3);
    time_s = zeros(2 * laneCount, 1);
    sampleDistance_m = zeros(2 * laneCount, 1);
    labels = cell(2 * laneCount, 1);
    isStart = false(2 * laneCount, 1);
    isEnd = false(2 * laneCount, 1);
    laneNumber = zeros(2 * laneCount, 1);

    for iLane = 1:laneCount
        iStartSeg = laneStartSegIdx(iLane);
        iEndSeg = laneEndSegIdx(iLane);

        startPointIdx = iStartSeg;
        endPointIdx = iEndSeg + 1;

        iRowStart = 2 * iLane - 1;
        iRowEnd = 2 * iLane;

        xyz(iRowStart,:) = sortieDataIn.xyz(startPointIdx,:);
        xyz(iRowEnd,:) = sortieDataIn.xyz(endPointIdx,:);
        time_s(iRowStart) = sortieDataIn.time_s(startPointIdx);
        time_s(iRowEnd) = sortieDataIn.time_s(endPointIdx);
        sampleDistance_m(iRowStart) = sortieDataIn.cumulativeDistance_m(startPointIdx);
        sampleDistance_m(iRowEnd) = sortieDataIn.cumulativeDistance_m(endPointIdx);
        labels{iRowStart} = sprintf('S%d start', iLane);
        labels{iRowEnd} = sprintf('S%d end', iLane);
        isStart(iRowStart) = true;
        isEnd(iRowEnd) = true;
        laneNumber([iRowStart iRowEnd]) = iLane;
    end

    sprayBoundaryData.xyz = xyz;
    sprayBoundaryData.time_s = time_s;
    sprayBoundaryData.sampleDistance_m = sampleDistance_m;
    sprayBoundaryData.labels = labels;
    sprayBoundaryData.isStart = isStart;
    sprayBoundaryData.isEnd = isEnd;
    sprayBoundaryData.laneNumber = laneNumber;
    sprayBoundaryData.laneCount = laneCount;
end

function waypointData = sampleWaypointsAlongPath(pathXYZIn, pathTimeIn, waypointSpacing_mIn)
    nPts = size(pathXYZIn, 1);
    if nPts == 0
        waypointData.xyz = zeros(0,3);
        waypointData.time_s = zeros(0,1);
        waypointData.sampleDistance_m = zeros(0,1);
        waypointData.totalDistance_m = 0;
        return
    elseif nPts == 1
        waypointData.xyz = pathXYZIn;
        waypointData.time_s = pathTimeIn(:);
        waypointData.sampleDistance_m = 0;
        waypointData.totalDistance_m = 0;
        return
    end

    segmentLengths = vecnorm(diff(pathXYZIn,1,1), 2, 2);
    cumulativeDistance = [0; cumsum(segmentLengths)];
    totalDistance_m = cumulativeDistance(end);

    keepIdx = [true; diff(cumulativeDistance) > 1e-9];
    cumulativeUnique = cumulativeDistance(keepIdx);
    xyzUnique = pathXYZIn(keepIdx,:);
    timeUnique = pathTimeIn(keepIdx);

    if totalDistance_m <= 1e-9 || isscalar(cumulativeUnique)
        waypointData.xyz = pathXYZIn(1,:);
        waypointData.time_s = pathTimeIn(1);
        waypointData.sampleDistance_m = 0;
        waypointData.totalDistance_m = totalDistance_m;
        return
    end

    sampleDistance_m = buildWaypointSampleDistances(pathXYZIn, cumulativeDistance, segmentLengths, waypointSpacing_mIn);

    waypointXYZ = zeros(numel(sampleDistance_m), 3);
    waypointTime_s = zeros(numel(sampleDistance_m), 1);
    waypointXYZ(:,1) = interp1(cumulativeUnique, xyzUnique(:,1), sampleDistance_m, 'linear');
    waypointXYZ(:,2) = interp1(cumulativeUnique, xyzUnique(:,2), sampleDistance_m, 'linear');
    waypointXYZ(:,3) = interp1(cumulativeUnique, xyzUnique(:,3), sampleDistance_m, 'linear');
    waypointTime_s(:) = interp1(cumulativeUnique, timeUnique(:), sampleDistance_m, 'linear');

    waypointData.xyz = waypointXYZ;
    waypointData.time_s = waypointTime_s;
    waypointData.sampleDistance_m = sampleDistance_m;
    waypointData.totalDistance_m = totalDistance_m;
end

function sampleDistance_m = buildWaypointSampleDistances(pathXYZIn, cumulativeDistance, segmentLengths, waypointSpacing_mIn)
    altitudeChangeTol_m = 1e-6;
    distanceTol_m = 1e-9;
    nSeg = numel(segmentLengths);
    totalDistance_m = cumulativeDistance(end);
    sampleDistance_m = zeros(0,1);

    iSeg = 1;
    while iSeg <= nSeg
        if segmentLengths(iSeg) <= distanceTol_m
            iSeg = iSeg + 1;
            continue
        end

        runStartSeg = iSeg;
        runVerticalDirection = classifyVerticalDirection(pathXYZIn(iSeg+1,3) - pathXYZIn(iSeg,3), altitudeChangeTol_m);

        while iSeg < nSeg
            nextVerticalDirection = classifyVerticalDirection(pathXYZIn(iSeg+2,3) - pathXYZIn(iSeg+1,3), altitudeChangeTol_m);
            if nextVerticalDirection ~= runVerticalDirection
                break
            end
            iSeg = iSeg + 1;
        end

        runEndSeg = iSeg;
        runStartDistance_m = cumulativeDistance(runStartSeg);
        runEndDistance_m = cumulativeDistance(runEndSeg + 1);

        if runVerticalDirection ~= 0
            sampleDistance_m = [sampleDistance_m; runStartDistance_m; runEndDistance_m]; %#ok<AGROW>
        else
            sampleDistance_m = [sampleDistance_m; buildLevelRunSampleDistances(runStartDistance_m, runEndDistance_m, waypointSpacing_mIn)]; %#ok<AGROW>
        end

        iSeg = iSeg + 1;
    end

    sampleDistance_m = [0; sampleDistance_m; totalDistance_m];
    sampleDistance_m = unique(round(sampleDistance_m / distanceTol_m) * distanceTol_m, 'stable');
    sampleDistance_m = sort(sampleDistance_m);
    sampleDistance_m = sampleDistance_m(sampleDistance_m >= -distanceTol_m & sampleDistance_m <= totalDistance_m + distanceTol_m);
    sampleDistance_m(1) = max(sampleDistance_m(1), 0);
    sampleDistance_m(end) = min(sampleDistance_m(end), totalDistance_m);
end

function verticalDirection = classifyVerticalDirection(deltaUp_m, altitudeChangeTol_m)
    if deltaUp_m > altitudeChangeTol_m
        verticalDirection = 1;
    elseif deltaUp_m < -altitudeChangeTol_m
        verticalDirection = -1;
    else
        verticalDirection = 0;
    end
end

function sampleDistance_m = buildLevelRunSampleDistances(runStartDistance_m, runEndDistance_m, waypointSpacing_mIn)
    distanceTol_m = 1e-9;

    if runEndDistance_m <= runStartDistance_m + distanceTol_m
        sampleDistance_m = runStartDistance_m;
        return
    end

    firstInteriorSample_m = ceil((runStartDistance_m + distanceTol_m) / waypointSpacing_mIn) * waypointSpacing_mIn;
    if firstInteriorSample_m < runEndDistance_m - distanceTol_m
        interiorSamples_m = (firstInteriorSample_m:waypointSpacing_mIn:(runEndDistance_m - distanceTol_m)).';
    else
        interiorSamples_m = zeros(0,1);
    end

    sampleDistance_m = [runStartDistance_m; interiorSamples_m; runEndDistance_m];
end

function waypointData = removeFinalWaypointFromData(waypointData)
    if size(waypointData.xyz,1) <= 1
        return
    end

    keepIdx = 1:(size(waypointData.xyz,1)-1);
    waypointData.xyz = waypointData.xyz(keepIdx,:);
    waypointData.time_s = waypointData.time_s(keepIdx);
    waypointData.sampleDistance_m = waypointData.sampleDistance_m(keepIdx);
end

function waypointData = includeSprayBoundaryCheckpoints(waypointData, sprayBoundaryData)
    if isempty(sprayBoundaryData.xyz)
        return
    end

    distanceTol_m = 1e-9;
    combinedXYZ = [waypointData.xyz; sprayBoundaryData.xyz];
    combinedTime_s = [waypointData.time_s; sprayBoundaryData.time_s];
    combinedDistance_m = [waypointData.sampleDistance_m; sprayBoundaryData.sampleDistance_m];

    [sortedDistance_m, sortIdx] = sort(combinedDistance_m);
    sortedXYZ = combinedXYZ(sortIdx,:);
    sortedTime_s = combinedTime_s(sortIdx);

    distanceKey = round(sortedDistance_m / distanceTol_m) * distanceTol_m;
    [~, keepIdx] = unique(distanceKey, 'stable');
    keepIdx = sort(keepIdx);

    waypointData.xyz = sortedXYZ(keepIdx,:);
    waypointData.time_s = sortedTime_s(keepIdx);
    waypointData.sampleDistance_m = sortedDistance_m(keepIdx);
end

function ned = enuToNed(enu)
    ned = [enu(:,2), enu(:,1), -enu(:,3)];
end

function allSortieWaypointData = buildAllSortieWaypointExports(simulationDataIn, sortieList, codesIn, waypointSpacing_m, finalEndpointRemoved)
    sortieCount = numel(sortieList);
    allSortieWaypointData = repmat(struct( ...
        'sortieNumber', NaN, ...
        'waypointsNED', zeros(0,3), ...
        'waypointsENU', zeros(0,3), ...
        'waypointTime_s', zeros(0,1), ...
        'waypointDistance_m', zeros(0,1), ...
        'sprayBoundaryWaypointsNED', zeros(0,3), ...
        'sprayBoundaryWaypointsENU', zeros(0,3), ...
        'sprayBoundaryWaypointTime_s', zeros(0,1), ...
        'sprayBoundaryWaypointDistance_m', zeros(0,1), ...
        'sprayBoundaryWaypointLabels', {cell(0,1)}), sortieCount, 1);

    for iSortie = 1:sortieCount
        sortieNumber = sortieList(iSortie);
        sortieData = extractSelectedSortieWaypointPath(simulationDataIn, sortieNumber, codesIn);
        waypointData = sampleWaypointsAlongPath(sortieData.xyz, sortieData.time_s, waypointSpacing_m);
        sprayBoundaryData = buildSprayBoundaryWaypointData(sortieData, codesIn.stage.sprayLeg);
        if finalEndpointRemoved
            waypointData = removeFinalWaypointFromData(waypointData);
        end
        waypointData = includeSprayBoundaryCheckpoints(waypointData, sprayBoundaryData);
        waypointData.ned = enuToNed(waypointData.xyz);
        sprayBoundaryData.ned = enuToNed(sprayBoundaryData.xyz);

        allSortieWaypointData(iSortie).sortieNumber = sortieNumber;
        allSortieWaypointData(iSortie).waypointsNED = waypointData.ned;
        allSortieWaypointData(iSortie).waypointsENU = waypointData.xyz;
        allSortieWaypointData(iSortie).waypointTime_s = waypointData.time_s;
        allSortieWaypointData(iSortie).waypointDistance_m = waypointData.sampleDistance_m;
        allSortieWaypointData(iSortie).sprayBoundaryWaypointsNED = sprayBoundaryData.ned;
        allSortieWaypointData(iSortie).sprayBoundaryWaypointsENU = sprayBoundaryData.xyz;
        allSortieWaypointData(iSortie).sprayBoundaryWaypointTime_s = sprayBoundaryData.time_s;
        allSortieWaypointData(iSortie).sprayBoundaryWaypointDistance_m = sprayBoundaryData.sampleDistance_m;
        allSortieWaypointData(iSortie).sprayBoundaryWaypointLabels = sprayBoundaryData.labels;
    end
end

function exportWaypointDataToBaseWorkspace(sortieNumber, waypointData, sprayBoundaryData, ...
        waypointSpacing_m, finalEndpointRemoved, allSortieWaypointData)
    waypointsNEDBySortie = {allSortieWaypointData.waypointsNED}.';
    waypointsENUBySortie = {allSortieWaypointData.waypointsENU}.';
    waypointTimeBySortie_s = {allSortieWaypointData.waypointTime_s}.';
    waypointDistanceBySortie_m = {allSortieWaypointData.waypointDistance_m}.';
    sprayBoundaryWaypointsNEDBySortie = {allSortieWaypointData.sprayBoundaryWaypointsNED}.';
    sprayBoundaryWaypointsENUBySortie = {allSortieWaypointData.sprayBoundaryWaypointsENU}.';
    sprayBoundaryWaypointTimeBySortie_s = {allSortieWaypointData.sprayBoundaryWaypointTime_s}.';
    sprayBoundaryWaypointDistanceBySortie_m = {allSortieWaypointData.sprayBoundaryWaypointDistance_m}.';
    sprayBoundaryWaypointLabelsBySortie = {allSortieWaypointData.sprayBoundaryWaypointLabels}.';
    waypointSortieNumbers = [allSortieWaypointData.sortieNumber].';
    maxSortieNumber = max(waypointSortieNumbers);
    waypoints = cell(maxSortieNumber, 1);
    waypointTimes = cell(maxSortieNumber, 1);
    waypointDistances = cell(maxSortieNumber, 1);
    waypointsENU = cell(maxSortieNumber, 1);
    spraywaypoints = cell(maxSortieNumber, 1);
    sprayBoundaryWaypoints = cell(maxSortieNumber, 1);
    sprayBoundaryWaypointTimes = cell(maxSortieNumber, 1);
    sprayBoundaryWaypointDistances = cell(maxSortieNumber, 1);
    sprayBoundaryWaypointsENU = cell(maxSortieNumber, 1);
    sprayBoundaryWaypointLabels = cell(maxSortieNumber, 1);
    for iSortie = 1:numel(waypointSortieNumbers)
        sortieNumberForCell = waypointSortieNumbers(iSortie);
        waypoints{sortieNumberForCell} = waypointsNEDBySortie{iSortie};
        waypointTimes{sortieNumberForCell} = waypointTimeBySortie_s{iSortie};
        waypointDistances{sortieNumberForCell} = waypointDistanceBySortie_m{iSortie};
        waypointsENU{sortieNumberForCell} = waypointsENUBySortie{iSortie};
        spraywaypoints{sortieNumberForCell} = sprayBoundaryWaypointsNEDBySortie{iSortie};
        sprayBoundaryWaypoints{sortieNumberForCell} = sprayBoundaryWaypointsNEDBySortie{iSortie};
        sprayBoundaryWaypointTimes{sortieNumberForCell} = sprayBoundaryWaypointTimeBySortie_s{iSortie};
        sprayBoundaryWaypointDistances{sortieNumberForCell} = sprayBoundaryWaypointDistanceBySortie_m{iSortie};
        sprayBoundaryWaypointsENU{sortieNumberForCell} = sprayBoundaryWaypointsENUBySortie{iSortie};
        sprayBoundaryWaypointLabels{sortieNumberForCell} = sprayBoundaryWaypointLabelsBySortie{iSortie};
    end

    exportStruct = struct();
    exportStruct.sortieNumber = sortieNumber;
    exportStruct.coordinateSystem = 'NED [North East Down]';
    exportStruct.altitudeConvention = 'Altitude commands remain positive Up; waypoint column 3 is Down.';
    exportStruct.spacing_m = waypointSpacing_m;
    exportStruct.finalEndpointRemoved = finalEndpointRemoved;
    exportStruct.firstCruiseWaypointKept = true;
    exportStruct.pathSelection = 'Full selected sortie path';
    exportStruct.waypointsNED = waypointData.ned;
    exportStruct.waypointsENU = waypointData.xyz;
    exportStruct.waypointTime_s = waypointData.time_s;
    exportStruct.waypointDistance_m = waypointData.sampleDistance_m;
    exportStruct.sprayBoundaryWaypointsNED = sprayBoundaryData.ned;
    exportStruct.sprayBoundaryWaypointsENU = sprayBoundaryData.xyz;
    exportStruct.sprayBoundaryWaypointTime_s = sprayBoundaryData.time_s;
    exportStruct.sprayBoundaryWaypointDistance_m = sprayBoundaryData.sampleDistance_m;
    exportStruct.sprayBoundaryWaypointLabels = sprayBoundaryData.labels;
    exportStruct.sortieNumbers = waypointSortieNumbers;
    exportStruct.waypointsNEDBySortie = waypointsNEDBySortie;
    exportStruct.waypointsENUBySortie = waypointsENUBySortie;
    exportStruct.waypointTimeBySortie_s = waypointTimeBySortie_s;
    exportStruct.waypointDistanceBySortie_m = waypointDistanceBySortie_m;
    exportStruct.sprayBoundaryWaypointsNEDBySortie = sprayBoundaryWaypointsNEDBySortie;
    exportStruct.sprayBoundaryWaypointsENUBySortie = sprayBoundaryWaypointsENUBySortie;
    exportStruct.sprayBoundaryWaypointTimeBySortie_s = sprayBoundaryWaypointTimeBySortie_s;
    exportStruct.sprayBoundaryWaypointDistanceBySortie_m = sprayBoundaryWaypointDistanceBySortie_m;
    exportStruct.sprayBoundaryWaypointLabelsBySortie = sprayBoundaryWaypointLabelsBySortie;
    exportStruct.waypoints = waypoints;
    exportStruct.waypointsENUIndexedBySortie = waypointsENU;
    exportStruct.waypointTimes = waypointTimes;
    exportStruct.waypointDistances = waypointDistances;
    exportStruct.spraywaypoints = spraywaypoints;
    exportStruct.sprayBoundaryWaypoints = sprayBoundaryWaypoints;
    exportStruct.sprayBoundaryWaypointsENUIndexedBySortie = sprayBoundaryWaypointsENU;
    exportStruct.sprayBoundaryWaypointTimes = sprayBoundaryWaypointTimes;
    exportStruct.sprayBoundaryWaypointDistances = sprayBoundaryWaypointDistances;
    exportStruct.sprayBoundaryWaypointLabelsIndexedBySortie = sprayBoundaryWaypointLabels;
    exportStruct.bySortie = allSortieWaypointData;

    assignin('base', 'selectedSortie', sortieNumber);
    assignin('base', 'selectedWaypointSortie', sortieNumber);
    assignin('base', 'waypointsNED', waypointData.ned);
    assignin('base', 'waypointsENU', waypointData.xyz);
    assignin('base', 'waypointTime_s', waypointData.time_s);
    assignin('base', 'waypointDistance_m', waypointData.sampleDistance_m);
    assignin('base', 'sprayBoundaryWaypointsNED', sprayBoundaryData.ned);
    assignin('base', 'sprayBoundaryWaypointsENU', sprayBoundaryData.xyz);
    assignin('base', 'sprayBoundaryWaypointTime_s', sprayBoundaryData.time_s);
    assignin('base', 'sprayBoundaryWaypointDistance_m', sprayBoundaryData.sampleDistance_m);
    assignin('base', 'sprayBoundaryWaypointLabels', sprayBoundaryData.labels);
    assignin('base', 'waypointSortieNumbers', waypointSortieNumbers);
    assignin('base', 'waypointsNEDBySortie', waypointsNEDBySortie);
    assignin('base', 'waypointsENUBySortie', waypointsENUBySortie);
    assignin('base', 'waypointTimeBySortie_s', waypointTimeBySortie_s);
    assignin('base', 'waypointDistanceBySortie_m', waypointDistanceBySortie_m);
    assignin('base', 'sprayBoundaryWaypointsNEDBySortie', sprayBoundaryWaypointsNEDBySortie);
    assignin('base', 'sprayBoundaryWaypointsENUBySortie', sprayBoundaryWaypointsENUBySortie);
    assignin('base', 'sprayBoundaryWaypointTimeBySortie_s', sprayBoundaryWaypointTimeBySortie_s);
    assignin('base', 'sprayBoundaryWaypointDistanceBySortie_m', sprayBoundaryWaypointDistanceBySortie_m);
    assignin('base', 'sprayBoundaryWaypointLabelsBySortie', sprayBoundaryWaypointLabelsBySortie);
    assignin('base', 'waypoints', waypoints);
    assignin('base', 'waypointsENUIndexedBySortie', waypointsENU);
    assignin('base', 'waypointTimes', waypointTimes);
    assignin('base', 'waypointDistances', waypointDistances);
    assignin('base', 'spraywaypoints', spraywaypoints);
    assignin('base', 'sprayBoundaryWaypoints', sprayBoundaryWaypoints);
    assignin('base', 'sprayBoundaryWaypointsENUIndexedBySortie', sprayBoundaryWaypointsENU);
    assignin('base', 'sprayBoundaryWaypointTimes', sprayBoundaryWaypointTimes);
    assignin('base', 'sprayBoundaryWaypointDistances', sprayBoundaryWaypointDistances);
    assignin('base', 'sprayBoundaryWaypointLabelsIndexedBySortie', sprayBoundaryWaypointLabels);
    assignin('base', 'sortieWaypointExport', exportStruct);
end

function outStr = logicalToYesNo(valueIn)
    if valueIn
        outStr = 'yes';
    else
        outStr = 'no';
    end
end

function outStr = formatLastDistance(sampleDistance_m)
    if isempty(sampleDistance_m)
        outStr = 'n/a';
    else
        outStr = sprintf('%.2f m', sampleDistance_m(end));
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

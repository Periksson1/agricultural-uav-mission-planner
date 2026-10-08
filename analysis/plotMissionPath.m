%% plotMissionPath
% Plot the full mission path in a separate figure using the same field
% context, view angles, and general styling as the other mission plots.
%
% Expected workflow:
% 1. Run runMission.m so missionCfg,
%    planningData, and simulationData exist in the workspace.
% 2. Run this script.
%
% The selected sortie's regular path waypoints, spray lane boundary
% waypoints, return run-out waypoints, and pre-climb waypoints are exported
% to the base workspace in NED coordinates.

%% -------------------- LOAD REQUIRED DATA --------------------
assert(exist('missionCfg', 'var') == 1, ...
    'missionCfg was not found. Run runMission.m first.');
assert(exist('planningData', 'var') == 1, ...
    'planningData was not found. Run runMission.m first.');
assert(exist('simulationData', 'var') == 1, ...
    'simulationData was not found. Run runMission.m first.');

validateMissionCfg(missionCfg);
validatePlanningData(planningData);
validateSimulationData(simulationData);
codes = missionCodes();
waypointSpacing_m = 2.0;
exportWaypointsToBaseWorkspace = true;

sortieList = unique(simulationData.segmentSortie(:));
sortieList = sortieList(isfinite(sortieList) & sortieList >= 1);
sortieList = sortieList(:).';
assert(~isempty(sortieList), 'No sortie information was found in simulationData.');
selectedSortie = sortieList(1);

%% -------------------- FIGURE --------------------
fig = figure('Color', 'w', ...
    'Name', 'Mission Path', 'NumberTitle', 'off');

ax = axes('Parent', fig);
set(ax, 'Position', [0.20 0.12 0.76 0.76]);
hold(ax, 'on')
drawRealWorldGroundContext(ax, planningData);
grid(ax, 'on')
axis(ax, 'equal')
daspect(ax, [1 1 1/max(missionCfg.verticalExaggeration, eps)])
xlabel(ax, 'East [m]')
ylabel(ax, 'North [m]')
zlabel(ax, 'Up [m]')
title(ax, 'Mission path by sortie')

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.84 0.08 0.03], 'String', 'Sortie', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'left', ...
    'FontSize', 10, 'FontWeight', 'bold');

hSortiePopup = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.08 0.838 0.10 0.04], ...
    'String', arrayfun(@(s) sprintf('Sortie %d', s), sortieList, 'UniformOutput', false), ...
    'Value', 1, 'BackgroundColor', 'w', 'FontSize', 10);

fieldFaceColor = [0.60 0.90 0.60];
groundStationFaceColor = [1.00 0.83 0.25];
missionPathColor = [0.18 0.43 0.82];
waypointColor = [0.93 0.45 0.12];

hFieldLegend = patch(ax, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
    'FaceColor', fieldFaceColor, 'EdgeColor', 'k', 'LineWidth', 2.0, ...
    'FaceAlpha', 0.22, 'DisplayName', 'Fields');
hGroundStation = fill3(ax, planningData.groundStationX, planningData.groundStationY, planningData.groundStationZ, ...
    groundStationFaceColor, 'EdgeColor', [0.60 0.40 0.00], 'LineWidth', 2.0, ...
    'FaceAlpha', 0.80, 'DisplayName', 'Ground station');
hMissionPath = plot3(ax, NaN, NaN, NaN, ...
    '-', 'Color', missionPathColor, 'LineWidth', 1.8, 'DisplayName', 'Mission path');
hWaypoints = plot3(ax, NaN, NaN, NaN, 'o', 'Color', waypointColor, ...
    'MarkerFaceColor', 'w', 'MarkerSize', 6.5, 'LineWidth', 1.2, 'DisplayName', 'Waypoint markers');
hSprayStartMarkers = plot3(ax, NaN, NaN, NaN, '^', 'Color', [0.00 0.60 0.20], ...
    'MarkerFaceColor', [0.55 0.95 0.55], 'MarkerSize', 12.0, 'LineWidth', 1.8, ...
    'DisplayName', 'Spray lane starts');
hSprayEndMarkers = plot3(ax, NaN, NaN, NaN, 's', 'Color', [0.80 0.10 0.10], ...
    'MarkerFaceColor', [1.00 0.75 0.75], 'MarkerSize', 11.5, 'LineWidth', 1.8, ...
    'DisplayName', 'Spray lane ends');
hSprayBoundaryWaypointMarkers = plot3(ax, NaN, NaN, NaN, 'o', 'Color', waypointColor, ...
    'MarkerFaceColor', 'none', 'MarkerSize', 5.2, 'LineWidth', 1.2, ...
    'HandleVisibility', 'off');
hPreClimbWaypointMarkers = plot3(ax, NaN, NaN, NaN, 'd', 'Color', [0.45 0.12 0.72], ...
    'MarkerFaceColor', [0.88 0.74 1.00], 'MarkerSize', 8.0, 'LineWidth', 1.4, ...
    'DisplayName', 'Pre-climb waypoints');
hReturnRunOutMarkers = plot3(ax, NaN, NaN, NaN, 'p', 'Color', [0.08 0.46 0.56], ...
    'MarkerFaceColor', [0.64 0.92 0.96], 'MarkerSize', 8.5, 'LineWidth', 1.4, ...
    'DisplayName', 'Return run-out waypoints');
hStart = plot3(ax, NaN, NaN, NaN, ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Start');
hEnd = plot3(ax, NaN, NaN, NaN, ...
    'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'End');

drawMissionContext(ax, planningData, fieldFaceColor);
text(ax, planningData.groundStationCentreXY(1), planningData.groundStationCentreXY(2), 0, ...
    '  Ground station', 'FontSize', 10, 'FontWeight', 'bold', ...
    'Color', [0.50 0.30 0.00], 'HitTest', 'off');

legend(ax, [hFieldLegend, hGroundStation, hMissionPath, hWaypoints, hSprayStartMarkers, hSprayEndMarkers, hPreClimbWaypointMarkers, hReturnRunOutMarkers, hStart, hEnd], ...
    'Location', 'northeast');

hStats = annotation(fig, 'textbox', [0.02 0.22 0.15 0.56], ...
    'String', '', 'FitBoxToText', 'off', ...
    'BackgroundColor', 'w', 'EdgeColor', 'k', 'FontSize', 10, ...
    'Interpreter', 'none');

setappdata(fig, 'axMission', ax);
setappdata(fig, 'hMissionPath', hMissionPath);
setappdata(fig, 'hWaypoints', hWaypoints);
setappdata(fig, 'hSprayStartMarkers', hSprayStartMarkers);
setappdata(fig, 'hSprayEndMarkers', hSprayEndMarkers);
setappdata(fig, 'hSprayBoundaryWaypointMarkers', hSprayBoundaryWaypointMarkers);
setappdata(fig, 'hPreClimbWaypointMarkers', hPreClimbWaypointMarkers);
setappdata(fig, 'hReturnRunOutMarkers', hReturnRunOutMarkers);
setappdata(fig, 'hStart', hStart);
setappdata(fig, 'hEnd', hEnd);
setappdata(fig, 'hStats', hStats);
setappdata(fig, 'missionCfg', missionCfg);
setappdata(fig, 'planningData', planningData);
setappdata(fig, 'simulationData', simulationData);
setappdata(fig, 'sortieList', sortieList);
setappdata(fig, 'codes', codes);
setappdata(fig, 'waypointSpacing_m', waypointSpacing_m);
setappdata(fig, 'exportWaypointsToBaseWorkspace', exportWaypointsToBaseWorkspace);
setappdata(fig, 'sprayBoundaryLabels', gobjects(0,1));

set(hSortiePopup, 'Callback', @(src,~) refreshMissionPathFigure(fig, sortieList(get(src, 'Value'))));
refreshMissionPathFigure(fig, selectedSortie);

%% -------------------- LOCAL FUNCTIONS --------------------
function refreshMissionPathFigure(figHandle, sortieNumber)
    axHandle = getappdata(figHandle, 'axMission');
    hMissionPathHandle = getappdata(figHandle, 'hMissionPath');
    hWaypointsHandle = getappdata(figHandle, 'hWaypoints');
    hSprayStartMarkersHandle = getappdata(figHandle, 'hSprayStartMarkers');
    hSprayEndMarkersHandle = getappdata(figHandle, 'hSprayEndMarkers');
    hSprayBoundaryWaypointMarkersHandle = getappdata(figHandle, 'hSprayBoundaryWaypointMarkers');
    hPreClimbWaypointMarkersHandle = getappdata(figHandle, 'hPreClimbWaypointMarkers');
    hReturnRunOutMarkersHandle = getappdata(figHandle, 'hReturnRunOutMarkers');
    hStartHandle = getappdata(figHandle, 'hStart');
    hEndHandle = getappdata(figHandle, 'hEnd');
    hStatsHandle = getappdata(figHandle, 'hStats');
    missionCfgIn = getappdata(figHandle, 'missionCfg');
    planningDataIn = getappdata(figHandle, 'planningData');
    simulationDataIn = getappdata(figHandle, 'simulationData');
    codesIn = getappdata(figHandle, 'codes');
    waypointSpacing_mIn = getappdata(figHandle, 'waypointSpacing_m');
    exportWaypointsToBaseWorkspaceIn = getappdata(figHandle, 'exportWaypointsToBaseWorkspace');

    oldSprayLabels = getappdata(figHandle, 'sprayBoundaryLabels');
    if ~isempty(oldSprayLabels)
        delete(oldSprayLabels(ishghandle(oldSprayLabels)));
    end

    sortieData = extractSelectedSortiePath(simulationDataIn, sortieNumber, codesIn);
    waypointData = sampleWaypointsAlongPath(sortieData.xyz, sortieData.time_s, waypointSpacing_mIn);
    sprayBoundaryData = buildSprayBoundaryWaypointData(sortieData, codesIn.stage.sprayLeg);
    preClimbData = buildPreClimbWaypointData(sortieData, codesIn);
    returnRunOutData = buildReturnRunOutWaypointData(sortieData, codesIn);
    waypointData = includeCheckpointData(waypointData, sprayBoundaryData);
    waypointData = includeCheckpointData(waypointData, returnRunOutData);
    waypointData = includeCheckpointData(waypointData, preClimbData);
    waypointData.ned = enuToNed(waypointData.xyz);
    sprayBoundaryData.ned = enuToNed(sprayBoundaryData.xyz);
    returnRunOutData.ned = enuToNed(returnRunOutData.xyz);
    preClimbData.ned = enuToNed(preClimbData.xyz);

    if exportWaypointsToBaseWorkspaceIn
        allSortieWaypointData = buildAllSortieWaypointExports(simulationDataIn, ...
            getappdata(figHandle, 'sortieList'), codesIn, waypointSpacing_mIn);
        exportWaypointDataToBaseWorkspace(sortieNumber, waypointData, sprayBoundaryData, returnRunOutData, preClimbData, ...
            waypointSpacing_mIn, allSortieWaypointData);
    end

    set(hMissionPathHandle, 'XData', sortieData.xyz(:,1), 'YData', sortieData.xyz(:,2), 'ZData', sortieData.xyz(:,3));
    set(hWaypointsHandle, 'XData', waypointData.xyz(:,1), 'YData', waypointData.xyz(:,2), 'ZData', waypointData.xyz(:,3));
    set(hSprayStartMarkersHandle, 'XData', sprayBoundaryData.xyz(sprayBoundaryData.isStart,1), ...
        'YData', sprayBoundaryData.xyz(sprayBoundaryData.isStart,2), ...
        'ZData', sprayBoundaryData.xyz(sprayBoundaryData.isStart,3));
    set(hSprayEndMarkersHandle, 'XData', sprayBoundaryData.xyz(sprayBoundaryData.isEnd,1), ...
        'YData', sprayBoundaryData.xyz(sprayBoundaryData.isEnd,2), ...
        'ZData', sprayBoundaryData.xyz(sprayBoundaryData.isEnd,3));
    set(hSprayBoundaryWaypointMarkersHandle, 'XData', sprayBoundaryData.xyz(:,1), ...
        'YData', sprayBoundaryData.xyz(:,2), ...
        'ZData', sprayBoundaryData.xyz(:,3));
    set(hPreClimbWaypointMarkersHandle, 'XData', preClimbData.xyz(:,1), ...
        'YData', preClimbData.xyz(:,2), ...
        'ZData', preClimbData.xyz(:,3));
    set(hReturnRunOutMarkersHandle, 'XData', returnRunOutData.xyz(:,1), ...
        'YData', returnRunOutData.xyz(:,2), ...
        'ZData', returnRunOutData.xyz(:,3));
    set(hStartHandle, 'XData', sortieData.xyz(1,1), 'YData', sortieData.xyz(1,2), 'ZData', sortieData.xyz(1,3));
    set(hEndHandle, 'XData', sortieData.xyz(end,1), 'YData', sortieData.xyz(end,2), 'ZData', sortieData.xyz(end,3));

    labelOffset = max(1.0, 0.02 * max(waypointData.totalDistance_m, 1));
    sprayBoundaryLabels = gobjects(size(sprayBoundaryData.xyz,1),1);
    for iBoundary = 1:size(sprayBoundaryData.xyz,1)
        if sprayBoundaryData.isStart(iBoundary)
            labelColor = [0.00 0.45 0.15];
        else
            labelColor = [0.70 0.05 0.05];
        end
        sprayBoundaryLabels(iBoundary) = text(axHandle, ...
            sprayBoundaryData.xyz(iBoundary,1), sprayBoundaryData.xyz(iBoundary,2), ...
            sprayBoundaryData.xyz(iBoundary,3) + 1.5 * labelOffset, ...
            sprayBoundaryData.labels{iBoundary}, 'FontSize', 8.5, 'Color', labelColor, ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Clipping', 'on');
    end
    setappdata(figHandle, 'sprayBoundaryLabels', sprayBoundaryLabels);

    set(hStatsHandle, 'String', buildSortieStatsLines(sortieData, waypointData, ...
        sprayBoundaryData, returnRunOutData, preClimbData, waypointSpacing_mIn, exportWaypointsToBaseWorkspaceIn));

    updateMissionAxes(axHandle, missionCfgIn, planningDataIn, ...
        [sortieData.xyz; waypointData.xyz; sprayBoundaryData.xyz; returnRunOutData.xyz; preClimbData.xyz]);
    view(axHandle, missionCfgIn.viewAzimuth_deg, missionCfgIn.viewElevation_deg);
    camproj(axHandle, 'perspective');
    daspect(axHandle, [1 1 1/max(missionCfgIn.verticalExaggeration, eps)])
    title(axHandle, sprintf('Mission path: sortie %d', sortieNumber))
end

function drawMissionContext(axHandle, planningDataIn, fieldFaceColorIn)
    for visitIdx = 1:planningDataIn.fieldCount
        fIdx = planningDataIn.fieldVisitOrder(visitIdx);
        displayLoops = planningDataIn.fieldInfo(fIdx).displayBoundaryLoops;

        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy) || size(cxy,1) < 3
                continue
            end
            fill3(axHandle, cxy(:,1), cxy(:,2), groundZForXY(planningDataIn, cxy), fieldFaceColorIn, ...
                'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.22, ...
                'HandleVisibility', 'off');
        end

        labelZ = groundZForXY(planningDataIn, planningDataIn.fieldInfo(fIdx).centreXY);
        text(axHandle, planningDataIn.fieldInfo(fIdx).centreXY(1), ...
            planningDataIn.fieldInfo(fIdx).centreXY(2), labelZ, ...
            sprintf('  %s', planningDataIn.fieldInfo(fIdx).name), ...
            'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0 0], ...
            'HitTest', 'off');
    end
end

function updateMissionAxes(axHandle, missionCfgIn, planningDataIn, missionXYZ)
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

    xCandidates = [allFieldX(:); planningDataIn.groundStationX(:); missionXYZ(:,1)];
    yCandidates = [allFieldY(:); planningDataIn.groundStationY(:); missionXYZ(:,2)];
    zCandidates = [missionXYZ(:,3); planningDataIn.groundStationZ(:); collectGroundElevationCandidates(planningDataIn)];

    xRange = max(xCandidates) - min(xCandidates);
    yRange = max(yCandidates) - min(yCandidates);
    zRange = max(zCandidates) - min(zCandidates);
    xMargin = max(5, 0.03 * max(xRange, 1));
    yMargin = max(5, 0.03 * max(yRange, 1));
    zMargin = max(1, 0.20 * max(zRange, missionCfgIn.vtolAltitude_m));

    xlim(axHandle, [min(xCandidates) - xMargin, max(xCandidates) + xMargin])
    ylim(axHandle, [min(yCandidates) - yMargin, max(yCandidates) + yMargin])
    zlim(axHandle, [min(zCandidates) - zMargin, max(zCandidates) + zMargin])
end

function drawRealWorldGroundContext(axHandle, planningDataIn)
    if ~isfield(planningDataIn, 'geodata') || ~isstruct(planningDataIn.geodata)
        return
    end
    geodata = planningDataIn.geodata;
    if isfield(geodata, 'terrainModel') && isstruct(geodata.terrainModel) && ...
            isfield(geodata.terrainModel, 'pointsXY') && ~isempty(geodata.terrainModel.pointsXY)
        pointsXY = geodata.terrainModel.pointsXY;
        xGrid = linspace(min(pointsXY(:,1)), max(pointsXY(:,1)), 55);
        yGrid = linspace(min(pointsXY(:,2)), max(pointsXY(:,2)), 55);
        [X, Y] = meshgrid(xGrid, yGrid);
        interpolant = scatteredInterpolant(pointsXY(:,1), pointsXY(:,2), ...
            geodata.terrainModel.elevationM(:), 'natural', 'none');
        Z = interpolant(X, Y);
        surf(axHandle, X, Y, Z, Z, 'EdgeColor', 'none', ...
            'FaceAlpha', 0.22, 'HandleVisibility', 'off', 'HitTest', 'off');
        colormap(axHandle, parula(256));
    end
    if isfield(geodata, 'groundImage') && isstruct(geodata.groundImage) && ...
            isfield(geodata.groundImage, 'image') && ~isempty(geodata.groundImage.image)
        imageInfo = geodata.groundImage;
        xLimits = imageInfo.xLimits;
        yLimits = imageInfo.yLimits;
        zPlane = 0;
        if isfield(geodata, 'terrainModel') && isfield(geodata.terrainModel, 'pointsXY') && ...
                ~isempty(geodata.terrainModel.pointsXY)
            zPlane = sampleTerrainAtXY(geodata.terrainModel, [mean(xLimits), mean(yLimits)]) - 0.05;
        end
        surface(axHandle, [xLimits(1), xLimits(2); xLimits(1), xLimits(2)], ...
            [yLimits(2), yLimits(2); yLimits(1), yLimits(1)], ...
            zPlane * ones(2,2), imageInfo.image, ...
            'FaceColor', 'texturemap', 'EdgeColor', 'none', ...
            'FaceAlpha', 0.72, 'HandleVisibility', 'off', 'HitTest', 'off');
    end
end

function zCandidates = collectGroundElevationCandidates(planningDataIn)
    zCandidates = 0;
    if isfield(planningDataIn, 'geodata') && isfield(planningDataIn.geodata, 'terrainModel') && ...
            isstruct(planningDataIn.geodata.terrainModel) && ...
            isfield(planningDataIn.geodata.terrainModel, 'elevationM') && ...
            ~isempty(planningDataIn.geodata.terrainModel.elevationM)
        zCandidates = planningDataIn.geodata.terrainModel.elevationM(:);
    end
end

function z = groundZForXY(planningDataIn, xy)
    z = zeros(size(xy,1), 1);
    if isfield(planningDataIn, 'geodata') && isfield(planningDataIn.geodata, 'terrainModel') && ...
            isstruct(planningDataIn.geodata.terrainModel) && ...
            isfield(planningDataIn.geodata.terrainModel, 'pointsXY') && ...
            ~isempty(planningDataIn.geodata.terrainModel.pointsXY)
        z = sampleTerrainAtXY(planningDataIn.geodata.terrainModel, xy);
    end
end

function z = sampleTerrainAtXY(terrainModel, xy)
    interpolant = scatteredInterpolant(terrainModel.pointsXY(:,1), terrainModel.pointsXY(:,2), ...
        terrainModel.elevationM(:), 'natural', 'nearest');
    z = interpolant(xy(:,1), xy(:,2));
end

function sortieData = extractSelectedSortiePath(simulationDataIn, sortieNumber, codesIn)
    idxSeg = find(simulationDataIn.segmentSortie == sortieNumber);
    assert(~isempty(idxSeg), 'Sortie %d was not found in simulationData.', sortieNumber);

    firstSeg = idxSeg(1);
    lastSeg = idxSeg(end);

    sortieData.sortieNumber = sortieNumber;
    sortieData.xyz = simulationDataIn.pathXYZ(firstSeg:lastSeg+1,:);
    sortieData.time_s = simulationDataIn.pathTime(firstSeg:lastSeg+1);
    sortieData.time_s = sortieData.time_s - sortieData.time_s(1);
    sortieData.stages = simulationDataIn.segmentStages(firstSeg:lastSeg);
    sortieData.segmentLengths_m = vecnorm(diff(sortieData.xyz,1,1), 2, 2);
    sortieData.cumulativeDistance_m = [0; cumsum(sortieData.segmentLengths_m)];
    sortieData.totalDistance_m = sum(sortieData.segmentLengths_m);
    sortieData.sprayDistance_m = sum(sortieData.segmentLengths_m(sortieData.stages == codesIn.stage.sprayLeg));
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

function preClimbData = buildPreClimbWaypointData(sortieDataIn, codesIn)
    preClimbData = struct();
    preClimbData.xyz = zeros(0,3);
    preClimbData.time_s = zeros(0,1);
    preClimbData.sampleDistance_m = zeros(0,1);
    preClimbData.labels = cell(0,1);
    preClimbData.stageCode = zeros(0,1);
    preClimbData.climbCount = 0;

    if isempty(sortieDataIn.stages) || size(sortieDataIn.xyz,1) < 2
        return
    end

    climbStageCodes = codesIn.stage.interPassClimbOut;
    if isfield(codesIn.stage, 'returnPullUp')
        climbStageCodes = [climbStageCodes, codesIn.stage.returnPullUp];
    end

    climbMask = ismember(sortieDataIn.stages, climbStageCodes);
    if ~any(climbMask)
        return
    end

    climbStartSegIdx = find(climbMask & [true; ~climbMask(1:end-1)]);
    climbCount = numel(climbStartSegIdx);

    xyz = zeros(climbCount, 3);
    time_s = zeros(climbCount, 1);
    sampleDistance_m = zeros(climbCount, 1);
    labels = cell(climbCount, 1);
    stageCode = zeros(climbCount, 1);

    for iClimb = 1:climbCount
        iStartSeg = climbStartSegIdx(iClimb);
        startPointIdx = iStartSeg;

        xyz(iClimb,:) = sortieDataIn.xyz(startPointIdx,:);
        time_s(iClimb) = sortieDataIn.time_s(startPointIdx);
        sampleDistance_m(iClimb) = sortieDataIn.cumulativeDistance_m(startPointIdx);
        stageCode(iClimb) = sortieDataIn.stages(iStartSeg);
        labels{iClimb} = sprintf('C%d pre-climb', iClimb);
    end

    preClimbData.xyz = xyz;
    preClimbData.time_s = time_s;
    preClimbData.sampleDistance_m = sampleDistance_m;
    preClimbData.labels = labels;
    preClimbData.stageCode = stageCode;
    preClimbData.climbCount = climbCount;
end

function returnRunOutData = buildReturnRunOutWaypointData(sortieDataIn, codesIn)
    returnRunOutData = struct();
    returnRunOutData.xyz = zeros(0,3);
    returnRunOutData.time_s = zeros(0,1);
    returnRunOutData.sampleDistance_m = zeros(0,1);
    returnRunOutData.labels = cell(0,1);
    returnRunOutData.isStart = false(0,1);
    returnRunOutData.isEnd = false(0,1);
    returnRunOutData.runOutCount = 0;

    if ~isfield(codesIn.stage, 'returnRunOut') || isempty(sortieDataIn.stages) || size(sortieDataIn.xyz,1) < 2
        return
    end

    runOutMask = sortieDataIn.stages == codesIn.stage.returnRunOut;
    if ~any(runOutMask)
        return
    end

    runOutStartSegIdx = find(runOutMask & [true; ~runOutMask(1:end-1)]);
    runOutEndSegIdx = find(runOutMask & [~runOutMask(2:end); true]);
    runOutCount = numel(runOutStartSegIdx);

    xyz = zeros(2 * runOutCount, 3);
    time_s = zeros(2 * runOutCount, 1);
    sampleDistance_m = zeros(2 * runOutCount, 1);
    labels = cell(2 * runOutCount, 1);
    isStart = false(2 * runOutCount, 1);
    isEnd = false(2 * runOutCount, 1);

    for iRunOut = 1:runOutCount
        iStartSeg = runOutStartSegIdx(iRunOut);
        iEndSeg = runOutEndSegIdx(iRunOut);

        iRowStart = 2 * iRunOut - 1;
        iRowEnd = 2 * iRunOut;
        startPointIdx = iStartSeg;
        endPointIdx = iEndSeg + 1;

        xyz(iRowStart,:) = sortieDataIn.xyz(startPointIdx,:);
        xyz(iRowEnd,:) = sortieDataIn.xyz(endPointIdx,:);
        time_s(iRowStart) = sortieDataIn.time_s(startPointIdx);
        time_s(iRowEnd) = sortieDataIn.time_s(endPointIdx);
        sampleDistance_m(iRowStart) = sortieDataIn.cumulativeDistance_m(startPointIdx);
        sampleDistance_m(iRowEnd) = sortieDataIn.cumulativeDistance_m(endPointIdx);
        labels{iRowStart} = sprintf('R%d return-out start', iRunOut);
        labels{iRowEnd} = sprintf('R%d return-out end', iRunOut);
        isStart(iRowStart) = true;
        isEnd(iRowEnd) = true;
    end

    returnRunOutData.xyz = xyz;
    returnRunOutData.time_s = time_s;
    returnRunOutData.sampleDistance_m = sampleDistance_m;
    returnRunOutData.labels = labels;
    returnRunOutData.isStart = isStart;
    returnRunOutData.isEnd = isEnd;
    returnRunOutData.runOutCount = runOutCount;
end

function statsText = buildSortieStatsLines(sortieData, waypointData, sprayBoundaryData, returnRunOutData, preClimbData, ...
        waypointSpacing_m, exportWaypointsToBaseWorkspace)
    statsLines = {
        sprintf('Sortie %d', sortieData.sortieNumber)
        ''
        sprintf('Path points: %d', size(sortieData.xyz, 1))
        sprintf('Segments: %d', numel(sortieData.stages))
        sprintf('Waypoints @ %.0f m: %d', waypointSpacing_m, size(waypointData.xyz, 1))
        'Spray lane starts/ends included as waypoints'
        'Return run-out starts/ends included as waypoints'
        'Pre-climb low-altitude endpoints included as waypoints'
        sprintf('Spray boundary waypoints: %d', size(sprayBoundaryData.xyz, 1))
        sprintf('Return run-out waypoints: %d', size(returnRunOutData.xyz, 1))
        sprintf('Pre-climb waypoints: %d', size(preClimbData.xyz, 1))
        sprintf('Spray lanes: %d', sprayBoundaryData.laneCount)
        sprintf('Exported NED: %s', logicalToYesNo(exportWaypointsToBaseWorkspace))
        ''
        sprintf('Sortie time: %.2f s', sortieData.time_s(end))
        sprintf('Sortie distance: %.2f m', sortieData.totalDistance_m)
        sprintf('Spray distance: %.2f m', sortieData.sprayDistance_m)
        ''
        sprintf('Start: [%.1f, %.1f, %.1f] m', ...
            sortieData.xyz(1,1), sortieData.xyz(1,2), sortieData.xyz(1,3))
        sprintf('End: [%.1f, %.1f, %.1f] m', ...
            sortieData.xyz(end,1), sortieData.xyz(end,2), sortieData.xyz(end,3))
        };
    statsText = strjoin(statsLines, newline);
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

    sampleDistance_m = (0:waypointSpacing_mIn:totalDistance_m).';
    if isempty(sampleDistance_m) || abs(sampleDistance_m(end) - totalDistance_m) > 1e-9
        sampleDistance_m(end+1,1) = totalDistance_m;
    end

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

function waypointData = includeCheckpointData(waypointData, checkpointData)
    if isempty(checkpointData.xyz)
        return
    end

    distanceTol_m = 1e-9;
    combinedXYZ = [waypointData.xyz; checkpointData.xyz];
    combinedTime_s = [waypointData.time_s; checkpointData.time_s];
    combinedDistance_m = [waypointData.sampleDistance_m; checkpointData.sampleDistance_m];

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

function allSortieWaypointData = buildAllSortieWaypointExports(simulationDataIn, sortieListIn, codesIn, waypointSpacing_m)
    sortieCount = numel(sortieListIn);
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
        'sprayBoundaryWaypointLabels', {cell(0,1)}, ...
        'returnRunOutWaypointsNED', zeros(0,3), ...
        'returnRunOutWaypointsENU', zeros(0,3), ...
        'returnRunOutWaypointTime_s', zeros(0,1), ...
        'returnRunOutWaypointDistance_m', zeros(0,1), ...
        'returnRunOutWaypointLabels', {cell(0,1)}, ...
        'preClimbWaypointsNED', zeros(0,3), ...
        'preClimbWaypointsENU', zeros(0,3), ...
        'preClimbWaypointTime_s', zeros(0,1), ...
        'preClimbWaypointDistance_m', zeros(0,1), ...
        'preClimbWaypointLabels', {cell(0,1)}), sortieCount, 1);

    for iSortie = 1:sortieCount
        sortieNumber = sortieListIn(iSortie);
        sortieData = extractSelectedSortiePath(simulationDataIn, sortieNumber, codesIn);
        waypointData = sampleWaypointsAlongPath(sortieData.xyz, sortieData.time_s, waypointSpacing_m);
        sprayBoundaryData = buildSprayBoundaryWaypointData(sortieData, codesIn.stage.sprayLeg);
        returnRunOutData = buildReturnRunOutWaypointData(sortieData, codesIn);
        preClimbData = buildPreClimbWaypointData(sortieData, codesIn);
        waypointData = includeCheckpointData(waypointData, sprayBoundaryData);
        waypointData = includeCheckpointData(waypointData, returnRunOutData);
        waypointData = includeCheckpointData(waypointData, preClimbData);
        waypointData.ned = enuToNed(waypointData.xyz);
        sprayBoundaryData.ned = enuToNed(sprayBoundaryData.xyz);
        returnRunOutData.ned = enuToNed(returnRunOutData.xyz);
        preClimbData.ned = enuToNed(preClimbData.xyz);

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
        allSortieWaypointData(iSortie).returnRunOutWaypointsNED = returnRunOutData.ned;
        allSortieWaypointData(iSortie).returnRunOutWaypointsENU = returnRunOutData.xyz;
        allSortieWaypointData(iSortie).returnRunOutWaypointTime_s = returnRunOutData.time_s;
        allSortieWaypointData(iSortie).returnRunOutWaypointDistance_m = returnRunOutData.sampleDistance_m;
        allSortieWaypointData(iSortie).returnRunOutWaypointLabels = returnRunOutData.labels;
        allSortieWaypointData(iSortie).preClimbWaypointsNED = preClimbData.ned;
        allSortieWaypointData(iSortie).preClimbWaypointsENU = preClimbData.xyz;
        allSortieWaypointData(iSortie).preClimbWaypointTime_s = preClimbData.time_s;
        allSortieWaypointData(iSortie).preClimbWaypointDistance_m = preClimbData.sampleDistance_m;
        allSortieWaypointData(iSortie).preClimbWaypointLabels = preClimbData.labels;
    end
end

function exportWaypointDataToBaseWorkspace(sortieNumber, waypointData, sprayBoundaryData, returnRunOutData, preClimbData, ...
        waypointSpacing_m, allSortieWaypointData)
    waypointsNEDBySortie = {allSortieWaypointData.waypointsNED}.';
    waypointsENUBySortie = {allSortieWaypointData.waypointsENU}.';
    waypointTimeBySortie_s = {allSortieWaypointData.waypointTime_s}.';
    waypointDistanceBySortie_m = {allSortieWaypointData.waypointDistance_m}.';
    sprayBoundaryWaypointsNEDBySortie = {allSortieWaypointData.sprayBoundaryWaypointsNED}.';
    sprayBoundaryWaypointsENUBySortie = {allSortieWaypointData.sprayBoundaryWaypointsENU}.';
    sprayBoundaryWaypointTimeBySortie_s = {allSortieWaypointData.sprayBoundaryWaypointTime_s}.';
    sprayBoundaryWaypointDistanceBySortie_m = {allSortieWaypointData.sprayBoundaryWaypointDistance_m}.';
    sprayBoundaryWaypointLabelsBySortie = {allSortieWaypointData.sprayBoundaryWaypointLabels}.';
    returnRunOutWaypointsNEDBySortie = {allSortieWaypointData.returnRunOutWaypointsNED}.';
    returnRunOutWaypointsENUBySortie = {allSortieWaypointData.returnRunOutWaypointsENU}.';
    returnRunOutWaypointTimeBySortie_s = {allSortieWaypointData.returnRunOutWaypointTime_s}.';
    returnRunOutWaypointDistanceBySortie_m = {allSortieWaypointData.returnRunOutWaypointDistance_m}.';
    returnRunOutWaypointLabelsBySortie = {allSortieWaypointData.returnRunOutWaypointLabels}.';
    preClimbWaypointsNEDBySortie = {allSortieWaypointData.preClimbWaypointsNED}.';
    preClimbWaypointsENUBySortie = {allSortieWaypointData.preClimbWaypointsENU}.';
    preClimbWaypointTimeBySortie_s = {allSortieWaypointData.preClimbWaypointTime_s}.';
    preClimbWaypointDistanceBySortie_m = {allSortieWaypointData.preClimbWaypointDistance_m}.';
    preClimbWaypointLabelsBySortie = {allSortieWaypointData.preClimbWaypointLabels}.';
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
    returnRunOutWaypoints = cell(maxSortieNumber, 1);
    returnRunOutWaypointTimes = cell(maxSortieNumber, 1);
    returnRunOutWaypointDistances = cell(maxSortieNumber, 1);
    returnRunOutWaypointsENU = cell(maxSortieNumber, 1);
    returnRunOutWaypointLabels = cell(maxSortieNumber, 1);
    preClimbWaypoints = cell(maxSortieNumber, 1);
    preClimbWaypointTimes = cell(maxSortieNumber, 1);
    preClimbWaypointDistances = cell(maxSortieNumber, 1);
    preClimbWaypointsENU = cell(maxSortieNumber, 1);
    preClimbWaypointLabels = cell(maxSortieNumber, 1);

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
        returnRunOutWaypoints{sortieNumberForCell} = returnRunOutWaypointsNEDBySortie{iSortie};
        returnRunOutWaypointTimes{sortieNumberForCell} = returnRunOutWaypointTimeBySortie_s{iSortie};
        returnRunOutWaypointDistances{sortieNumberForCell} = returnRunOutWaypointDistanceBySortie_m{iSortie};
        returnRunOutWaypointsENU{sortieNumberForCell} = returnRunOutWaypointsENUBySortie{iSortie};
        returnRunOutWaypointLabels{sortieNumberForCell} = returnRunOutWaypointLabelsBySortie{iSortie};
        preClimbWaypoints{sortieNumberForCell} = preClimbWaypointsNEDBySortie{iSortie};
        preClimbWaypointTimes{sortieNumberForCell} = preClimbWaypointTimeBySortie_s{iSortie};
        preClimbWaypointDistances{sortieNumberForCell} = preClimbWaypointDistanceBySortie_m{iSortie};
        preClimbWaypointsENU{sortieNumberForCell} = preClimbWaypointsENUBySortie{iSortie};
        preClimbWaypointLabels{sortieNumberForCell} = preClimbWaypointLabelsBySortie{iSortie};
    end

    exportStruct = struct();
    exportStruct.sortieNumber = sortieNumber;
    exportStruct.coordinateSystem = 'NED [North East Down]';
    exportStruct.altitudeConvention = 'Altitude commands remain positive Up; waypoint column 3 is Down.';
    exportStruct.spacing_m = waypointSpacing_m;
    exportStruct.finalEndpointRemoved = false;
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
    exportStruct.returnRunOutWaypointsNED = returnRunOutData.ned;
    exportStruct.returnRunOutWaypointsENU = returnRunOutData.xyz;
    exportStruct.returnRunOutWaypointTime_s = returnRunOutData.time_s;
    exportStruct.returnRunOutWaypointDistance_m = returnRunOutData.sampleDistance_m;
    exportStruct.returnRunOutWaypointLabels = returnRunOutData.labels;
    exportStruct.preClimbWaypointsNED = preClimbData.ned;
    exportStruct.preClimbWaypointsENU = preClimbData.xyz;
    exportStruct.preClimbWaypointTime_s = preClimbData.time_s;
    exportStruct.preClimbWaypointDistance_m = preClimbData.sampleDistance_m;
    exportStruct.preClimbWaypointLabels = preClimbData.labels;
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
    exportStruct.returnRunOutWaypointsNEDBySortie = returnRunOutWaypointsNEDBySortie;
    exportStruct.returnRunOutWaypointsENUBySortie = returnRunOutWaypointsENUBySortie;
    exportStruct.returnRunOutWaypointTimeBySortie_s = returnRunOutWaypointTimeBySortie_s;
    exportStruct.returnRunOutWaypointDistanceBySortie_m = returnRunOutWaypointDistanceBySortie_m;
    exportStruct.returnRunOutWaypointLabelsBySortie = returnRunOutWaypointLabelsBySortie;
    exportStruct.preClimbWaypointsNEDBySortie = preClimbWaypointsNEDBySortie;
    exportStruct.preClimbWaypointsENUBySortie = preClimbWaypointsENUBySortie;
    exportStruct.preClimbWaypointTimeBySortie_s = preClimbWaypointTimeBySortie_s;
    exportStruct.preClimbWaypointDistanceBySortie_m = preClimbWaypointDistanceBySortie_m;
    exportStruct.preClimbWaypointLabelsBySortie = preClimbWaypointLabelsBySortie;
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
    exportStruct.returnRunOutWaypoints = returnRunOutWaypoints;
    exportStruct.returnRunOutWaypointsENUIndexedBySortie = returnRunOutWaypointsENU;
    exportStruct.returnRunOutWaypointTimes = returnRunOutWaypointTimes;
    exportStruct.returnRunOutWaypointDistances = returnRunOutWaypointDistances;
    exportStruct.returnRunOutWaypointLabelsIndexedBySortie = returnRunOutWaypointLabels;
    exportStruct.preClimbWaypoints = preClimbWaypoints;
    exportStruct.preClimbWaypointsENUIndexedBySortie = preClimbWaypointsENU;
    exportStruct.preClimbWaypointTimes = preClimbWaypointTimes;
    exportStruct.preClimbWaypointDistances = preClimbWaypointDistances;
    exportStruct.preClimbWaypointLabelsIndexedBySortie = preClimbWaypointLabels;
    exportStruct.bySortie = allSortieWaypointData;

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
    assignin('base', 'returnRunOutWaypointsNED', returnRunOutData.ned);
    assignin('base', 'returnRunOutWaypointsENU', returnRunOutData.xyz);
    assignin('base', 'returnRunOutWaypointTime_s', returnRunOutData.time_s);
    assignin('base', 'returnRunOutWaypointDistance_m', returnRunOutData.sampleDistance_m);
    assignin('base', 'returnRunOutWaypointLabels', returnRunOutData.labels);
    assignin('base', 'preClimbWaypointsNED', preClimbData.ned);
    assignin('base', 'preClimbWaypointsENU', preClimbData.xyz);
    assignin('base', 'preClimbWaypointTime_s', preClimbData.time_s);
    assignin('base', 'preClimbWaypointDistance_m', preClimbData.sampleDistance_m);
    assignin('base', 'preClimbWaypointLabels', preClimbData.labels);
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
    assignin('base', 'returnRunOutWaypointsNEDBySortie', returnRunOutWaypointsNEDBySortie);
    assignin('base', 'returnRunOutWaypointsENUBySortie', returnRunOutWaypointsENUBySortie);
    assignin('base', 'returnRunOutWaypointTimeBySortie_s', returnRunOutWaypointTimeBySortie_s);
    assignin('base', 'returnRunOutWaypointDistanceBySortie_m', returnRunOutWaypointDistanceBySortie_m);
    assignin('base', 'returnRunOutWaypointLabelsBySortie', returnRunOutWaypointLabelsBySortie);
    assignin('base', 'preClimbWaypointsNEDBySortie', preClimbWaypointsNEDBySortie);
    assignin('base', 'preClimbWaypointsENUBySortie', preClimbWaypointsENUBySortie);
    assignin('base', 'preClimbWaypointTimeBySortie_s', preClimbWaypointTimeBySortie_s);
    assignin('base', 'preClimbWaypointDistanceBySortie_m', preClimbWaypointDistanceBySortie_m);
    assignin('base', 'preClimbWaypointLabelsBySortie', preClimbWaypointLabelsBySortie);
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
    assignin('base', 'returnRunOutWaypoints', returnRunOutWaypoints);
    assignin('base', 'returnRunOutWaypointsENUIndexedBySortie', returnRunOutWaypointsENU);
    assignin('base', 'returnRunOutWaypointTimes', returnRunOutWaypointTimes);
    assignin('base', 'returnRunOutWaypointDistances', returnRunOutWaypointDistances);
    assignin('base', 'returnRunOutWaypointLabelsIndexedBySortie', returnRunOutWaypointLabels);
    assignin('base', 'preClimbWaypoints', preClimbWaypoints);
    assignin('base', 'preClimbWaypointsENUIndexedBySortie', preClimbWaypointsENU);
    assignin('base', 'preClimbWaypointTimes', preClimbWaypointTimes);
    assignin('base', 'preClimbWaypointDistances', preClimbWaypointDistances);
    assignin('base', 'preClimbWaypointLabelsIndexedBySortie', preClimbWaypointLabels);
    assignin('base', 'sortieWaypointExport', exportStruct);
end

function outStr = logicalToYesNo(valueIn)
    if valueIn
        outStr = 'yes';
    else
        outStr = 'no';
    end
end

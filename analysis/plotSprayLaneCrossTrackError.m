%% plotSprayLaneCrossTrackError
% Analyse spray-lane cross-track and altitude error for one selected sortie
% by comparing the actual NED trajectory against the exported spray boundary
% waypoints. Altitude error is positive when the aircraft is above the
% planned spray-lane altitude.
%
% Expected workflow:
% 1. Run runMission.m so missionCfg,
%    planningData, and simulationData exist in the workspace.
% 2. Run plotSortieWaypoints.m so spraywaypoints{sortie} exists.
% 3. Run your Simulink sortie model so that either:
%       out.Position.Data / out.Position.Time
%    or:
%       pos / t
%    are available in the workspace.
% 4. Run this script.

%% -------------------- USER SETTINGS --------------------
if ~exist('selectedSortie', 'var') || isempty(selectedSortie)
    selectedSortie = 1;
end
actualTimeOffset_s = 0.0;
actualPositionFrame = 'NED';
crossTrackTarget_m = 0.6;
altitudeTarget_m = 0.7;
steadyStateIgnore_s = 5.0;

%% -------------------- LOAD REQUIRED DATA --------------------
assert(exist('missionCfg', 'var') == 1, ...
    'missionCfg was not found. Run runMission.m first.');
assert(exist('planningData', 'var') == 1, ...
    'planningData was not found. Run runMission.m first.');
assert(exist('simulationData', 'var') == 1, ...
    'simulationData was not found. Run runMission.m first.');
assert(exist('spraywaypoints', 'var') == 1, ...
    ['spraywaypoints was not found. Run plotSortieWaypoints.m first so the spray lane ' ...
     'boundary waypoints are exported.']);

validateMissionCfg(missionCfg);
validatePlanningData(planningData);
validateSimulationData(simulationData);

assert(iscell(spraywaypoints), 'spraywaypoints must be a cell array indexed by sortie number.');
assert(selectedSortie >= 1 && selectedSortie <= numel(spraywaypoints) && ...
    ~isempty(spraywaypoints{selectedSortie}), ...
    'spraywaypoints{%d} was not found or is empty.', selectedSortie);

[actualPosRaw, actualTime_s] = resolveActualTrajectoryFromWorkspace();
actualTime_s = actualTime_s(:) - actualTimeOffset_s;
actualPosNED = convertActualPositionToNED(actualPosRaw(:,1:3), actualPositionFrame);

validActual = all(isfinite(actualPosNED), 2) & isfinite(actualTime_s);
actualPosNED = actualPosNED(validActual,:);
actualTime_s = actualTime_s(validActual);

assert(size(actualPosNED, 1) >= 2, 'At least two valid actual trajectory samples are required.');

sprayWaypointNED = spraywaypoints{selectedSortie};
assert(size(sprayWaypointNED, 2) >= 3, 'spraywaypoints{%d} must have at least 3 columns [N E D].', selectedSortie);
assert(mod(size(sprayWaypointNED, 1), 2) == 0, ...
    'spraywaypoints{%d} must contain start/end pairs for each spray lane.', selectedSortie);

laneLabels = cell(0,1);
if exist('sprayBoundaryWaypointLabelsIndexedBySortie', 'var') == 1 && ...
        iscell(sprayBoundaryWaypointLabelsIndexedBySortie) && ...
        selectedSortie <= numel(sprayBoundaryWaypointLabelsIndexedBySortie) && ...
        ~isempty(sprayBoundaryWaypointLabelsIndexedBySortie{selectedSortie})
    laneLabels = sprayBoundaryWaypointLabelsIndexedBySortie{selectedSortie};
end

laneData = buildLaneDefinitions(sprayWaypointNED(:,1:3), laneLabels);
actualVelNED = estimateSampleVelocityNED(actualPosNED, actualTime_s);
laneResults = analyseSequentialSprayLanes(actualPosNED, actualVelNED, actualTime_s, laneData);
sprayWidth_m = resolveSprayWidthForDiagnostics(missionCfg);
laneResults = addPerformanceMetricsToLaneResults(laneResults, crossTrackTarget_m, ...
    altitudeTarget_m, steadyStateIgnore_s, sprayWidth_m);
overallStats = summarizeLaneResults(laneResults);

%% -------------------- FIGURE --------------------
fig = figure('Color', 'w', ...
    'Name', 'Spray Lane Tracking Errors', 'NumberTitle', 'off');

axRoute = axes('Parent', fig);
set(axRoute, 'Position', [0.06 0.12 0.56 0.80]);
hold(axRoute, 'on')
grid(axRoute, 'on')
axis(axRoute, 'equal')
xlabel(axRoute, 'East [m]')
ylabel(axRoute, 'North [m]')
title(axRoute, sprintf('Spray lane tracking over field: sortie %d', selectedSortie))

axError = axes('Parent', fig);
set(axError, 'Position', [0.67 0.58 0.30 0.32]);
hold(axError, 'on')
grid(axError, 'on')
xlabel(axError, 'Time [s]')
ylabel(axError, 'Cross-track error [m]')
title(axError, 'Spray lane cross-track error')

axAltitudeError = axes('Parent', fig);
set(axAltitudeError, 'Position', [0.67 0.12 0.30 0.32]);
hold(axAltitudeError, 'on')
grid(axAltitudeError, 'on')
xlabel(axAltitudeError, 'Time [s]')
ylabel(axAltitudeError, 'Altitude error [m]')
title(axAltitudeError, 'Spray lane altitude error')

fieldFaceColor = [0.60 0.90 0.60];
plannedLaneColor = [0.20 0.20 0.20];
actualFallbackColor = [0.55 0.55 0.55];
laneColors = lines(max(numel(laneResults), 1));

hFieldLegend = patch(axRoute, 'XData', NaN, 'YData', NaN, ...
    'FaceColor', fieldFaceColor, 'EdgeColor', 'k', 'LineWidth', 1.5, ...
    'FaceAlpha', 0.22, 'DisplayName', 'Fields');
hPlannedLaneLegend = plot(axRoute, NaN, NaN, '--', 'Color', plannedLaneColor, ...
    'LineWidth', 1.8, 'DisplayName', 'Planned spray lane');
hActualLegend = plot(axRoute, NaN, NaN, '-', 'Color', laneColors(1,:), ...
    'LineWidth', 2.0, 'DisplayName', 'Actual over-field segment');
hAttemptLegend = plot(axRoute, NaN, NaN, '--', 'Color', actualFallbackColor, ...
    'LineWidth', 1.6, 'DisplayName', 'Attempted / incomplete segment');
hStartLegend = plot(axRoute, NaN, NaN, '^', 'Color', [0.00 0.60 0.20], ...
    'MarkerFaceColor', [0.55 0.95 0.55], 'MarkerSize', 8.0, ...
    'LineWidth', 1.2, 'DisplayName', 'Spray lane start');
hEndLegend = plot(axRoute, NaN, NaN, 's', 'Color', [0.80 0.10 0.10], ...
    'MarkerFaceColor', [1.00 0.75 0.75], 'MarkerSize', 7.5, ...
    'LineWidth', 1.2, 'DisplayName', 'Spray lane end');

drawMissionContext2D(axRoute, planningData, fieldFaceColor);

crossTrackPlotHandles = gobjects(numel(laneResults), 1);
altitudePlotHandles = gobjects(numel(laneResults), 1);
laneLabelHandles = gobjects(2 * numel(laneResults), 1);
routeCandidatesEast = zeros(0,1);
routeCandidatesNorth = zeros(0,1);

for iLane = 1:numel(laneResults)
    laneResult = laneResults(iLane);
    laneColor = laneColors(iLane,:);

    laneENU = nedToEnu(laneResult.laneWaypointsNED);
    plot(axRoute, laneENU(:,1), laneENU(:,2), '--', 'Color', plannedLaneColor, ...
        'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot(axRoute, laneENU(1,1), laneENU(1,2), '^', ...
        'Color', [0.00 0.60 0.20], 'MarkerFaceColor', [0.55 0.95 0.55], ...
        'MarkerSize', 8.0, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(axRoute, laneENU(2,1), laneENU(2,2), 's', ...
        'Color', [0.80 0.10 0.10], 'MarkerFaceColor', [1.00 0.75 0.75], ...
        'MarkerSize', 7.5, 'LineWidth', 1.2, 'HandleVisibility', 'off');

    laneLabelHandles(2*iLane-1) = text(axRoute, laneENU(1,1), laneENU(1,2), ...
        [' ' laneResult.startLabel], 'FontSize', 8.5, 'FontWeight', 'bold', ...
        'Color', [0.00 0.45 0.15], 'HorizontalAlignment', 'left', 'Clipping', 'on');
    laneLabelHandles(2*iLane) = text(axRoute, laneENU(2,1), laneENU(2,2), ...
        [' ' laneResult.endLabel], 'FontSize', 8.5, 'FontWeight', 'bold', ...
        'Color', [0.70 0.05 0.05], 'HorizontalAlignment', 'left', 'Clipping', 'on');

    actualSectionENU = nedToEnu(laneResult.sectionPolylineNED);
    if laneResult.hasValidEntry && laneResult.hasValidExit
        routeLineStyle = '-';
        routeColor = laneColor;
    else
        routeLineStyle = '--';
        routeColor = actualFallbackColor;
    end
    plot(axRoute, actualSectionENU(:,1), actualSectionENU(:,2), routeLineStyle, ...
        'Color', routeColor, 'LineWidth', 2.0, 'HandleVisibility', 'off');

    crossTrackPlotHandles(iLane) = plot(axError, laneResult.sectionTime_s, ...
        laneResult.crossTrackSigned_m, '-', 'Color', laneColor, 'LineWidth', 1.6, ...
        'DisplayName', sprintf('Lane %d', iLane));

    altitudePlotHandles(iLane) = plot(axAltitudeError, laneResult.sectionTime_s, ...
        laneResult.altitudeErrorSigned_m, '-', 'Color', laneColor, 'LineWidth', 1.6, ...
        'DisplayName', sprintf('Lane %d', iLane));

    routeCandidatesEast = [routeCandidatesEast; laneENU(:,1); actualSectionENU(:,1)]; %#ok<AGROW>
    routeCandidatesNorth = [routeCandidatesNorth; laneENU(:,2); actualSectionENU(:,2)]; %#ok<AGROW>
end

legend(axRoute, ...
    [hFieldLegend, hPlannedLaneLegend, hActualLegend, hAttemptLegend, hStartLegend, hEndLegend], ...
    'Location', 'northeast');
legend(axError, crossTrackPlotHandles(ishghandle(crossTrackPlotHandles)), 'Location', 'best')
legend(axAltitudeError, altitudePlotHandles(ishghandle(altitudePlotHandles)), 'Location', 'best')

setRouteAxisLimits2D(axRoute, planningData, routeCandidatesEast, routeCandidatesNorth);
setCrossTrackAxisLimits(axError, overallStats);
setAltitudeAxisLimits(axAltitudeError, overallStats);
yline(axError, 0, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off')
yline(axAltitudeError, 0, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off')
yline(axError, crossTrackTarget_m, 'r--', 'LineWidth', 0.9, 'HandleVisibility', 'off')
yline(axError, -crossTrackTarget_m, 'r--', 'LineWidth', 0.9, 'HandleVisibility', 'off')
yline(axAltitudeError, altitudeTarget_m, 'r--', 'LineWidth', 0.9, 'HandleVisibility', 'off')
yline(axAltitudeError, -altitudeTarget_m, 'r--', 'LineWidth', 0.9, 'HandleVisibility', 'off')

statsLines = buildStatsLines(selectedSortie, laneResults, overallStats);
altitudeStatsLines = buildAltitudeStatsLines(selectedSortie, laneResults, overallStats);

figStats = figure('Color', 'w', ...
    'Name', 'Spray Lane Error Summaries', 'NumberTitle', 'off');
annotation(figStats, 'textbox', [0.03 0.04 0.45 0.92], ...
    'String', strjoin(statsLines, newline), ...
    'FitBoxToText', 'off', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'FontSize', 8.5, 'Interpreter', 'none');
annotation(figStats, 'textbox', [0.52 0.04 0.45 0.92], ...
    'String', strjoin(altitudeStatsLines, newline), ...
    'FitBoxToText', 'off', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'FontSize', 8.5, 'Interpreter', 'none');

plotSprayLaneMetricsDashboard(selectedSortie, laneResults, planningData, ...
    crossTrackTarget_m, altitudeTarget_m, steadyStateIgnore_s, sprayWidth_m);

%% -------------------- LOCAL FUNCTIONS --------------------
function [actualPosRaw, actualTime_s] = resolveActualTrajectoryFromWorkspace()
    if evalin('base', 'exist(''out'', ''var'')') == 1
        outVar = evalin('base', 'out');
        [foundOutSignal, actualPosRaw, actualTime_s] = tryResolvePositionSignal(outVar);
        if foundOutSignal
            actualPosRaw = reshapePositionSamples(actualPosRaw);
            assert(size(actualPosRaw,2) >= 3, 'Actual position data must have at least 3 columns.');
            return
        end
    end

    if evalin('base', 'exist(''pos'', ''var'')') == 1 && ...
            evalin('base', 'exist(''t'', ''var'')') == 1
        actualPosRaw = evalin('base', 'pos');
        actualTime_s = evalin('base', 't');
        actualPosRaw = reshapePositionSamples(actualPosRaw);
    else
        error(['Simulink trajectory not found. Expected one of: out.Position, out.position, ' ...
               'or workspace variables pos and t.']);
    end

    assert(size(actualPosRaw,2) >= 3, 'Actual position data must have at least 3 columns.');
end

function [foundSignal, posData, timeData] = tryResolvePositionSignal(outVar)
    foundSignal = false;
    posData = zeros(0,3);
    timeData = zeros(0,1);

    candidateNames = {'Position', 'position'};
    for iName = 1:numel(candidateNames)
        signalName = candidateNames{iName};

        if isprop(outVar, signalName)
            signalValue = outVar.(signalName);
        elseif isstruct(outVar) && isfield(outVar, signalName)
            signalValue = outVar.(signalName);
        else
            continue
        end

        if isstruct(signalValue) && isfield(signalValue, 'Data') && isfield(signalValue, 'Time')
            posData = signalValue.Data;
            timeData = signalValue.Time;
            foundSignal = true;
            return
        end

        if isobject(signalValue)
            hasDataProp = isprop(signalValue, 'Data');
            hasTimeProp = isprop(signalValue, 'Time');
            if hasDataProp && hasTimeProp
                posData = signalValue.Data;
                timeData = signalValue.Time;
                foundSignal = true;
                return
            end
        end
    end
end

function posSamples = reshapePositionSamples(posRawIn)
    posSamples = squeeze(posRawIn);

    if isvector(posSamples)
        posSamples = posSamples(:).';
    end

    if size(posSamples, 2) < 3 && size(posSamples, 1) >= 3
        posSamples = posSamples.';
    end
end

function posNED = convertActualPositionToNED(posRawIn, frameLabel)
    frameUpper = upper(strtrim(frameLabel));

    switch frameUpper
        case 'NED'
            posNED = posRawIn;
        case 'ENU'
            posNED = [posRawIn(:,2), posRawIn(:,1), -posRawIn(:,3)];
        otherwise
            error('Unsupported actualPositionFrame "%s". Use ''NED'' or ''ENU''.', frameLabel);
    end
end

function laneData = buildLaneDefinitions(sprayWaypointNEDIn, laneLabelsIn)
    laneCount = size(sprayWaypointNEDIn, 1) / 2;
    laneData = repmat(struct( ...
        'laneNumber', NaN, ...
        'startNED', zeros(1,3), ...
        'endNED', zeros(1,3), ...
        'laneWaypointsNED', zeros(2,3), ...
        'startLabel', '', ...
        'endLabel', '', ...
        'length_m', NaN), laneCount, 1);

    for iLane = 1:laneCount
        rowStart = 2 * iLane - 1;
        rowEnd = 2 * iLane;
        laneData(iLane).laneNumber = iLane;
        laneData(iLane).startNED = sprayWaypointNEDIn(rowStart,:);
        laneData(iLane).endNED = sprayWaypointNEDIn(rowEnd,:);
        laneData(iLane).laneWaypointsNED = sprayWaypointNEDIn([rowStart rowEnd],:);
        laneData(iLane).length_m = norm(laneData(iLane).endNED(1:2) - laneData(iLane).startNED(1:2));

        if numel(laneLabelsIn) >= rowEnd
            laneData(iLane).startLabel = laneLabelsIn{rowStart};
            laneData(iLane).endLabel = laneLabelsIn{rowEnd};
        else
            laneData(iLane).startLabel = sprintf('S%d start', iLane);
            laneData(iLane).endLabel = sprintf('S%d end', iLane);
        end
    end
end

function laneResults = analyseSequentialSprayLanes(actualPosNEDIn, actualVelNEDIn, actualTimeIn_s, laneDataIn)
    laneCount = numel(laneDataIn);
    laneResults = repmat(makeEmptyLaneResult(), laneCount, 1);
    searchStartIdx = 1;

    for iLane = 1:laneCount
        laneResults(iLane) = analyseSingleSprayLane(actualPosNEDIn, actualVelNEDIn, ...
            actualTimeIn_s, laneDataIn(iLane), searchStartIdx);
        searchStartIdx = min(max(laneResults(iLane).nextSearchIdx, searchStartIdx + 1), size(actualPosNEDIn, 1));
    end
end

function laneResult = makeEmptyLaneResult()
    emptyCross = struct('found', false, 'beforeIdx', NaN, 'afterIdx', NaN, ...
        'alpha', NaN, 'posNED', [NaN NaN NaN], 'time_s', NaN);
    laneResult = struct( ...
        'laneNumber', NaN, ...
        'startLabel', '', ...
        'endLabel', '', ...
        'laneWaypointsNED', zeros(2,3), ...
        'entryStatus', '', ...
        'exitStatus', '', ...
        'hasValidEntry', false, ...
        'hasValidExit', false, ...
        'entryCross', emptyCross, ...
        'exitCross', emptyCross, ...
        'sectionStartSampleIdx', NaN, ...
        'sectionEndSampleIdx', NaN, ...
        'sectionStartPointNED', [NaN NaN NaN], ...
        'sectionEndPointNED', [NaN NaN NaN], ...
        'sectionTime_s', zeros(0,1), ...
        'sectionPosNED', zeros(0,3), ...
        'sectionPolylineNED', zeros(0,3), ...
        'crossTrackSigned_m', zeros(0,1), ...
        'crossTrackAbs_m', zeros(0,1), ...
        'closestPointNED', zeros(0,3), ...
        'actualAltitude_m', zeros(0,1), ...
        'targetAltitude_m', zeros(0,1), ...
        'altitudeErrorSigned_m', zeros(0,1), ...
        'altitudeErrorAbs_m', zeros(0,1), ...
        'crossTrackRate_mps', zeros(0,1), ...
        'alongTrackRate_mps', zeros(0,1), ...
        'duration_s', NaN, ...
        'sampleCount', 0, ...
        'meanAbsCTE_m', NaN, ...
        'maxAbsCTE_m', NaN, ...
        'rmsCTE_m', NaN, ...
        'cumulativeAbsSampleError_m', NaN, ...
        'meanAbsAltitudeError_m', NaN, ...
        'maxAbsAltitudeError_m', NaN, ...
        'rmsAltitudeError_m', NaN, ...
        'cumulativeAbsAltitudeSampleError_m', NaN, ...
        'nextSearchIdx', NaN, ...
        'statusSummary', '');
end

function laneResult = analyseSingleSprayLane(actualPosNEDIn, actualVelNEDIn, actualTimeIn_s, laneDataIn, searchStartIdx)
    nSamples = size(actualPosNEDIn, 1);
    searchStartIdx = min(max(searchStartIdx, 1), nSamples);

    laneVecNE = laneDataIn.endNED(1:2) - laneDataIn.startNED(1:2);
    laneLength_m = norm(laneVecNE);
    assert(laneLength_m > 1e-9, 'Spray lane %d has zero horizontal length.', laneDataIn.laneNumber);

    laneUnitNE = laneVecNE / laneLength_m;
    alongTrack_m = (actualPosNEDIn(:,1:2) - laneDataIn.startNED(1:2)) * laneUnitNE.';
    inBandMask = alongTrack_m >= 0.0 & alongTrack_m <= laneLength_m;

    entryCross = findFinishLineCrossing(actualPosNEDIn, actualTimeIn_s, alongTrack_m, 0.0, searchStartIdx, 1);
    if entryCross.found
        entryStatus = 'valid';
        sectionStartPointNED = entryCross.posNED;
        sectionStartTime_s = entryCross.time_s;
        sectionStartSampleIdx = entryCross.afterIdx;
    else
        laneResult = makeUntrackedLaneResult(laneDataIn, entryCross, ...
            'no valid start gate crossing', min(searchStartIdx + 1, nSamples));
        return
    end

    endSearchIdx = min(max(sectionStartSampleIdx, searchStartIdx), nSamples);
    exitCross = findFinishLineCrossing(actualPosNEDIn, actualTimeIn_s, alongTrack_m, laneLength_m, endSearchIdx);
    if exitCross.found
        exitStatus = 'valid';
        sectionEndPointNED = exitCross.posNED;
        sectionEndTime_s = exitCross.time_s;
        sectionEndSampleIdx = max(sectionStartSampleIdx, exitCross.beforeIdx);
        nextSearchIdx = exitCross.afterIdx;
    else
        inBandAfterStart = find(inBandMask(endSearchIdx:end), 1, 'last');
        if ~isempty(inBandAfterStart)
            sectionEndSampleIdx = endSearchIdx + inBandAfterStart - 1;
            sectionEndPointNED = actualPosNEDIn(sectionEndSampleIdx,:);
            sectionEndTime_s = actualTimeIn_s(sectionEndSampleIdx);
            exitStatus = 'no exit crossing';
            nextSearchIdx = sectionEndSampleIdx + 1;
        else
            sectionEndSampleIdx = sectionStartSampleIdx;
            sectionEndPointNED = actualPosNEDIn(sectionEndSampleIdx,:);
            sectionEndTime_s = actualTimeIn_s(sectionEndSampleIdx);
            exitStatus = 'attempted only';
            nextSearchIdx = sectionEndSampleIdx + 1;
        end
    end

    sectionEndSampleIdx = min(max(sectionEndSampleIdx, sectionStartSampleIdx), nSamples);
    nextSearchIdx = min(max(nextSearchIdx, sectionEndSampleIdx + 1), nSamples);

    sectionSampleIdx = sectionStartSampleIdx:sectionEndSampleIdx;
    if isempty(sectionSampleIdx)
        sectionSampleIdx = sectionStartSampleIdx;
    end

    laneMetrics = computeLaneCrossTrackMetrics( ...
        actualPosNEDIn(sectionSampleIdx,:), actualVelNEDIn(sectionSampleIdx,:), ...
        laneDataIn.laneWaypointsNED);

    laneResult = struct();
    laneResult.laneNumber = laneDataIn.laneNumber;
    laneResult.startLabel = laneDataIn.startLabel;
    laneResult.endLabel = laneDataIn.endLabel;
    laneResult.laneWaypointsNED = laneDataIn.laneWaypointsNED;
    laneResult.entryStatus = entryStatus;
    laneResult.exitStatus = exitStatus;
    laneResult.hasValidEntry = strcmp(entryStatus, 'valid');
    laneResult.hasValidExit = strcmp(exitStatus, 'valid');
    laneResult.entryCross = entryCross;
    laneResult.exitCross = exitCross;
    laneResult.sectionStartSampleIdx = sectionStartSampleIdx;
    laneResult.sectionEndSampleIdx = sectionEndSampleIdx;
    laneResult.sectionStartPointNED = sectionStartPointNED;
    laneResult.sectionEndPointNED = sectionEndPointNED;
    laneResult.sectionTime_s = actualTimeIn_s(sectionSampleIdx);
    laneResult.sectionPosNED = actualPosNEDIn(sectionSampleIdx,:);
    laneResult.sectionPolylineNED = buildSectionPolylineNED( ...
        sectionStartPointNED, actualPosNEDIn(sectionSampleIdx,:), sectionEndPointNED);
    laneResult.crossTrackSigned_m = laneMetrics.crossTrackSigned_m;
    laneResult.crossTrackAbs_m = laneMetrics.crossTrackAbs_m;
    laneResult.closestPointNED = laneMetrics.closestPointNED;
    laneResult.actualAltitude_m = laneMetrics.actualAltitude_m;
    laneResult.targetAltitude_m = laneMetrics.targetAltitude_m;
    laneResult.altitudeErrorSigned_m = laneMetrics.altitudeErrorSigned_m;
    laneResult.altitudeErrorAbs_m = laneMetrics.altitudeErrorAbs_m;
    laneResult.crossTrackRate_mps = laneMetrics.crossTrackRate_mps;
    laneResult.alongTrackRate_mps = laneMetrics.alongTrackRate_mps;
    laneResult.duration_s = max(sectionEndTime_s - sectionStartTime_s, 0.0);
    laneResult.sampleCount = numel(sectionSampleIdx);
    laneResult.meanAbsCTE_m = mean(laneResult.crossTrackAbs_m, 'omitnan');
    laneResult.maxAbsCTE_m = maxOrNaN(laneResult.crossTrackAbs_m);
    laneResult.rmsCTE_m = sqrt(mean(laneResult.crossTrackSigned_m.^2, 'omitnan'));
    laneResult.cumulativeAbsSampleError_m = sum(laneResult.crossTrackAbs_m, 'omitnan');
    laneResult.meanAbsAltitudeError_m = mean(laneResult.altitudeErrorAbs_m, 'omitnan');
    laneResult.maxAbsAltitudeError_m = maxOrNaN(laneResult.altitudeErrorAbs_m);
    laneResult.rmsAltitudeError_m = sqrt(mean(laneResult.altitudeErrorSigned_m.^2, 'omitnan'));
    laneResult.cumulativeAbsAltitudeSampleError_m = sum(laneResult.altitudeErrorAbs_m, 'omitnan');
    laneResult.nextSearchIdx = nextSearchIdx;
    laneResult.statusSummary = buildLaneStatusSummary(entryStatus, exitStatus);
end

function laneResult = makeUntrackedLaneResult(laneDataIn, entryCross, entryStatus, nextSearchIdx)
    laneResult = makeEmptyLaneResult();
    laneResult.laneNumber = laneDataIn.laneNumber;
    laneResult.startLabel = laneDataIn.startLabel;
    laneResult.endLabel = laneDataIn.endLabel;
    laneResult.laneWaypointsNED = laneDataIn.laneWaypointsNED;
    laneResult.entryStatus = entryStatus;
    laneResult.exitStatus = 'skipped';
    laneResult.entryCross = entryCross;
    laneResult.nextSearchIdx = nextSearchIdx;
    laneResult.statusSummary = buildLaneStatusSummary(laneResult.entryStatus, laneResult.exitStatus);
end

function cross = findFinishLineCrossing(actualPosNEDIn, actualTimeIn_s, alongTrack_m, threshold_m, startIdx, crossingDirection)
    cross = struct('found', false, 'beforeIdx', NaN, 'afterIdx', NaN, ...
        'alpha', NaN, 'posNED', [NaN NaN NaN], 'time_s', NaN);

    if nargin < 6
        crossingDirection = 0;
    end

    nSamples = size(actualPosNEDIn, 1);
    if startIdx >= nSamples
        return
    end

    tol_m = 1e-9;
    for iSample = startIdx:(nSamples - 1)
        d0 = alongTrack_m(iSample) - threshold_m;
        d1 = alongTrack_m(iSample + 1) - threshold_m;

        if ~isfinite(d0) || ~isfinite(d1)
            continue
        end

        trackDelta_m = d1 - d0;
        if crossingDirection > 0
            if d0 > tol_m || d1 < -tol_m || trackDelta_m <= tol_m
                continue
            end
        elseif crossingDirection < 0
            if d0 < -tol_m || d1 > tol_m || trackDelta_m >= -tol_m
                continue
            end
        elseif ~(abs(d0) <= tol_m || abs(d1) <= tol_m || d0 * d1 < 0.0)
            continue
        end

        if abs(d0) <= tol_m
            alpha = 0.0;
        elseif abs(d1) <= tol_m
            alpha = 1.0;
        else
            alpha = d0 / (d0 - d1);
        end

        alpha = min(max(alpha, 0.0), 1.0);
        cross.found = true;
        cross.beforeIdx = iSample;
        cross.afterIdx = iSample + 1;
        cross.alpha = alpha;
        cross.posNED = actualPosNEDIn(iSample,:) + alpha * (actualPosNEDIn(iSample + 1,:) - actualPosNEDIn(iSample,:));
        cross.time_s = actualTimeIn_s(iSample) + alpha * (actualTimeIn_s(iSample + 1) - actualTimeIn_s(iSample));
        return
    end
end

function bestIdx = findBestAttemptSample(actualPosNEDIn, laneDataIn, startIdx)
    remainingPosNED = actualPosNEDIn(startIdx:end,:);
    distanceToLane_m = distanceToFiniteLane(remainingPosNED, laneDataIn.laneWaypointsNED);
    [~, localIdx] = min(distanceToLane_m);
    bestIdx = startIdx + localIdx - 1;
end

function distance_m = distanceToFiniteLane(posNEDIn, laneWaypointsNEDIn)
    wpA = laneWaypointsNEDIn(1,:);
    wpB = laneWaypointsNEDIn(2,:);
    AB = wpB - wpA;
    AB_NE = AB(1:2);
    segLenSq = AB_NE * AB_NE.';

    if segLenSq <= 1e-12
        delta = posNEDIn - wpA;
        distance_m = vecnorm(delta, 2, 2);
        return
    end

    AP_NE = posNEDIn(:,1:2) - wpA(1:2);
    sRaw = (AP_NE * AB_NE.') / segLenSq;
    sClamped = min(max(sRaw, 0.0), 1.0);
    closestNED = wpA + sClamped * AB;
    distance_m = vecnorm(posNEDIn - closestNED, 2, 2);
end

function laneMetrics = computeLaneCrossTrackMetrics(posNEDIn, velNEDIn, laneWaypointsNEDIn)
    sampleCount = size(posNEDIn, 1);
    laneMetrics = struct();
    laneMetrics.crossTrackSigned_m = zeros(sampleCount,1);
    laneMetrics.crossTrackAbs_m = zeros(sampleCount,1);
    laneMetrics.crossTrackRate_mps = zeros(sampleCount,1);
    laneMetrics.alongTrackRate_mps = zeros(sampleCount,1);
    laneMetrics.closestPointNED = zeros(sampleCount,3);
    laneMetrics.actualAltitude_m = zeros(sampleCount,1);
    laneMetrics.targetAltitude_m = zeros(sampleCount,1);
    laneMetrics.altitudeErrorSigned_m = zeros(sampleCount,1);
    laneMetrics.altitudeErrorAbs_m = zeros(sampleCount,1);

    for iSample = 1:sampleCount
        [laneMetrics.crossTrackSigned_m(iSample), ...
            laneMetrics.crossTrackRate_mps(iSample), ...
            laneMetrics.alongTrackRate_mps(iSample), ...
            ~, ...
            ~, ...
            ~, ...
            laneMetrics.closestPointNED(iSample,:)] = ...
            crossTrackErrorABNED(posNEDIn(iSample,:), velNEDIn(iSample,:), ...
            laneWaypointsNEDIn, 1, 0.0);
    end

    laneMetrics.crossTrackAbs_m = abs(laneMetrics.crossTrackSigned_m);
    laneMetrics.actualAltitude_m = -posNEDIn(:,3);
    laneMetrics.targetAltitude_m = -laneMetrics.closestPointNED(:,3);
    laneMetrics.altitudeErrorSigned_m = ...
        laneMetrics.actualAltitude_m - laneMetrics.targetAltitude_m;
    laneMetrics.altitudeErrorAbs_m = abs(laneMetrics.altitudeErrorSigned_m);
end

function sprayWidth_m = resolveSprayWidthForDiagnostics(missionCfgIn)
    if isstruct(missionCfgIn) && isfield(missionCfgIn, 'sprayWidth') && ...
            isfinite(missionCfgIn.sprayWidth) && missionCfgIn.sprayWidth > 0
        sprayWidth_m = missionCfgIn.sprayWidth;
    else
        sprayWidth_m = NaN;
    end
end

function laneResultsOut = addPerformanceMetricsToLaneResults(laneResultsIn, ...
        crossTrackTarget_m, altitudeTarget_m, steadyStateIgnore_s, sprayWidth_m)
    laneResultsOut = laneResultsIn;

    for iLane = 1:numel(laneResultsOut)
        lane = laneResultsOut(iLane);
        t_s = lane.sectionTime_s(:);
        ctSigned_m = lane.crossTrackSigned_m(:);
        altSigned_m = lane.altitudeErrorSigned_m(:);
        ctAbs_m = abs(ctSigned_m);
        altAbs_m = abs(altSigned_m);
        combinedAbs_m = hypot(ctSigned_m, altSigned_m);

        inCrossTrackTarget = ctAbs_m <= crossTrackTarget_m;
        inAltitudeTarget = altAbs_m <= altitudeTarget_m;
        inCombinedTarget = inCrossTrackTarget & inAltitudeTarget;

        laneResultsOut(iLane).p95AbsCTE_m = percentileFinite(ctAbs_m, 95);
        laneResultsOut(iLane).p95AbsAltitudeError_m = percentileFinite(altAbs_m, 95);
        laneResultsOut(iLane).meanSignedCTE_m = meanOrNaN(ctSigned_m);
        laneResultsOut(iLane).meanSignedAltitudeError_m = meanOrNaN(altSigned_m);
        laneResultsOut(iLane).stdCTE_m = stdOrNaN(ctSigned_m);
        laneResultsOut(iLane).stdAltitudeError_m = stdOrNaN(altSigned_m);

        laneResultsOut(iLane).combinedErrorAbs_m = combinedAbs_m;
        laneResultsOut(iLane).meanAbsCombinedError_m = meanOrNaN(combinedAbs_m);
        laneResultsOut(iLane).rmsCombinedError_m = sqrt(meanOrNaN(combinedAbs_m.^2));
        laneResultsOut(iLane).maxAbsCombinedError_m = maxOrNaN(combinedAbs_m);
        laneResultsOut(iLane).p95AbsCombinedError_m = percentileFinite(combinedAbs_m, 95);

        laneResultsOut(iLane).crossTrackTimeInTarget_pct = timeInMaskPct(t_s, inCrossTrackTarget);
        laneResultsOut(iLane).altitudeTimeInTarget_pct = timeInMaskPct(t_s, inAltitudeTarget);
        laneResultsOut(iLane).combinedTimeInTarget_pct = timeInMaskPct(t_s, inCombinedTarget);

        laneResultsOut(iLane).settlingTimeCTE_s = computeSettlingTime(t_s, inCrossTrackTarget);
        laneResultsOut(iLane).settlingTimeAltitude_s = computeSettlingTime(t_s, inAltitudeTarget);
        laneResultsOut(iLane).settlingTimeCombined_s = computeSettlingTime(t_s, inCombinedTarget);

        if isempty(t_s)
            steadyMask = false(size(t_s));
        else
            steadyMask = isfinite(t_s) & t_s >= t_s(1) + steadyStateIgnore_s;
        end
        laneResultsOut(iLane).steadyMeanAbsCTE_m = meanOrNaN(ctAbs_m(steadyMask));
        laneResultsOut(iLane).steadyMeanAbsAltitudeError_m = meanOrNaN(altAbs_m(steadyMask));
        laneResultsOut(iLane).steadyRmsCTE_m = sqrt(meanOrNaN(ctSigned_m(steadyMask).^2));
        laneResultsOut(iLane).steadyRmsAltitudeError_m = sqrt(meanOrNaN(altSigned_m(steadyMask).^2));

        laneResultsOut(iLane).coverageQuality_pct = laneResultsOut(iLane).combinedTimeInTarget_pct;
        laneResultsOut(iLane).crossTrackTargetExceedanceP95_m = ...
            max(laneResultsOut(iLane).p95AbsCTE_m - crossTrackTarget_m, 0.0);
        laneResultsOut(iLane).altitudeTargetExceedanceP95_m = ...
            max(laneResultsOut(iLane).p95AbsAltitudeError_m - altitudeTarget_m, 0.0);
        laneResultsOut(iLane).meanSwathOffset_m = laneResultsOut(iLane).meanSignedCTE_m;
        laneResultsOut(iLane).meanAbsSwathOffset_pct = swathOffsetPct( ...
            meanOrNaN(ctAbs_m), sprayWidth_m);
        laneResultsOut(iLane).maxAbsSwathOffset_pct = swathOffsetPct( ...
            laneResultsOut(iLane).maxAbsCTE_m, sprayWidth_m);
    end
end

function plotSprayLaneMetricsDashboard(sortieNumber, laneResultsIn, planningDataIn, ...
        crossTrackTarget_m, altitudeTarget_m, steadyStateIgnore_s, sprayWidth_m)
    if isempty(laneResultsIn)
        return
    end

    laneNumbers = 1:numel(laneResultsIn);
    laneLabels = arrayfun(@(idx) sprintf('L%d', idx), laneNumbers, 'UniformOutput', false);
    laneColors = lines(max(numel(laneResultsIn), 1));

    figMetrics = figure('Color', 'w', ...
        'Name', 'Spray Lane Metrics Dashboard', 'NumberTitle', 'off');
    tiledlayout(figMetrics, 5, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    axCT = nexttile;
    hCTBars = bar(axCT, laneNumbers, [metricVector(laneResultsIn, 'rmsCTE_m'), ...
        metricVector(laneResultsIn, 'maxAbsCTE_m'), ...
        metricVector(laneResultsIn, 'p95AbsCTE_m')], 'grouped');
    hold(axCT, 'on')
    yline(axCT, crossTrackTarget_m, 'r--', 'Target', 'LineWidth', 1.0);
    title(axCT, 'Cross-track error')
    ylabel(axCT, 'Error [m]')
    setLaneAxis(axCT, laneNumbers, laneLabels)
    legend(axCT, hCTBars, {'RMSE', 'Max', 'P95'}, 'Location', 'best')
    grid(axCT, 'on')

    axAlt = nexttile;
    hAltBars = bar(axAlt, laneNumbers, [metricVector(laneResultsIn, 'rmsAltitudeError_m'), ...
        metricVector(laneResultsIn, 'maxAbsAltitudeError_m'), ...
        metricVector(laneResultsIn, 'p95AbsAltitudeError_m')], 'grouped');
    hold(axAlt, 'on')
    yline(axAlt, altitudeTarget_m, 'r--', 'Target', 'LineWidth', 1.0);
    title(axAlt, 'Altitude error')
    ylabel(axAlt, 'Error [m]')
    setLaneAxis(axAlt, laneNumbers, laneLabels)
    legend(axAlt, hAltBars, {'RMSE', 'Max', 'P95'}, 'Location', 'best')
    grid(axAlt, 'on')

    axNormalised = nexttile;
    hNormalisedBars = bar(axNormalised, laneNumbers, [ ...
        metricVector(laneResultsIn, 'rmsCTE_m') ./ crossTrackTarget_m, ...
        metricVector(laneResultsIn, 'p95AbsCTE_m') ./ crossTrackTarget_m, ...
        metricVector(laneResultsIn, 'rmsAltitudeError_m') ./ altitudeTarget_m, ...
        metricVector(laneResultsIn, 'p95AbsAltitudeError_m') ./ altitudeTarget_m], 'grouped');
    hold(axNormalised, 'on')
    yline(axNormalised, 1.0, 'k--', 'Target ratio = 1', 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
    title(axNormalised, 'Target-normalised error')
    ylabel(axNormalised, 'Error / target [-]')
    setLaneAxis(axNormalised, laneNumbers, laneLabels)
    legend(axNormalised, hNormalisedBars, {'CT RMSE', 'CT P95', 'Alt RMSE', 'Alt P95'}, ...
        'Location', 'best')
    grid(axNormalised, 'on')

    axTolerance = nexttile;
    hToleranceBars = bar(axTolerance, laneNumbers, [metricVector(laneResultsIn, 'crossTrackTimeInTarget_pct'), ...
        metricVector(laneResultsIn, 'altitudeTimeInTarget_pct'), ...
        metricVector(laneResultsIn, 'combinedTimeInTarget_pct')], 'grouped');
    hold(axTolerance, 'on')
    yline(axTolerance, 95, 'k--', '95%', 'LineWidth', 1.0);
    ylim(axTolerance, [0 100])
    title(axTolerance, 'Time within target')
    ylabel(axTolerance, 'Time [%]')
    setLaneAxis(axTolerance, laneNumbers, laneLabels)
    legend(axTolerance, hToleranceBars, {'CT', 'Altitude', 'Both'}, 'Location', 'southoutside', ...
        'Orientation', 'horizontal')
    grid(axTolerance, 'on')

    axSettling = nexttile;
    hSettlingBars = bar(axSettling, laneNumbers, [metricVector(laneResultsIn, 'settlingTimeCTE_s'), ...
        metricVector(laneResultsIn, 'settlingTimeAltitude_s'), ...
        metricVector(laneResultsIn, 'settlingTimeCombined_s')], 'grouped');
    title(axSettling, 'Settling time after lane entry')
    ylabel(axSettling, 'Time [s]')
    setLaneAxis(axSettling, laneNumbers, laneLabels)
    legend(axSettling, hSettlingBars, {'CT', 'Altitude', 'Both'}, 'Location', 'best')
    grid(axSettling, 'on')

    axBias = nexttile;
    hBiasBars = bar(axBias, laneNumbers, [metricVector(laneResultsIn, 'meanSignedCTE_m'), ...
        metricVector(laneResultsIn, 'stdCTE_m'), ...
        metricVector(laneResultsIn, 'meanSignedAltitudeError_m'), ...
        metricVector(laneResultsIn, 'stdAltitudeError_m')], 'grouped');
    title(axBias, 'Bias and oscillation')
    ylabel(axBias, 'Error [m]')
    setLaneAxis(axBias, laneNumbers, laneLabels)
    legend(axBias, hBiasBars, {'CT bias', 'CT std', 'Alt bias', 'Alt std'}, ...
        'Location', 'southoutside', 'Orientation', 'horizontal')
    grid(axBias, 'on')

    axSteady = nexttile;
    hSteadyBars = bar(axSteady, laneNumbers, [metricVector(laneResultsIn, 'steadyMeanAbsCTE_m'), ...
        metricVector(laneResultsIn, 'steadyMeanAbsAltitudeError_m')], 'grouped');
    hold(axSteady, 'on')
    yline(axSteady, crossTrackTarget_m, 'b--', 'CT target', 'LineWidth', 0.9);
    yline(axSteady, altitudeTarget_m, 'r--', 'Alt target', 'LineWidth', 0.9);
    title(axSteady, 'Steady-state mean absolute error')
    ylabel(axSteady, 'Error [m]')
    setLaneAxis(axSteady, laneNumbers, laneLabels)
    legend(axSteady, hSteadyBars, {'CT', 'Altitude'}, 'Location', 'best')
    grid(axSteady, 'on')

    axExceed = nexttile;
    hExceedBars = bar(axExceed, laneNumbers, [metricVector(laneResultsIn, 'crossTrackTargetExceedanceP95_m'), ...
        metricVector(laneResultsIn, 'altitudeTargetExceedanceP95_m')], 'grouped');
    title(axExceed, 'P95 target exceedance')
    ylabel(axExceed, 'Exceedance [m]')
    setLaneAxis(axExceed, laneNumbers, laneLabels)
    legend(axExceed, hExceedBars, {'CT', 'Altitude'}, 'Location', 'best')
    grid(axExceed, 'on')

    axSwath = nexttile;
    hSwathBars = bar(axSwath, laneNumbers, [metricVector(laneResultsIn, 'meanAbsSwathOffset_pct'), ...
        metricVector(laneResultsIn, 'maxAbsSwathOffset_pct')], 'grouped');
    title(axSwath, 'Swath centre offset')
    ylabel(axSwath, 'Half-width offset [%]')
    setLaneAxis(axSwath, laneNumbers, laneLabels)
    legend(axSwath, hSwathBars, {'Mean |CT|', 'Max |CT|'}, 'Location', 'best')
    grid(axSwath, 'on')

    axNormCT = nexttile;
    hold(axNormCT, 'on')
    for iLane = 1:numel(laneResultsIn)
        if isempty(laneResultsIn(iLane).sectionTime_s)
            continue
        end
        tLocal_s = laneResultsIn(iLane).sectionTime_s - laneResultsIn(iLane).sectionTime_s(1);
        plot(axNormCT, tLocal_s, laneResultsIn(iLane).crossTrackSigned_m, ...
            'Color', laneColors(iLane,:), 'LineWidth', 1.2);
    end
    yline(axNormCT, crossTrackTarget_m, 'r--', 'LineWidth', 0.9);
    yline(axNormCT, -crossTrackTarget_m, 'r--', 'LineWidth', 0.9);
    title(axNormCT, 'Lane-normalised cross-track')
    xlabel(axNormCT, 'Time from lane entry [s]')
    ylabel(axNormCT, 'Error [m]')
    grid(axNormCT, 'on')

    axNormAlt = nexttile;
    hold(axNormAlt, 'on')
    for iLane = 1:numel(laneResultsIn)
        if isempty(laneResultsIn(iLane).sectionTime_s)
            continue
        end
        tLocal_s = laneResultsIn(iLane).sectionTime_s - laneResultsIn(iLane).sectionTime_s(1);
        plot(axNormAlt, tLocal_s, laneResultsIn(iLane).altitudeErrorSigned_m, ...
            'Color', laneColors(iLane,:), 'LineWidth', 1.2);
    end
    yline(axNormAlt, altitudeTarget_m, 'r--', 'LineWidth', 0.9);
    yline(axNormAlt, -altitudeTarget_m, 'r--', 'LineWidth', 0.9);
    title(axNormAlt, 'Lane-normalised altitude')
    xlabel(axNormAlt, 'Time from lane entry [s]')
    ylabel(axNormAlt, 'Error [m]')
    grid(axNormAlt, 'on')

    axHist = nexttile;
    ctAbsAll_m = vertcat(laneResultsIn.crossTrackAbs_m);
    altAbsAll_m = vertcat(laneResultsIn.altitudeErrorAbs_m);
    histogram(axHist, ctAbsAll_m(isfinite(ctAbsAll_m)), 'Normalization', 'probability', ...
        'FaceAlpha', 0.55, 'DisplayName', '|CT|');
    hold(axHist, 'on')
    histogram(axHist, altAbsAll_m(isfinite(altAbsAll_m)), 'Normalization', 'probability', ...
        'FaceAlpha', 0.55, 'DisplayName', '|Altitude|');
    xline(axHist, crossTrackTarget_m, 'b--', 'CT target', 'LineWidth', 1.0);
    xline(axHist, altitudeTarget_m, 'r--', 'Alt target', 'LineWidth', 1.0);
    title(axHist, 'Error distribution')
    xlabel(axHist, 'Absolute error [m]')
    ylabel(axHist, 'Sample fraction')
    legend(axHist, 'Location', 'best')
    grid(axHist, 'on')

    axCdf = nexttile;
    plotEmpiricalCdf(axCdf, ctAbsAll_m, [0.00 0.45 0.74], '|CT|');
    hold(axCdf, 'on')
    plotEmpiricalCdf(axCdf, altAbsAll_m, [0.85 0.33 0.10], '|Altitude|');
    xline(axCdf, crossTrackTarget_m, 'b--', 'CT target', 'LineWidth', 1.0);
    xline(axCdf, altitudeTarget_m, 'r--', 'Alt target', 'LineWidth', 1.0);
    title(axCdf, 'Error CDF')
    xlabel(axCdf, 'Absolute error [m]')
    ylabel(axCdf, 'Fraction <= error')
    legend(axCdf, 'Location', 'southeast')
    grid(axCdf, 'on')

    axField = nexttile;
    plotPathColoredByCrossTrack(axField, laneResultsIn, planningDataIn, crossTrackTarget_m);

    axTraffic = nexttile;
    drawTrafficLightSummary(axTraffic, sortieNumber, laneResultsIn, crossTrackTarget_m, ...
        altitudeTarget_m, steadyStateIgnore_s, sprayWidth_m);
end

function setLaneAxis(axHandle, laneNumbers, laneLabels)
    xlim(axHandle, [min(laneNumbers) - 0.6, max(laneNumbers) + 0.6])
    xticks(axHandle, laneNumbers)
    xticklabels(axHandle, laneLabels)
end

function plotEmpiricalCdf(axHandle, valuesIn, colorIn, displayName)
    finiteValues = sort(valuesIn(isfinite(valuesIn)));
    if isempty(finiteValues)
        plot(axHandle, NaN, NaN, 'Color', colorIn, 'DisplayName', displayName);
        return
    end

    yValues = (1:numel(finiteValues)).' / numel(finiteValues);
    plot(axHandle, finiteValues, yValues, 'Color', colorIn, 'LineWidth', 1.8, ...
        'DisplayName', displayName);
end

function plotPathColoredByCrossTrack(axHandle, laneResultsIn, planningDataIn, crossTrackTarget_m)
    hold(axHandle, 'on')
    drawMissionContext2D(axHandle, planningDataIn, [0.80 0.88 0.80]);

    scatterEast = zeros(0,1);
    scatterNorth = zeros(0,1);
    scatterCTE = zeros(0,1);
    plannedEast = zeros(0,1);
    plannedNorth = zeros(0,1);

    for iLane = 1:numel(laneResultsIn)
        laneENU = nedToEnu(laneResultsIn(iLane).laneWaypointsNED);
        plot(axHandle, laneENU(:,1), laneENU(:,2), '-', 'Color', [0.25 0.25 0.25], ...
            'LineWidth', 1.0, 'HandleVisibility', 'off');

        sectionENU = nedToEnu(laneResultsIn(iLane).sectionPosNED);
        scatterEast = [scatterEast; sectionENU(:,1)]; %#ok<AGROW>
        scatterNorth = [scatterNorth; sectionENU(:,2)]; %#ok<AGROW>
        scatterCTE = [scatterCTE; laneResultsIn(iLane).crossTrackSigned_m(:)]; %#ok<AGROW>
        plannedEast = [plannedEast; laneENU(:,1)]; %#ok<AGROW>
        plannedNorth = [plannedNorth; laneENU(:,2)]; %#ok<AGROW>
    end

    if ~isempty(scatterEast)
        scatter(axHandle, scatterEast, scatterNorth, 12, scatterCTE, 'filled', ...
            'MarkerFaceAlpha', 0.75);
        colormap(axHandle, parula)
        colorbar(axHandle)
        cMax = max(abs(scatterCTE(isfinite(scatterCTE))));
        if ~isempty(cMax) && isfinite(cMax) && cMax > 0
            clim(axHandle, [-max(cMax, crossTrackTarget_m), max(cMax, crossTrackTarget_m)])
        end
    end

    axis(axHandle, 'equal')
    title(axHandle, 'Field path coloured by CT error')
    xlabel(axHandle, 'East [m]')
    ylabel(axHandle, 'North [m]')
    grid(axHandle, 'on')
    if ~isempty(plannedEast) && ~isempty(plannedNorth)
        setRouteAxisLimits2D(axHandle, planningDataIn, [plannedEast; scatterEast], ...
            [plannedNorth; scatterNorth]);
    end
end

function drawTrafficLightSummary(axHandle, sortieNumber, laneResultsIn, crossTrackTarget_m, ...
        altitudeTarget_m, steadyStateIgnore_s, sprayWidth_m)
    cla(axHandle)
    axis(axHandle, 'off')
    hold(axHandle, 'on')
    title(axHandle, 'Lane quality summary')

    rowCount = numel(laneResultsIn);
    xlim(axHandle, [0 1])
    ylim(axHandle, [0 rowCount + 2.2])

    header = sprintf('Sortie %d | Targets: CT %.1f m, Alt %.1f m | Steady-state skip %.1f s', ...
        sortieNumber, crossTrackTarget_m, altitudeTarget_m, steadyStateIgnore_s);
    if isfinite(sprayWidth_m)
        header = sprintf('%s | Spray width %.1f m', header, sprayWidth_m);
    end
    text(axHandle, 0.02, rowCount + 1.55, header, 'FontSize', 8.0, ...
        'FontWeight', 'bold', 'Interpreter', 'none');
    text(axHandle, 0.02, rowCount + 0.85, ...
        'Lane   Quality   Both-in-target   CT P95   Alt P95   Reason            Settled', ...
        'FontSize', 8.0, 'FontWeight', 'bold', 'Interpreter', 'none');

    for iLane = 1:rowCount
        lane = laneResultsIn(iLane);
        y = rowCount - iLane + 0.35;
        qualityPct = lane.coverageQuality_pct;
        [qualityLabel, rowColor] = qualityStatus(qualityPct);
        reasonText = laneFailureReasonLabel(lane.crossTrackTimeInTarget_pct, ...
            lane.altitudeTimeInTarget_pct);
        rectangle(axHandle, 'Position', [0.02 y 0.96 0.58], ...
            'FaceColor', rowColor, 'EdgeColor', [0.85 0.85 0.85]);

        settleText = formatStatValue(lane.settlingTimeCombined_s, '%.1f s');
        rowText = sprintf('L%-4d %-8s %6s        %6s  %7s  %-16s %7s', ...
            iLane, qualityLabel, formatStatValue(qualityPct, '%.1f%%'), ...
            formatStatValue(lane.p95AbsCTE_m, '%.2f'), ...
            formatStatValue(lane.p95AbsAltitudeError_m, '%.2f'), ...
            reasonText, settleText);
        text(axHandle, 0.04, y + 0.29, rowText, 'FontName', 'Consolas', ...
            'FontSize', 8.0, 'VerticalAlignment', 'middle', 'Interpreter', 'none');
    end
end

function [labelOut, colorOut] = qualityStatus(qualityPct)
    if ~isfinite(qualityPct)
        labelOut = 'No data';
        colorOut = [0.92 0.92 0.92];
    elseif qualityPct >= 95
        labelOut = 'Green';
        colorOut = [0.78 0.94 0.78];
    elseif qualityPct >= 80
        labelOut = 'Amber';
        colorOut = [1.00 0.91 0.62];
    else
        labelOut = 'Red';
        colorOut = [1.00 0.78 0.78];
    end
end

function reasonText = laneFailureReasonLabel(crossTrackTimeInTarget_pct, altitudeTimeInTarget_pct)
    ctPass = isfinite(crossTrackTimeInTarget_pct) && crossTrackTimeInTarget_pct >= 95;
    altPass = isfinite(altitudeTimeInTarget_pct) && altitudeTimeInTarget_pct >= 95;

    if ctPass && altPass
        reasonText = 'Both pass';
    elseif ~ctPass && altPass
        reasonText = 'CT only';
    elseif ctPass && ~altPass
        reasonText = 'Altitude only';
    else
        reasonText = 'Both fail';
    end
end

function polylineNED = buildSectionPolylineNED(startPointNED, samplePosNED, endPointNED)
    polylineNED = zeros(0,3);
    polylineNED = [polylineNED; startPointNED];

    if ~isempty(samplePosNED)
        if norm(polylineNED(end,:) - samplePosNED(1,:)) > 1e-9
            polylineNED = [polylineNED; samplePosNED];
        else
            polylineNED = [polylineNED; samplePosNED(2:end,:)];
        end
    end

    if isempty(polylineNED) || norm(polylineNED(end,:) - endPointNED) > 1e-9
        polylineNED = [polylineNED; endPointNED];
    end
end

function stats = summarizeLaneResults(laneResultsIn)
    stats = struct();
    stats.totalSampleCount = sum([laneResultsIn.sampleCount]);
    stats.cumulativeAbsSampleError_m = sum([laneResultsIn.cumulativeAbsSampleError_m], 'omitnan');
    stats.cumulativeAbsAltitudeSampleError_m = ...
        sum([laneResultsIn.cumulativeAbsAltitudeSampleError_m], 'omitnan');
    stats.maxAbsCTE_m = maxOrNaN([laneResultsIn.maxAbsCTE_m]);
    stats.maxAbsAltitudeError_m = maxOrNaN([laneResultsIn.maxAbsAltitudeError_m]);
    stats.meanAbsCTE_m = NaN;
    stats.rmsCTE_m = NaN;
    stats.meanAbsAltitudeError_m = NaN;
    stats.rmsAltitudeError_m = NaN;
    stats.timeRange_s = [0 1];
    stats.timeCombined_s = zeros(0,1);
    stats.crossTrackCombined_m = zeros(0,1);
    stats.altitudeErrorCombined_m = zeros(0,1);

    if stats.totalSampleCount <= 0
        return
    end

    combinedAbs = vertcat(laneResultsIn.crossTrackAbs_m);
    combinedSigned = vertcat(laneResultsIn.crossTrackSigned_m);
    combinedAltitudeAbs = vertcat(laneResultsIn.altitudeErrorAbs_m);
    combinedAltitudeSigned = vertcat(laneResultsIn.altitudeErrorSigned_m);
    combinedTime = cell(numel(laneResultsIn), 1);
    combinedCrossTrack = cell(numel(laneResultsIn), 1);
    combinedAltitudeError = cell(numel(laneResultsIn), 1);
    for iLane = 1:numel(laneResultsIn)
        combinedTime{iLane} = [laneResultsIn(iLane).sectionTime_s; NaN];
        combinedCrossTrack{iLane} = [laneResultsIn(iLane).crossTrackSigned_m; NaN];
        combinedAltitudeError{iLane} = [laneResultsIn(iLane).altitudeErrorSigned_m; NaN];
    end

    stats.meanAbsCTE_m = mean(combinedAbs, 'omitnan');
    stats.rmsCTE_m = sqrt(mean(combinedSigned.^2, 'omitnan'));
    stats.meanAbsAltitudeError_m = mean(combinedAltitudeAbs, 'omitnan');
    stats.rmsAltitudeError_m = sqrt(mean(combinedAltitudeSigned.^2, 'omitnan'));
    stats.timeCombined_s = vertcat(combinedTime{:});
    stats.crossTrackCombined_m = vertcat(combinedCrossTrack{:});
    stats.altitudeErrorCombined_m = vertcat(combinedAltitudeError{:});

    finiteCombinedTime_s = stats.timeCombined_s(isfinite(stats.timeCombined_s));
    if ~isempty(finiteCombinedTime_s)
        stats.timeRange_s = [min(finiteCombinedTime_s), max(finiteCombinedTime_s)];
    end
end

function statsLines = buildStatsLines(sortieNumber, laneResultsIn, overallStatsIn)
    statsLines = {
        sprintf('Sortie %d', sortieNumber)
        ''
        sprintf('Spray lane count: %d', numel(laneResultsIn))
        sprintf('Total spray samples used: %d', overallStatsIn.totalSampleCount)
        sprintf('Overall mean |CTE|: %s', formatStatValue(overallStatsIn.meanAbsCTE_m, '%.3f m'))
        sprintf('Overall RMS CTE: %s', formatStatValue(overallStatsIn.rmsCTE_m, '%.3f m'))
        sprintf('Overall max |CTE|: %s', formatStatValue(overallStatsIn.maxAbsCTE_m, '%.3f m'))
        sprintf('Overall cumulative |CTE| sample sum: %s', ...
            formatStatValue(overallStatsIn.cumulativeAbsSampleError_m, '%.3f m'))
        ''
        };

    for iLane = 1:numel(laneResultsIn)
        lane = laneResultsIn(iLane);
        statsLines{end+1} = sprintf('Lane %d (%s)', iLane, lane.statusSummary); %#ok<AGROW>
        statsLines{end+1} = sprintf('  Duration %.2f s | Samples %d | Mean |CTE| %s', ...
            lane.duration_s, lane.sampleCount, formatStatValue(lane.meanAbsCTE_m, '%.3f m')); %#ok<AGROW>
        statsLines{end+1} = sprintf('  Max |CTE| %s | RMS %s | Cum |CTE| %s', ...
            formatStatValue(lane.maxAbsCTE_m, '%.3f m'), ...
            formatStatValue(lane.rmsCTE_m, '%.3f m'), ...
            formatStatValue(lane.cumulativeAbsSampleError_m, '%.3f m')); %#ok<AGROW>
    end
end

function statsLines = buildAltitudeStatsLines(sortieNumber, laneResultsIn, overallStatsIn)
    statsLines = {
        sprintf('Sortie %d', sortieNumber)
        ''
        sprintf('Spray lane count: %d', numel(laneResultsIn))
        sprintf('Total spray samples used: %d', overallStatsIn.totalSampleCount)
        sprintf('Overall mean |altitude error|: %s', ...
            formatStatValue(overallStatsIn.meanAbsAltitudeError_m, '%.3f m'))
        sprintf('Overall RMS altitude error: %s', ...
            formatStatValue(overallStatsIn.rmsAltitudeError_m, '%.3f m'))
        sprintf('Overall max |altitude error|: %s', ...
            formatStatValue(overallStatsIn.maxAbsAltitudeError_m, '%.3f m'))
        sprintf('Overall cumulative |altitude error| sample sum: %s', ...
            formatStatValue(overallStatsIn.cumulativeAbsAltitudeSampleError_m, '%.3f m'))
        ''
        };

    for iLane = 1:numel(laneResultsIn)
        lane = laneResultsIn(iLane);
        statsLines{end+1} = sprintf('Lane %d (%s)', iLane, lane.statusSummary); %#ok<AGROW>
        statsLines{end+1} = sprintf('  Duration %.2f s | Samples %d | Mean |alt err| %s', ...
            lane.duration_s, lane.sampleCount, ...
            formatStatValue(lane.meanAbsAltitudeError_m, '%.3f m')); %#ok<AGROW>
        statsLines{end+1} = sprintf('  Max |alt err| %s | RMS %s | Cum |alt err| %s', ...
            formatStatValue(lane.maxAbsAltitudeError_m, '%.3f m'), ...
            formatStatValue(lane.rmsAltitudeError_m, '%.3f m'), ...
            formatStatValue(lane.cumulativeAbsAltitudeSampleError_m, '%.3f m')); %#ok<AGROW>
    end
end

function outStr = buildLaneStatusSummary(entryStatus, exitStatus)
    if strcmp(entryStatus, 'valid') && strcmp(exitStatus, 'valid')
        outStr = 'valid';
    else
        outStr = sprintf('%s / %s', entryStatus, exitStatus);
    end
end

function outStr = formatStatValue(valueIn, fmt)
    if isempty(valueIn) || ~isfinite(valueIn)
        outStr = 'n/a';
    else
        outStr = sprintf(fmt, valueIn);
    end
end

function valueOut = meanOrNaN(valueIn)
    finiteValues = valueIn(isfinite(valueIn));
    if isempty(finiteValues)
        valueOut = NaN;
    else
        valueOut = mean(finiteValues);
    end
end

function valueOut = stdOrNaN(valueIn)
    finiteValues = valueIn(isfinite(valueIn));
    if isempty(finiteValues)
        valueOut = NaN;
    elseif isscalar(finiteValues)
        valueOut = 0.0;
    else
        valueOut = std(finiteValues);
    end
end

function valueOut = percentileFinite(valueIn, percentileIn)
    finiteValues = sort(valueIn(isfinite(valueIn)));
    if isempty(finiteValues)
        valueOut = NaN;
        return
    end

    if isscalar(finiteValues)
        valueOut = finiteValues(1);
        return
    end

    rankValue = 1 + (percentileIn / 100) * (numel(finiteValues) - 1);
    idxLow = floor(rankValue);
    idxHigh = ceil(rankValue);
    blend = rankValue - idxLow;
    valueOut = finiteValues(idxLow) + blend * (finiteValues(idxHigh) - finiteValues(idxLow));
end

function pctOut = timeInMaskPct(time_s, maskIn)
    if isempty(time_s) || isempty(maskIn)
        pctOut = NaN;
        return
    end

    time_s = time_s(:);
    maskIn = logical(maskIn(:));
    valid = isfinite(time_s) & isfinite(double(maskIn));
    time_s = time_s(valid);
    maskIn = maskIn(valid);

    if isempty(time_s)
        pctOut = NaN;
    elseif isscalar(time_s)
        pctOut = 100.0 * double(maskIn(1));
    else
        dt_s = diff(time_s);
        validIntervals = isfinite(dt_s) & dt_s > 0;
        if ~any(validIntervals)
            pctOut = 100.0 * mean(double(maskIn));
            return
        end

        intervalGood = maskIn(1:end-1) & maskIn(2:end);
        pctOut = 100.0 * sum(dt_s(validIntervals) .* double(intervalGood(validIntervals))) / ...
            sum(dt_s(validIntervals));
    end
end

function settlingTime_s = computeSettlingTime(time_s, maskIn)
    if isempty(time_s) || isempty(maskIn)
        settlingTime_s = NaN;
        return
    end

    time_s = time_s(:);
    maskIn = logical(maskIn(:));
    valid = isfinite(time_s);
    time_s = time_s(valid);
    maskIn = maskIn(valid);

    if isempty(time_s) || ~maskIn(end)
        settlingTime_s = NaN;
        return
    end

    firstSettledIdx = NaN;
    for iSample = 1:numel(maskIn)
        if all(maskIn(iSample:end))
            firstSettledIdx = iSample;
            break
        end
    end

    if isnan(firstSettledIdx)
        settlingTime_s = NaN;
    else
        settlingTime_s = time_s(firstSettledIdx) - time_s(1);
    end
end

function valuesOut = metricVector(laneResultsIn, fieldName)
    valuesOut = NaN(numel(laneResultsIn), 1);
    for iLane = 1:numel(laneResultsIn)
        if isfield(laneResultsIn(iLane), fieldName)
            value = laneResultsIn(iLane).(fieldName);
            if isscalar(value) && isnumeric(value)
                valuesOut(iLane) = value;
            end
        end
    end
end

function pctOut = swathOffsetPct(offset_m, sprayWidth_m)
    if ~isfinite(offset_m) || ~isfinite(sprayWidth_m) || sprayWidth_m <= 0
        pctOut = NaN;
    else
        pctOut = 100.0 * min(abs(offset_m) / max(0.5 * sprayWidth_m, eps), 1.0);
    end
end

function valueOut = maxOrNaN(valueIn)
    if isempty(valueIn) || all(~isfinite(valueIn))
        valueOut = NaN;
    else
        valueOut = max(valueIn, [], 'omitnan');
    end
end

function enu = nedToEnu(ned)
    enu = [ned(:,2), ned(:,1), -ned(:,3)];
end

function drawMissionContext2D(axHandle, planningDataIn, fieldFaceColor)
    for visitIdx = 1:planningDataIn.fieldCount
        fIdx = planningDataIn.fieldVisitOrder(visitIdx);
        displayLoops = planningDataIn.fieldInfo(fIdx).displayBoundaryLoops;

        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy) || size(cxy,1) < 3
                continue
            end
            patch(axHandle, 'XData', cxy(:,1), 'YData', cxy(:,2), ...
                'FaceColor', fieldFaceColor, 'EdgeColor', 'k', 'LineWidth', 1.5, ...
                'FaceAlpha', 0.22, 'HandleVisibility', 'off');
        end

        text(axHandle, planningDataIn.fieldInfo(fIdx).centreXY(1), ...
            planningDataIn.fieldInfo(fIdx).centreXY(2), ...
            [' ' planningDataIn.fieldInfo(fIdx).name], 'FontSize', 9, ...
            'FontWeight', 'bold', 'Color', [0 0 0], 'Clipping', 'on');
    end
end

function setRouteAxisLimits2D(axHandle, planningDataIn, candidateEastIn, candidateNorthIn)
    if isempty(candidateEastIn) || isempty(candidateNorthIn)
        fieldEast = zeros(0,1);
        fieldNorth = zeros(0,1);
        for iField = 1:planningDataIn.fieldCount
            displayLoops = planningDataIn.fieldInfo(iField).displayBoundaryLoops;
            for iLoop = 1:numel(displayLoops)
                cxy = displayLoops{iLoop};
                if isempty(cxy)
                    continue
                end
                fieldEast = [fieldEast; cxy(:,1)]; %#ok<AGROW>
                fieldNorth = [fieldNorth; cxy(:,2)]; %#ok<AGROW>
            end
        end

        eastCandidates = fieldEast;
        northCandidates = fieldNorth;
    else
        routeEast = candidateEastIn(:);
        routeNorth = candidateNorthIn(:);
        routeEast = routeEast(isfinite(routeEast));
        routeNorth = routeNorth(isfinite(routeNorth));

        eastMin = min(routeEast);
        eastMax = max(routeEast);
        northMin = min(routeNorth);
        northMax = max(routeNorth);
        eastRange = max(eastMax - eastMin, 1);
        northRange = max(northMax - northMin, 1);
        contextMarginEast = max(10, 0.20 * eastRange);
        contextMarginNorth = max(10, 0.20 * northRange);

        fieldEast = zeros(0,1);
        fieldNorth = zeros(0,1);
        for iField = 1:planningDataIn.fieldCount
            displayLoops = planningDataIn.fieldInfo(iField).displayBoundaryLoops;
            for iLoop = 1:numel(displayLoops)
                cxy = displayLoops{iLoop};
                if isempty(cxy)
                    continue
                end

                loopEast = cxy(:,1);
                loopNorth = cxy(:,2);
                loopTouchesView = max(loopEast) >= eastMin - contextMarginEast && ...
                    min(loopEast) <= eastMax + contextMarginEast && ...
                    max(loopNorth) >= northMin - contextMarginNorth && ...
                    min(loopNorth) <= northMax + contextMarginNorth;
                if loopTouchesView
                    fieldEast = [fieldEast; loopEast]; %#ok<AGROW>
                    fieldNorth = [fieldNorth; loopNorth]; %#ok<AGROW>
                end
            end
        end

        eastCandidates = [routeEast; fieldEast];
        northCandidates = [routeNorth; fieldNorth];
    end

    if isempty(eastCandidates) || isempty(northCandidates)
        return
    end

    eastRange = max(eastCandidates) - min(eastCandidates);
    northRange = max(northCandidates) - min(northCandidates);
    eastMargin = max(5, 0.08 * max(eastRange, 1));
    northMargin = max(5, 0.08 * max(northRange, 1));

    xlim(axHandle, [min(eastCandidates) - eastMargin, max(eastCandidates) + eastMargin])
    ylim(axHandle, [min(northCandidates) - northMargin, max(northCandidates) + northMargin])
end

function setCrossTrackAxisLimits(axHandle, overallStatsIn)
    if overallStatsIn.totalSampleCount <= 0
        return
    end

    finiteTime = overallStatsIn.timeCombined_s(isfinite(overallStatsIn.timeCombined_s));
    finiteCrossTrack = overallStatsIn.crossTrackCombined_m(isfinite(overallStatsIn.crossTrackCombined_m));
    if isempty(finiteTime) || isempty(finiteCrossTrack)
        return
    end

    timeMin = min(finiteTime);
    timeMax = max(finiteTime);
    timeRange = max(timeMax - timeMin, 1);
    timeMargin = max(0.25, 0.02 * timeRange);
    xlim(axHandle, [timeMin - timeMargin, timeMax + timeMargin]);

    maxAbsCTE_m = max(abs(finiteCrossTrack));
    yLimitAbs_m = roundUpNice(max(maxAbsCTE_m * 1.10, 0.5));
    ylim(axHandle, [-yLimitAbs_m, yLimitAbs_m]);
end

function setAltitudeAxisLimits(axHandle, overallStatsIn)
    if overallStatsIn.totalSampleCount <= 0
        return
    end

    finiteTime = overallStatsIn.timeCombined_s(isfinite(overallStatsIn.timeCombined_s));
    finiteAltitudeError = ...
        overallStatsIn.altitudeErrorCombined_m(isfinite(overallStatsIn.altitudeErrorCombined_m));
    if isempty(finiteTime) || isempty(finiteAltitudeError)
        return
    end

    timeMin = min(finiteTime);
    timeMax = max(finiteTime);
    timeRange = max(timeMax - timeMin, 1);
    timeMargin = max(0.25, 0.02 * timeRange);
    xlim(axHandle, [timeMin - timeMargin, timeMax + timeMargin]);

    maxAbsAltitudeError_m = max(abs(finiteAltitudeError));
    yLimitAbs_m = roundUpNice(max(maxAbsAltitudeError_m * 1.10, 0.5));
    ylim(axHandle, [-yLimitAbs_m, yLimitAbs_m]);
end

function roundedValue = roundUpNice(valueIn)
    if ~isfinite(valueIn) || valueIn <= 0
        roundedValue = 1.0;
        return
    end

    exponentValue = floor(log10(valueIn));
    scaleValue = 10^exponentValue;
    mantissaValue = valueIn / scaleValue;

    if mantissaValue <= 1.0
        roundedMantissa = 1.0;
    elseif mantissaValue <= 2.0
        roundedMantissa = 2.0;
    elseif mantissaValue <= 5.0
        roundedMantissa = 5.0;
    else
        roundedMantissa = 10.0;
    end

    roundedValue = roundedMantissa * scaleValue;
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

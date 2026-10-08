%% plotPathTrackingDiagnostics
% Visualise how a simulated trajectory sits relative to an NED waypoint
% path using the same sequential active-leg logic as crosstrackerrortest.m.
%
% Expected workflow:
% 1. Ensure a waypoint path exists in the workspace, for example via:
%       - waypointsNED
%       - fakeWaypointExport.waypointsNED
%       - sortieWaypointExport.waypointsNED
%       - waypoints{selectedWaypointSortie}
% 2. Run your Simulink model so that either:
%       - out.Position.Data / out.Position.Time
%       - pos / t
%    are available in the workspace.
% 3. Run this script.

%% -------------------- USER SETTINGS --------------------
selectedSampleTime_s = NaN;      % NaN = start at the final sample
actualTimeOffset_s = 0.0;        % shift actual time if needed
actualPositionFrame = 'NED';     % 'NED' or 'ENU'
acceptanceRadiusOverride_m = []; % [] = auto-detect from workspace/default
showAcceptanceCircles = [];
if isempty(showAcceptanceCircles)
    showAcceptanceCircles = true;
end

%% -------------------- LOAD DATA --------------------
[waypointsNED, waypointSourceLabel] = resolveWaypointPath();
[actualPosRaw, actualTime_s, actualSourceLabel] = resolveActualTrajectory();

assert(size(waypointsNED, 2) >= 3, ...
    'waypointsNED must have at least 3 columns [North East Down].');
assert(size(waypointsNED, 1) >= 2, ...
    'waypointsNED must contain at least two waypoint rows.');

actualPosRaw = reshapePositionSamples(actualPosRaw);
assert(size(actualPosRaw, 2) >= 3, ...
    'Actual position data must have at least 3 columns.');

actualPosNED = convertActualPositionToNED(actualPosRaw(:,1:3), actualPositionFrame);
actualTime_s = actualTime_s(:) - actualTimeOffset_s;

validMask = all(isfinite(actualPosNED), 2) & isfinite(actualTime_s);
actualPosNED = actualPosNED(validMask,:);
actualTime_s = actualTime_s(validMask);

assert(~isempty(actualTime_s), 'No valid actual trajectory samples were found.');

if ~isempty(acceptanceRadiusOverride_m)
    acceptanceRadius_m = acceptanceRadiusOverride_m;
elseif evalin('base', 'exist(''acceptanceRadius_m'', ''var'')') == 1
    acceptanceRadiusCandidate_m = evalin('base', 'acceptanceRadius_m');
    if isnumeric(acceptanceRadiusCandidate_m) && isscalar(acceptanceRadiusCandidate_m) && ...
            isfinite(acceptanceRadiusCandidate_m) && acceptanceRadiusCandidate_m > 0
        acceptanceRadius_m = acceptanceRadiusCandidate_m;
    else
        acceptanceRadius_m = 20.0;
    end
elseif evalin('base', 'exist(''fakeWaypointExport'', ''var'')') == 1
    fakeWaypointExportStruct = evalin('base', 'fakeWaypointExport');
    if isstruct(fakeWaypointExportStruct) && ...
            isfield(fakeWaypointExportStruct, 'acceptanceRadius_m') && ...
            isfinite(fakeWaypointExportStruct.acceptanceRadius_m) && ...
            fakeWaypointExportStruct.acceptanceRadius_m > 0
        acceptanceRadius_m = fakeWaypointExportStruct.acceptanceRadius_m;
    else
        acceptanceRadius_m = 20.0;
    end
else
    acceptanceRadius_m = 20.0;
end

diagnostics = computeSequentialCrossTrackDiagnostics(actualPosNED, actualTime_s, ...
    waypointsNED, acceptanceRadius_m);

if isnan(selectedSampleTime_s)
    initialSampleIdx = numel(actualTime_s);
else
    [~, initialSampleIdx] = min(abs(actualTime_s - selectedSampleTime_s));
end
initialSampleIdx = min(max(initialSampleIdx, 1), numel(actualTime_s));

%% -------------------- FIGURE --------------------
fig = figure('Color', 'w', ...
    'Name', 'Path Tracking Diagnostics', 'NumberTitle', 'off');

axRoute = axes('Parent', fig);
set(axRoute, 'Position', [0.07 0.16 0.55 0.76]);
hold(axRoute, 'on')
grid(axRoute, 'on')
axis(axRoute, 'equal')
xlabel(axRoute, 'East [m]')
ylabel(axRoute, 'North [m]')
title(axRoute, 'Waypoint path and cross-track geometry')

axCrossTrack = axes('Parent', fig);
set(axCrossTrack, 'Position', [0.68 0.58 0.28 0.28]);
hold(axCrossTrack, 'on')
grid(axCrossTrack, 'on')
xlabel(axCrossTrack, 'Time [s]')
ylabel(axCrossTrack, 'Error [m]')
title(axCrossTrack, 'Cross-track error')

axSegment = axes('Parent', fig);
set(axSegment, 'Position', [0.68 0.22 0.28 0.23]);
hold(axSegment, 'on')
grid(axSegment, 'on')
xlabel(axSegment, 'Time [s]')
ylabel(axSegment, 'Active leg index')
title(axSegment, 'Sequential active leg')

hWaypointPath = plot(axRoute, waypointsNED(:,2), waypointsNED(:,1), '--o', ...
    'Color', [0.88 0.46 0.10], 'LineWidth', 1.8, ...
    'MarkerFaceColor', 'w', 'DisplayName', 'Waypoint polyline');
hActualFull = plot(axRoute, actualPosNED(:,2), actualPosNED(:,1), '-', ...
    'Color', [0.90 0.78 0.78], 'LineWidth', 1.2, ...
    'DisplayName', 'Actual path (full)');
hActualTrail = plot(axRoute, NaN, NaN, '-', ...
    'Color', [0.82 0.15 0.15], 'LineWidth', 2.2, ...
    'DisplayName', 'Actual path to sample');
hActiveLeg = plot(axRoute, NaN, NaN, '-', ...
    'Color', [0.00 0.35 0.90], 'LineWidth', 2.8, ...
    'DisplayName', 'Active leg');
hCrossTrackLine = plot(axRoute, NaN, NaN, ':', ...
    'Color', [0.10 0.55 0.10], 'LineWidth', 2.0, ...
    'DisplayName', 'Cross-track line');
hCurrentPoint = plot(axRoute, NaN, NaN, 'd', ...
    'Color', [0.82 0.15 0.15], 'MarkerFaceColor', [1.00 0.78 0.78], ...
    'MarkerSize', 9, 'LineWidth', 1.5, 'DisplayName', 'Current sample');
hClosestPoint = plot(axRoute, NaN, NaN, 'x', ...
    'Color', [0.10 0.10 0.10], 'MarkerSize', 10, 'LineWidth', 2.0, ...
    'DisplayName', 'Closest point');
hWaypointA = plot(axRoute, NaN, NaN, 'o', ...
    'Color', [0.10 0.45 0.10], 'MarkerFaceColor', [0.75 1.00 0.75], ...
    'MarkerSize', 8, 'LineWidth', 1.4, 'DisplayName', 'Waypoint A');
hWaypointB = plot(axRoute, NaN, NaN, 's', ...
    'Color', [0.10 0.45 0.10], 'MarkerFaceColor', [0.75 1.00 0.75], ...
    'MarkerSize', 8, 'LineWidth', 1.4, 'DisplayName', 'Waypoint B');

theta = linspace(0, 2*pi, 181);
if showAcceptanceCircles
    hAcceptance = gobjects(size(waypointsNED,1),1);
    for iWaypoint = 1:size(waypointsNED,1)
        eastCircle = waypointsNED(iWaypoint,2) + acceptanceRadius_m * cos(theta);
        northCircle = waypointsNED(iWaypoint,1) + acceptanceRadius_m * sin(theta);
        if iWaypoint == 1
            displayName = 'Acceptance radius';
        else
            displayName = '';
        end
        hAcceptance(iWaypoint) = plot(axRoute, eastCircle, northCircle, ':', ...
            'Color', [0.10 0.55 0.10], 'LineWidth', 1.2, ...
            'DisplayName', displayName);
        if iWaypoint > 1
            set(hAcceptance(iWaypoint), 'HandleVisibility', 'off');
        end
    end
else
    hAcceptance = gobjects(0,1);
end

plot(axCrossTrack, diagnostics.time_s, diagnostics.crossTrackMagnitude_m, '-', ...
    'Color', [0.10 0.35 0.90], 'LineWidth', 1.8, ...
    'DisplayName', '|Cross-track|');
plot(axCrossTrack, diagnostics.time_s, diagnostics.crossTrackSigned_m, '--', ...
    'Color', [0.75 0.15 0.75], 'LineWidth', 1.3, ...
    'DisplayName', 'Signed cross-track');
hCrossTrackCursor = plot(axCrossTrack, NaN, NaN, ':', ...
    'Color', [0.82 0.15 0.15], 'LineWidth', 1.4, ...
    'DisplayName', 'Selected sample');
hCrossTrackMarker = plot(axCrossTrack, NaN, NaN, 'o', ...
    'Color', [0.10 0.35 0.90], 'MarkerFaceColor', [0.10 0.35 0.90], ...
    'MarkerSize', 6, 'DisplayName', 'Current |error|');

stairs(axSegment, diagnostics.time_s, diagnostics.activeLegIdx, '-', ...
    'Color', [0.00 0.35 0.90], 'LineWidth', 1.8, ...
    'DisplayName', 'Active leg');
plot(axSegment, diagnostics.time_s(diagnostics.switchReasonCode > 0), ...
    diagnostics.activeLegIdx(diagnostics.switchReasonCode > 0), 'o', ...
    'Color', [0.85 0.45 0.10], 'MarkerFaceColor', [0.85 0.45 0.10], ...
    'MarkerSize', 5, 'DisplayName', 'Leg switch');
hSegmentCursor = plot(axSegment, NaN, NaN, ':', ...
    'Color', [0.82 0.15 0.15], 'LineWidth', 1.4, ...
    'DisplayName', 'Selected sample');
hSegmentMarker = plot(axSegment, NaN, NaN, 'o', ...
    'Color', [0.00 0.35 0.90], 'MarkerFaceColor', [0.00 0.35 0.90], ...
    'MarkerSize', 6, 'DisplayName', 'Current leg');

hStats = annotation(fig, 'textbox', [0.66 0.02 0.31 0.16], ...
    'String', '', 'FitBoxToText', 'off', 'BackgroundColor', 'w', ...
    'EdgeColor', 'k', 'FontSize', 10, 'Interpreter', 'none');

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.10 0.06 0.11 0.03], 'String', 'Selected sample', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'left', ...
    'FontSize', 10, 'FontWeight', 'bold');

hSlider = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.21 0.06 0.37 0.035], ...
    'Min', 1, 'Max', numel(actualTime_s), 'Value', initialSampleIdx);
if numel(actualTime_s) > 1
    set(hSlider, 'SliderStep', [1, max(1, round(numel(actualTime_s) / 20))] / (numel(actualTime_s) - 1));
else
    set(hSlider, 'Enable', 'off');
end

hReadout = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.10 0.02 0.50 0.03], 'String', '', ...
    'BackgroundColor', get(fig, 'Color'), 'HorizontalAlignment', 'left', ...
    'FontSize', 10, 'FontWeight', 'bold');

for iWaypoint = 1:size(waypointsNED,1)
    text(axRoute, waypointsNED(iWaypoint,2) + 3, waypointsNED(iWaypoint,1) + 3, ...
        sprintf('W%d', iWaypoint), 'FontSize', 9, ...
        'FontWeight', 'bold', 'Color', [0.35 0.18 0.05]);
end

legendHandles = [hWaypointPath, hActualFull, hActualTrail, hActiveLeg, ...
    hCrossTrackLine, hCurrentPoint, hClosestPoint, hWaypointA, hWaypointB];
if ~isempty(hAcceptance)
    legendHandles = [legendHandles, hAcceptance(1)];
end
legend(axRoute, legendHandles, 'Location', 'best')
legend(axCrossTrack, 'Location', 'best')
legend(axSegment, 'Location', 'best')

setRouteAxisLimits(axRoute, waypointsNED(:,1:2), actualPosNED(:,1:2), acceptanceRadius_m);
xlim(axCrossTrack, [diagnostics.time_s(1), diagnostics.time_s(end)])
xlim(axSegment, [diagnostics.time_s(1), diagnostics.time_s(end)])
ylim(axSegment, [0.5, max(size(waypointsNED,1) - 0.5, 1.5)])

setappdata(fig, 'diagnostics', diagnostics);
setappdata(fig, 'waypointsNED', waypointsNED);
setappdata(fig, 'acceptanceRadius_m', acceptanceRadius_m);
setappdata(fig, 'waypointSourceLabel', waypointSourceLabel);
setappdata(fig, 'actualSourceLabel', actualSourceLabel);
setappdata(fig, 'hActualTrail', hActualTrail);
setappdata(fig, 'hActiveLeg', hActiveLeg);
setappdata(fig, 'hCrossTrackLine', hCrossTrackLine);
setappdata(fig, 'hCurrentPoint', hCurrentPoint);
setappdata(fig, 'hClosestPoint', hClosestPoint);
setappdata(fig, 'hWaypointA', hWaypointA);
setappdata(fig, 'hWaypointB', hWaypointB);
setappdata(fig, 'hCrossTrackCursor', hCrossTrackCursor);
setappdata(fig, 'hCrossTrackMarker', hCrossTrackMarker);
setappdata(fig, 'hSegmentCursor', hSegmentCursor);
setappdata(fig, 'hSegmentMarker', hSegmentMarker);
setappdata(fig, 'hStats', hStats);
setappdata(fig, 'hReadout', hReadout);
setappdata(fig, 'hSlider', hSlider);

set(hSlider, 'Callback', @(src,~) updateTrackingDiagnosticFigure(fig, round(get(src, 'Value'))));
sliderListener = addlistener(hSlider, 'ContinuousValueChange', ...
    @(~,~) updateTrackingDiagnosticFigure(fig, round(get(hSlider, 'Value'))));
setappdata(fig, 'sliderListener', sliderListener);

updateTrackingDiagnosticFigure(fig, initialSampleIdx);

%% -------------------- LOCAL FUNCTIONS --------------------
function [waypointsNEDOut, sourceLabel] = resolveWaypointPath()
    if evalin('base', 'exist(''waypointsNED'', ''var'')') == 1
        waypointsNEDCandidate = evalin('base', 'waypointsNED');
        if isnumeric(waypointsNEDCandidate) && ~isempty(waypointsNEDCandidate)
            waypointsNEDOut = waypointsNEDCandidate;
            sourceLabel = 'waypointsNED';
            return
        end
    end

    if evalin('base', 'exist(''sortieWaypointExport'', ''var'')') == 1
        sortieWaypointExportStruct = evalin('base', 'sortieWaypointExport');
        if isstruct(sortieWaypointExportStruct) && ...
                isfield(sortieWaypointExportStruct, 'waypointsNED') && ...
                ~isempty(sortieWaypointExportStruct.waypointsNED)
            waypointsNEDOut = sortieWaypointExportStruct.waypointsNED;
            sourceLabel = 'sortieWaypointExport.waypointsNED';
            return
        end
    end

    if evalin('base', 'exist(''fakeWaypointExport'', ''var'')') == 1
        fakeWaypointExportStruct = evalin('base', 'fakeWaypointExport');
        if isstruct(fakeWaypointExportStruct) && ...
                isfield(fakeWaypointExportStruct, 'waypointsNED') && ...
                ~isempty(fakeWaypointExportStruct.waypointsNED)
            waypointsNEDOut = fakeWaypointExportStruct.waypointsNED;
            sourceLabel = 'fakeWaypointExport.waypointsNED';
            return
        end
    end

    if evalin('base', 'exist(''waypoints'', ''var'')') == 1
        waypointsCell = evalin('base', 'waypoints');
        if iscell(waypointsCell) && ~isempty(waypointsCell)
            sortieIdx = 0;
            if evalin('base', 'exist(''selectedWaypointSortie'', ''var'')') == 1
                selectedWaypointSortieValue = evalin('base', 'selectedWaypointSortie');
                if isfinite(selectedWaypointSortieValue) && selectedWaypointSortieValue >= 1 && ...
                        selectedWaypointSortieValue <= numel(waypointsCell) && ...
                        ~isempty(waypointsCell{selectedWaypointSortieValue})
                    sortieIdx = selectedWaypointSortieValue;
                end
            end

            if sortieIdx == 0
                for iCell = 1:numel(waypointsCell)
                    if ~isempty(waypointsCell{iCell})
                        sortieIdx = iCell;
                        break
                    end
                end
            end

            if sortieIdx > 0
                waypointsNEDOut = waypointsCell{sortieIdx};
                sourceLabel = sprintf('waypoints{%d}', sortieIdx);
                return
            end
        end
    end

    error(['Waypoint path not found. Expected waypointsNED, sortieWaypointExport.waypointsNED, ' ...
           'fakeWaypointExport.waypointsNED, or a non-empty waypoints cell array.']);
end

function [actualPosOut, actualTimeOut_s, sourceLabel] = resolveActualTrajectory()
    if evalin('base', 'exist(''out'', ''var'')') == 1
        outStruct = evalin('base', 'out');
        if isprop(outStruct, 'Position')
            actualPosOut = outStruct.Position.Data;
            actualTimeOut_s = outStruct.Position.Time;
            sourceLabel = 'out.Position';
            return
        end
    end

    if evalin('base', 'exist(''pos'', ''var'')') == 1 && ...
            evalin('base', 'exist(''t'', ''var'')') == 1
        actualPosOut = evalin('base', 'pos');
        actualTimeOut_s = evalin('base', 't');
        sourceLabel = 'pos / t';
        return
    end

    error(['Actual trajectory not found. Expected either out.Position.Data / out.Position.Time ' ...
           'or workspace variables pos and t.']);
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

function diagnosticsOut = computeSequentialCrossTrackDiagnostics(actualPosNEDIn, actualTimeIn_s, ...
        waypointsNEDIn, acceptanceRadiusIn_m)
    sampleCount = size(actualPosNEDIn, 1);
    waypointCount = size(waypointsNEDIn, 1);
    segmentCount = waypointCount - 1;

    diagnosticsOut = struct();
    diagnosticsOut.time_s = actualTimeIn_s(:);
    diagnosticsOut.actualPosNED = actualPosNEDIn;
    diagnosticsOut.actualPosNE = actualPosNEDIn(:,1:2);
    diagnosticsOut.activeLegIdx = zeros(sampleCount,1);
    diagnosticsOut.switchReasonCode = zeros(sampleCount,1);
    diagnosticsOut.closestPointNED = zeros(sampleCount,3);
    diagnosticsOut.closestPointNE = zeros(sampleCount,2);
    diagnosticsOut.crossTrackSigned_m = zeros(sampleCount,1);
    diagnosticsOut.crossTrackMagnitude_m = zeros(sampleCount,1);
    diagnosticsOut.sStarRaw = zeros(sampleCount,1);
    diagnosticsOut.sStar = zeros(sampleCount,1);
    diagnosticsOut.distanceToNextWaypoint_m = zeros(sampleCount,1);
    diagnosticsOut.distanceToClosestPoint_m = zeros(sampleCount,1);
    diagnosticsOut.w1NED = zeros(sampleCount,3);
    diagnosticsOut.w2NED = zeros(sampleCount,3);

    activeLegIdx = 1;

    for iSample = 1:sampleCount
        posNE = actualPosNEDIn(iSample,1:2);
        [activeLegIdx, switchReasonCode, distanceToNextWaypoint_m] = ...
            advanceActiveSegment(posNE, waypointsNEDIn(:,1:2), activeLegIdx, acceptanceRadiusIn_m);

        activeLegIdx = min(max(activeLegIdx, 1), segmentCount);
        w1 = waypointsNEDIn(activeLegIdx,:);
        w2 = waypointsNEDIn(activeLegIdx + 1,:);
        AB = w2(1:2) - w1(1:2);
        segLen_m = norm(AB);

        if segLen_m <= 1e-12
            sStarRaw = 0.0;
            sStar = 0.0;
            closestPointNED = w1;
            crossTrackSigned_m = 0.0;
        else
            A_air = posNE - w1(1:2);
            AB_unit = AB / segLen_m;
            sStarRaw = (A_air * AB.') / (segLen_m^2);
            sStar = min(max(sStarRaw, 0.0), 1.0);
            closestPointNED = w1 + sStar * (w2 - w1);
            crossTrackSigned_m = cross2D(A_air, AB_unit);
        end

        distanceToClosestPoint_m = norm(actualPosNEDIn(iSample,:) - closestPointNED);

        diagnosticsOut.activeLegIdx(iSample) = activeLegIdx;
        diagnosticsOut.switchReasonCode(iSample) = switchReasonCode;
        diagnosticsOut.closestPointNED(iSample,:) = closestPointNED;
        diagnosticsOut.closestPointNE(iSample,:) = closestPointNED(1:2);
        diagnosticsOut.crossTrackSigned_m(iSample) = crossTrackSigned_m;
        diagnosticsOut.crossTrackMagnitude_m(iSample) = distanceToClosestPoint_m;
        diagnosticsOut.sStarRaw(iSample) = sStarRaw;
        diagnosticsOut.sStar(iSample) = sStar;
        diagnosticsOut.distanceToNextWaypoint_m(iSample) = distanceToNextWaypoint_m;
        diagnosticsOut.distanceToClosestPoint_m(iSample) = distanceToClosestPoint_m;
        diagnosticsOut.w1NED(iSample,:) = w1;
        diagnosticsOut.w2NED(iSample,:) = w2;
    end
end

function [activeLegIdxOut, switchReasonCode, distanceToNextWaypoint_m] = ...
        advanceActiveSegment(posNE, waypointsNE, activeLegIdxIn, acceptanceRadiusIn_m)
    segmentCount = size(waypointsNE, 1) - 1;
    activeLegIdxOut = min(max(activeLegIdxIn, 1), segmentCount);
    switchReasonCode = 0.0;
    distanceToNextWaypoint_m = 0.0;

    while activeLegIdxOut < segmentCount
        candidateW1 = waypointsNE(activeLegIdxOut,:);
        candidateW2 = waypointsNE(activeLegIdxOut + 1,:);
        candidateAB = candidateW2 - candidateW1;
        candidateLenSq = candidateAB * candidateAB.';

        if candidateLenSq <= 1e-12
            activeLegIdxOut = activeLegIdxOut + 1;
            switchReasonCode = 3.0;
            continue
        end

        sStarCandidate = ((posNE - candidateW1) * candidateAB.') / candidateLenSq;
        distanceToNextWaypoint_m = norm(posNE - candidateW2);

        if distanceToNextWaypoint_m <= acceptanceRadiusIn_m
            activeLegIdxOut = activeLegIdxOut + 1;
            switchReasonCode = 1.0;
        elseif sStarCandidate > 1.0
            activeLegIdxOut = activeLegIdxOut + 1;
            switchReasonCode = 2.0;
        else
            break
        end
    end

    activeLegIdxOut = min(max(activeLegIdxOut, 1), segmentCount);

    if distanceToNextWaypoint_m <= 0
        distanceToNextWaypoint_m = norm(posNE - waypointsNE(activeLegIdxOut + 1,:));
    end
end

function updateTrackingDiagnosticFigure(figHandle, sampleIdx)
    diagnosticsLocal = getappdata(figHandle, 'diagnostics');
    acceptanceRadiusLocal_m = getappdata(figHandle, 'acceptanceRadius_m');
    waypointSourceLocal = getappdata(figHandle, 'waypointSourceLabel');
    actualSourceLocal = getappdata(figHandle, 'actualSourceLabel');

    sampleIdx = min(max(round(sampleIdx), 1), numel(diagnosticsLocal.time_s));

    set(getappdata(figHandle, 'hSlider'), 'Value', sampleIdx);

    hActualTrailLocal = getappdata(figHandle, 'hActualTrail');
    hActiveLegLocal = getappdata(figHandle, 'hActiveLeg');
    hCrossTrackLineLocal = getappdata(figHandle, 'hCrossTrackLine');
    hCurrentPointLocal = getappdata(figHandle, 'hCurrentPoint');
    hClosestPointLocal = getappdata(figHandle, 'hClosestPoint');
    hWaypointALocal = getappdata(figHandle, 'hWaypointA');
    hWaypointBLocal = getappdata(figHandle, 'hWaypointB');
    hCrossTrackCursorLocal = getappdata(figHandle, 'hCrossTrackCursor');
    hCrossTrackMarkerLocal = getappdata(figHandle, 'hCrossTrackMarker');
    hSegmentCursorLocal = getappdata(figHandle, 'hSegmentCursor');
    hSegmentMarkerLocal = getappdata(figHandle, 'hSegmentMarker');
    hStatsLocal = getappdata(figHandle, 'hStats');
    hReadoutLocal = getappdata(figHandle, 'hReadout');

    actualNE = diagnosticsLocal.actualPosNE;
    closestPointNE = diagnosticsLocal.closestPointNE;
    currentNE = actualNE(sampleIdx,:);
    currentClosestNE = closestPointNE(sampleIdx,:);
    w1 = diagnosticsLocal.w1NED(sampleIdx,1:2);
    w2 = diagnosticsLocal.w2NED(sampleIdx,1:2);
    currentTime_s = diagnosticsLocal.time_s(sampleIdx);
    currentCrossTrackMag_m = diagnosticsLocal.crossTrackMagnitude_m(sampleIdx);
    currentCrossTrackSigned_m = diagnosticsLocal.crossTrackSigned_m(sampleIdx);
    currentLegIdx = diagnosticsLocal.activeLegIdx(sampleIdx);
    currentSwitchCode = diagnosticsLocal.switchReasonCode(sampleIdx);

    set(hActualTrailLocal, 'XData', actualNE(1:sampleIdx,2), 'YData', actualNE(1:sampleIdx,1));
    set(hActiveLegLocal, 'XData', [w1(2), w2(2)], 'YData', [w1(1), w2(1)]);
    set(hCrossTrackLineLocal, 'XData', [currentNE(2), currentClosestNE(2)], ...
        'YData', [currentNE(1), currentClosestNE(1)]);
    set(hCurrentPointLocal, 'XData', currentNE(2), 'YData', currentNE(1));
    set(hClosestPointLocal, 'XData', currentClosestNE(2), 'YData', currentClosestNE(1));
    set(hWaypointALocal, 'XData', w1(2), 'YData', w1(1));
    set(hWaypointBLocal, 'XData', w2(2), 'YData', w2(1));

    crossTrackY = ylim(get(hCrossTrackCursorLocal, 'Parent'));
    set(hCrossTrackCursorLocal, 'XData', [currentTime_s, currentTime_s], 'YData', crossTrackY);
    set(hCrossTrackMarkerLocal, 'XData', currentTime_s, 'YData', currentCrossTrackMag_m);

    segmentY = ylim(get(hSegmentCursorLocal, 'Parent'));
    set(hSegmentCursorLocal, 'XData', [currentTime_s, currentTime_s], 'YData', segmentY);
    set(hSegmentMarkerLocal, 'XData', currentTime_s, 'YData', currentLegIdx);

    statsLines = {
        sprintf('Sample %d / %d', sampleIdx, numel(diagnosticsLocal.time_s))
        sprintf('Time = %.3f s', currentTime_s)
        sprintf('Active leg = %d (W%d to W%d)', currentLegIdx, currentLegIdx, currentLegIdx + 1)
        sprintf('Switch reason = %s', switchReasonLabel(currentSwitchCode))
        ''
        sprintf('Current [N E D] = [%.2f, %.2f, %.2f] m', ...
            diagnosticsLocal.actualPosNED(sampleIdx,1), ...
            diagnosticsLocal.actualPosNED(sampleIdx,2), ...
            diagnosticsLocal.actualPosNED(sampleIdx,3))
        sprintf('Closest [N E D] = [%.2f, %.2f, %.2f] m', ...
            diagnosticsLocal.closestPointNED(sampleIdx,1), ...
            diagnosticsLocal.closestPointNED(sampleIdx,2), ...
            diagnosticsLocal.closestPointNED(sampleIdx,3))
        sprintf('|Cross-track| = %.3f m', currentCrossTrackMag_m)
        sprintf('Signed cross-track = %.3f m', currentCrossTrackSigned_m)
        sprintf('s* raw = %.3f', diagnosticsLocal.sStarRaw(sampleIdx))
        sprintf('s* clamped = %.3f', diagnosticsLocal.sStar(sampleIdx))
        sprintf('Distance to next waypoint = %.3f m', diagnosticsLocal.distanceToNextWaypoint_m(sampleIdx))
        ''
        sprintf('Mean |cross-track| = %.3f m', mean(diagnosticsLocal.crossTrackMagnitude_m, 'omitnan'))
        sprintf('Max |cross-track| = %.3f m', max(diagnosticsLocal.crossTrackMagnitude_m))
        sprintf('Acceptance radius = %.3f m', acceptanceRadiusLocal_m)
        sprintf('Waypoints source: %s', waypointSourceLocal)
        sprintf('Actual source: %s', actualSourceLocal)
        };
    set(hStatsLocal, 'String', strjoin(statsLines, newline));

    set(hReadoutLocal, 'String', sprintf(['t = %.3f s | leg %d | |cross-track| = %.3f m | ' ...
        'signed = %.3f m'], currentTime_s, currentLegIdx, ...
        currentCrossTrackMag_m, currentCrossTrackSigned_m));
end

function label = switchReasonLabel(code)
    switch round(code)
        case 1
            label = 'Acceptance radius';
        case 2
            label = 'Passed waypoint projection';
        case 3
            label = 'Skipped zero-length segment';
        otherwise
            label = 'No switch on this sample';
    end
end

function setRouteAxisLimits(axHandle, waypointsNEIn, actualNEIn, acceptanceRadiusIn_m)
    northCandidates = [waypointsNEIn(:,1); actualNEIn(:,1)];
    eastCandidates = [waypointsNEIn(:,2); actualNEIn(:,2)];

    eastRange = max(eastCandidates) - min(eastCandidates);
    northRange = max(northCandidates) - min(northCandidates);
    eastMargin = max([20, 0.10 * max(eastRange, 1), acceptanceRadiusIn_m]);
    northMargin = max([20, 0.10 * max(northRange, 1), acceptanceRadiusIn_m]);

    xlim(axHandle, [min(eastCandidates) - eastMargin, max(eastCandidates) + eastMargin])
    ylim(axHandle, [min(northCandidates) - northMargin, max(northCandidates) + northMargin])
end

function out = cross2D(u, v)
    out = u(1) * v(2) - u(2) * v(1);
end

%% runAllSortieTrackingAnalysis
% Batch-run the Simulink sortie model and analyse spray-lane cross-track and
% altitude tracking performance across the mission.
%
% Expected workflow:
% 1. Run runMission.m so missionCfg,
%    planningData, simulationData, and waypoints exist in the workspace.
% 2. Run plotSortieWaypoints.m so spraywaypoints exists.
% 3. Set simModelName below to your active Simulink model.
% 4. Run this script.
%
% The script finds Constant blocks whose Value is waypoints{...} and
% spraywaypoints{...}, changes their index for each sortie, updates the
% initial heading expression InitialHeadingRadBySortie(...), runs the model,
% and stores lane/sortie/global tracking metrics.

%% -------------------- USER SETTINGS --------------------
simModelName = 'SIXDOF28';
if ~exist('sortieIndices', 'var')
    sortieIndices = [];
end
maxSortiesToRun = Inf;

crossTrackTarget_m = 0.6;
altitudeTarget_m = 0.7;
steadyStateIgnore_s = 5.0;
actualTimeOffset_s = 0.0;
actualPositionFrame = 'NED';

saveResults = true;
outputFolder = fullfile(projectRoot(), 'outputs', 'tracking');
plotOverviewFigures = true;
storePerSortieData = true;
storePerSortieSampleTables = false;
storeActualTrajectoryTables = false;
storeRawSimOutput = false;
saveBatchSampleTable = false;

% Leave these empty for automatic detection. Fill with block paths if your
% model has more than one waypoint Constant block or uses unusual names.
waypointConstantBlockPaths = {};
sprayWaypointConstantBlockPaths = {};

% This is normally auto-detected from block or mask parameters containing
% InitialHeadingRadBySortie(46), for example:
% [0 InitPitch InitialHeadingRadBySortie(46)]
initialHeadingParameterRefs = [];

%% -------------------- VALIDATE INPUT DATA --------------------
fprintf('Starting batch tracking setup...\n');
assert(exist('missionCfg', 'var') == 1, ...
    'missionCfg was not found. Run runMission.m first.');
assert(exist('planningData', 'var') == 1, ...
    'planningData was not found. Run runMission.m first.');
assert(exist('simulationData', 'var') == 1, ...
    'simulationData was not found. Run runMission.m first.');
assert(exist('waypoints', 'var') == 1 && iscell(waypoints), ...
    'waypoints must be a cell array indexed by sortie number.');
assert(exist('spraywaypoints', 'var') == 1 && iscell(spraywaypoints), ...
    'spraywaypoints must be a cell array indexed by sortie number.');

validateMissionCfg(missionCfg);
validatePlanningData(planningData);
validateSimulationData(simulationData);

if isempty(sortieIndices)
    sortieIndices = findValidSortieIndices(waypoints, spraywaypoints, maxSortiesToRun);
else
    sortieIndices = sortieIndices(:).';
end
assert(~isempty(sortieIndices), 'No valid sorties were found to analyse.');

sprayWidth_m = resolveSprayWidthForDiagnostics(missionCfg);
ensureOutputFolder(outputFolder, saveResults);
fprintf('Loading Simulink model "%s"...\n', simModelName);
initialiseAircraft;
loadModelForBatch(simModelName);

if isempty(waypointConstantBlockPaths)
    fprintf('Finding waypoint Constant block(s)...\n');
    waypointConstantBlockPaths = findIndexedConstantBlocks(simModelName, 'waypoints');
end
fprintf('Found %d waypoint Constant block(s).\n', numel(waypointConstantBlockPaths));
if isempty(sprayWaypointConstantBlockPaths)
    fprintf('Finding spray waypoint Constant block(s)...\n');
    sprayWaypointConstantBlockPaths = findIndexedConstantBlocks(simModelName, 'spraywaypoints');
end
fprintf('Found %d spray waypoint Constant block(s).\n', numel(sprayWaypointConstantBlockPaths));
if isempty(initialHeadingParameterRefs)
    fprintf('Finding initial-heading parameter(s)...\n');
    initialHeadingParameterRefs = findBlockParametersContainingText(simModelName, ...
        'InitialHeadingRadBySortie');
end
fprintf('Found %d initial-heading parameter reference(s).\n', numel(initialHeadingParameterRefs));
assert(~isempty(waypointConstantBlockPaths), ...
    'Could not find a Constant block with Value like waypoints{46}. Fill waypointConstantBlockPaths manually.');
assert(~isempty(sprayWaypointConstantBlockPaths), ...
    'Could not find a Constant block with Value like spraywaypoints{46}. Fill sprayWaypointConstantBlockPaths manually.');
assert(~isempty(initialHeadingParameterRefs), ...
    ['Could not find a block or mask parameter containing InitialHeadingRadBySortie(...). ' ...
     'Check the 6DOF block initial Euler orientation parameter.']);

fprintf('Running %d sorties through model "%s"...\n', numel(sortieIndices), simModelName);
fprintf('Targets: |cross-track| <= %.2f m, |altitude error| <= %.2f m\n', ...
    crossTrackTarget_m, altitudeTarget_m);

%% -------------------- RUN AND ANALYSE SORTIES --------------------
allLaneTables = cell(numel(sortieIndices), 1);
allSortieTables = cell(numel(sortieIndices), 1);
failedRunTables = cell(0, 1);
allLaneResultsBySortie = cell(numel(sortieIndices), 1);
allLaneSampleTables = cell(numel(sortieIndices), 1);

for iRun = 1:numel(sortieIndices)
    sortieIdx = sortieIndices(iRun);
    fprintf('[%3d/%3d] Sortie %d\n', iRun, numel(sortieIndices), sortieIdx);

    try
        setSortieConstantBlocks(waypointConstantBlockPaths, ...
            sprintf('waypoints{%d}', sortieIdx));
        setSortieConstantBlocks(sprayWaypointConstantBlockPaths, ...
            sprintf('spraywaypoints{%d}', sortieIdx));
        configureFlightModel(sortieIdx, simulationData);
        % configureFlightModel also updates initial position and simulation duration.
        setInitialHeadingParameterRefs(initialHeadingParameterRefs, sortieIdx);

        runTimer = tic;
        simOut = runSingleSortieSimulation(simModelName);
        simRunWallTime_s = toc(runTimer);

        [actualPosRaw, actualTime_s] = resolveActualTrajectory(simOut);
        actualTime_s = actualTime_s(:) - actualTimeOffset_s;
        actualPosNED = convertActualPositionToNED(actualPosRaw(:,1:3), actualPositionFrame);

        validActual = all(isfinite(actualPosNED), 2) & isfinite(actualTime_s);
        actualPosNED = actualPosNED(validActual,:);
        actualTime_s = actualTime_s(validActual);
        assert(size(actualPosNED, 1) >= 2, ...
            'At least two valid actual trajectory samples are required.');

        laneResults = analyseSortieTracking(sortieIdx, actualPosNED, actualTime_s, ...
            spraywaypoints, crossTrackTarget_m, altitudeTarget_m, ...
            steadyStateIgnore_s, sprayWidth_m);
        allLaneResultsBySortie{iRun} = laneResults;

        allLaneTables{iRun} = laneResultsToTable(sortieIdx, laneResults);
        allSortieTables{iRun} = summarizeSortieToTable(sortieIdx, laneResults, ...
            simRunWallTime_s, '');
        if storePerSortieSampleTables || saveBatchSampleTable
            allLaneSampleTables{iRun} = laneResultsToSampleTable(sortieIdx, laneResults, ...
                crossTrackTarget_m, altitudeTarget_m);
        end

        if saveResults && storePerSortieData
            savePerSortieResults(outputFolder, sortieIdx, actualTime_s, actualPosNED, ...
                laneResults, allLaneTables{iRun}, allSortieTables{iRun}, ...
                allLaneSampleTables{iRun}, simOut, storeActualTrajectoryTables, ...
                storePerSortieSampleTables, storeRawSimOutput);
        end
    catch ME
        warning('Sortie %d failed: %s', sortieIdx, ME.message);
        failedRunTables{end+1,1} = failedRunToTable(sortieIdx, ME); %#ok<SAGROW>
        allSortieTables{iRun} = summarizeSortieToTable(sortieIdx, makeEmptyLaneResult(), ...
            NaN, ME.message);
        if saveResults && storePerSortieData
            saveFailedSortieResults(outputFolder, sortieIdx, ME, allSortieTables{iRun});
        end
    end
end

batchLaneTable = vertcatNonEmptyTables(allLaneTables);
batchSortieTable = vertcatNonEmptyTables(allSortieTables);
batchFailedRunTable = vertcatNonEmptyTables(failedRunTables);
if saveBatchSampleTable
    batchLaneSampleTable = vertcatNonEmptyTables(allLaneSampleTables);
else
    batchLaneSampleTable = table();
end
batchTrackingSummary = summarizeBatchTracking(batchLaneTable, batchSortieTable, ...
    crossTrackTarget_m, altitudeTarget_m);
batchDistributionStats = summarizeBatchDistributions(batchLaneTable, batchSortieTable);

assignin('base', 'batchLaneTable', batchLaneTable);
assignin('base', 'batchSortieTable', batchSortieTable);
assignin('base', 'batchFailedRunTable', batchFailedRunTable);
assignin('base', 'batchLaneSampleTable', batchLaneSampleTable);
assignin('base', 'batchTrackingSummary', batchTrackingSummary);
assignin('base', 'batchDistributionStats', batchDistributionStats);
assignin('base', 'batchLaneResultsBySortie', allLaneResultsBySortie);

fprintf('\nBatch tracking summary\n');
disp(batchTrackingSummary)

if saveResults
    save(fullfile(outputFolder, 'batchTrackingResults.mat'), ...
        'batchLaneTable', 'batchSortieTable', 'batchFailedRunTable', ...
        'batchLaneSampleTable', 'batchTrackingSummary', 'batchDistributionStats', ...
        'allLaneResultsBySortie', ...
        'sortieIndices', 'crossTrackTarget_m', 'altitudeTarget_m', ...
        'steadyStateIgnore_s', 'sprayWidth_m');
    writetable(batchLaneTable, fullfile(outputFolder, 'batchLaneTable.csv'));
    writetable(batchSortieTable, fullfile(outputFolder, 'batchSortieTable.csv'));
    writetable(batchDistributionStats, fullfile(outputFolder, 'batchDistributionStats.csv'));
    if saveBatchSampleTable && ~isempty(batchLaneSampleTable)
        writetable(batchLaneSampleTable, fullfile(outputFolder, 'batchLaneSampleTable.csv'));
    end
    if ~isempty(batchFailedRunTable)
        writetable(batchFailedRunTable, fullfile(outputFolder, 'batchFailedRunTable.csv'));
    end
end

if plotOverviewFigures
    plotBatchTrackingOverview(batchLaneTable, batchSortieTable, ...
        crossTrackTarget_m, altitudeTarget_m);
end

%% -------------------- LOCAL FUNCTIONS --------------------
function sortieIndices = findValidSortieIndices(waypointsIn, spraywaypointsIn, maxSortiesToRun)
    maxCandidate = min([numel(waypointsIn), numel(spraywaypointsIn), maxSortiesToRun]);
    validMask = false(1, maxCandidate);
    for iSortie = 1:maxCandidate
        validMask(iSortie) = ~isempty(waypointsIn{iSortie}) && ...
            ~isempty(spraywaypointsIn{iSortie});
    end
    sortieIndices = find(validMask);
end

function ensureOutputFolder(outputFolder, saveResults)
    if saveResults && ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
end

function loadModelForBatch(modelName)
    if bdIsLoaded(modelName)
        return
    end

    try
        load_system(modelName);
    catch ME
        error(['Could not load Simulink model "%s". Open the model or set simModelName ' ...
            'to the exact loaded model name before running this script. MATLAB message: %s'], ...
            modelName, ME.message);
    end
end

function blockPaths = findIndexedConstantBlocks(modelName, variableName)
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    constantBlocks = find_system(modelName, 'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', 'BlockType', 'Constant');
    blockPaths = cell(0, 1);
    pattern = ['(^|[^A-Za-z0-9_])' variableName '\s*\{'];

    for iBlock = 1:numel(constantBlocks)
        valueExpr = get_param(constantBlocks{iBlock}, 'Value');
        if ~isempty(regexp(valueExpr, pattern, 'once'))
            blockPaths{end+1,1} = constantBlocks{iBlock}; %#ok<AGROW>
        end
    end
end

function parameterRefs = findBlockParametersContainingText(modelName, searchText)
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    blockPaths = find_system(modelName, 'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', 'Type', 'Block');
    parameterRefs = struct('BlockPath', {}, 'ParameterName', {}, ...
        'OriginalValue', {}, 'ValueIndex', {});

    for iBlock = 1:numel(blockPaths)
        parameterRefs = appendDialogParameterRefs(parameterRefs, blockPaths{iBlock}, searchText);
        parameterRefs = appendMaskValueRefs(parameterRefs, blockPaths{iBlock}, searchText);
    end
end

function parameterRefs = appendDialogParameterRefs(parameterRefs, blockPath, searchText)
    try
        dialogParameters = get_param(blockPath, 'DialogParameters');
    catch
        return
    end

    if isempty(dialogParameters)
        return
    end

    parameterNames = fieldnames(dialogParameters);
    for iParam = 1:numel(parameterNames)
        paramName = parameterNames{iParam};
        try
            paramValue = get_param(blockPath, paramName);
        catch
            continue
        end

        if ischar(paramValue) && contains(paramValue, searchText)
            parameterRefs(end+1).BlockPath = blockPath; %#ok<AGROW>
            parameterRefs(end).ParameterName = paramName;
            parameterRefs(end).OriginalValue = paramValue;
            parameterRefs(end).ValueIndex = NaN;
        end
    end
end

function parameterRefs = appendMaskValueRefs(parameterRefs, blockPath, searchText)
    try
        maskValues = get_param(blockPath, 'MaskValues');
    catch
        return
    end

    if ~iscell(maskValues)
        return
    end

    for iValue = 1:numel(maskValues)
        if ischar(maskValues{iValue}) && contains(maskValues{iValue}, searchText)
            parameterRefs(end+1).BlockPath = blockPath; %#ok<AGROW>
            parameterRefs(end).ParameterName = 'MaskValues';
            parameterRefs(end).OriginalValue = maskValues{iValue};
            parameterRefs(end).ValueIndex = iValue;
        end
    end
end

function setSortieConstantBlocks(blockPaths, valueExpression)
    for iBlock = 1:numel(blockPaths)
        set_param(blockPaths{iBlock}, 'Value', valueExpression);
    end
end

function setInitialHeadingParameterRefs(parameterRefs, sortieIdx)
    replacementText = sprintf('InitialHeadingRadBySortie(%d)', sortieIdx);
    pattern = 'InitialHeadingRadBySortie\s*\(\s*\d+\s*\)';

    for iRef = 1:numel(parameterRefs)
        if strcmp(parameterRefs(iRef).ParameterName, 'MaskValues')
            maskValues = get_param(parameterRefs(iRef).BlockPath, 'MaskValues');
            valueIdx = parameterRefs(iRef).ValueIndex;
            if ~isfinite(valueIdx) || valueIdx < 1 || valueIdx > numel(maskValues)
                continue
            end

            currentValue = maskValues{valueIdx};
            updatedValue = regexprep(currentValue, pattern, replacementText);
            maskValues{valueIdx} = updatedValue;
            set_param(parameterRefs(iRef).BlockPath, 'MaskValues', maskValues);
        else
            currentValue = get_param(parameterRefs(iRef).BlockPath, ...
                parameterRefs(iRef).ParameterName);
            if ~ischar(currentValue)
                currentValue = parameterRefs(iRef).OriginalValue;
            end

            updatedValue = regexprep(currentValue, pattern, replacementText);
            set_param(parameterRefs(iRef).BlockPath, ...
                parameterRefs(iRef).ParameterName, updatedValue);
        end
    end
end

function simOut = runSingleSortieSimulation(modelName)
    try
        simOut = sim(modelName, 'ReturnWorkspaceOutputs', 'on');
    catch firstError
        try
            simOut = sim(modelName);
        catch
            rethrow(firstError)
        end
    end
end

function [actualPosRaw, actualTime_s] = resolveActualTrajectory(simOut)
    [foundSignal, actualPosRaw, actualTime_s] = tryResolveTrajectoryFromSimulationOutput(simOut);
    if foundSignal
        actualPosRaw = reshapePositionSamples(actualPosRaw);
        return
    end

    if evalin('base', 'exist(''out'', ''var'')') == 1
        outVar = evalin('base', 'out');
        [foundSignal, actualPosRaw, actualTime_s] = tryResolvePositionSignal(outVar);
        if foundSignal
            actualPosRaw = reshapePositionSamples(actualPosRaw);
            return
        end
    end

    if evalin('base', 'exist(''pos'', ''var'')') == 1 && ...
            evalin('base', 'exist(''t'', ''var'')') == 1
        actualPosRaw = evalin('base', 'pos');
        actualTime_s = evalin('base', 't');
        actualPosRaw = reshapePositionSamples(actualPosRaw);
        return
    end

    error(['Simulink trajectory not found. Expected out.Position, Position, ' ...
        'position, or pos/t output variables.']);
end

function [foundSignal, posData, timeData] = tryResolveTrajectoryFromSimulationOutput(simOut)
    foundSignal = false;
    posData = zeros(0,3);
    timeData = zeros(0,1);

    if isempty(simOut)
        return
    end

    candidateNames = {'Position', 'position', 'pos'};
    for iName = 1:numel(candidateNames)
        signalName = candidateNames{iName};
        signalValue = [];
        hasSignal = false;

        if isobject(simOut)
            try
                signalValue = simOut.get(signalName);
                hasSignal = true;
            catch
                hasSignal = false;
            end
        elseif isstruct(simOut) && isfield(simOut, signalName)
            signalValue = simOut.(signalName);
            hasSignal = true;
        end

        if hasSignal
            if strcmp(signalName, 'pos') && isobject(simOut)
                try
                    posData = signalValue;
                    timeData = simOut.get('t');
                    foundSignal = true;
                    return
                catch
                end
            end

            [foundSignal, posData, timeData] = tryResolvePositionSignalValue(signalValue);
            if foundSignal
                return
            end
        end
    end
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

        [foundSignal, posData, timeData] = tryResolvePositionSignalValue(signalValue);
        if foundSignal
            return
        end
    end
end

function [foundSignal, posData, timeData] = tryResolvePositionSignalValue(signalValue)
    foundSignal = false;
    posData = zeros(0,3);
    timeData = zeros(0,1);

    if isstruct(signalValue) && isfield(signalValue, 'Data') && isfield(signalValue, 'Time')
        posData = signalValue.Data;
        timeData = signalValue.Time;
        foundSignal = true;
        return
    end

    if isa(signalValue, 'timeseries')
        posData = signalValue.Data;
        timeData = signalValue.Time;
        foundSignal = true;
        return
    end

    if isobject(signalValue) && isprop(signalValue, 'Data') && isprop(signalValue, 'Time')
        posData = signalValue.Data;
        timeData = signalValue.Time;
        foundSignal = true;
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

    assert(size(posSamples, 2) >= 3, 'Actual position data must have at least 3 columns.');
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

function laneResults = analyseSortieTracking(sortieIdx, actualPosNED, actualTime_s, ...
        spraywaypointsIn, crossTrackTarget_m, altitudeTarget_m, ...
        steadyStateIgnore_s, sprayWidth_m)
    sprayWaypointNED = spraywaypointsIn{sortieIdx};
    assert(size(sprayWaypointNED, 2) >= 3, ...
        'spraywaypoints{%d} must have at least 3 columns [N E D].', sortieIdx);
    assert(mod(size(sprayWaypointNED, 1), 2) == 0, ...
        'spraywaypoints{%d} must contain start/end pairs for each spray lane.', sortieIdx);

    laneData = buildLaneDefinitions(sprayWaypointNED(:,1:3));
    actualVelNED = estimateSampleVelocityNED(actualPosNED, actualTime_s);
    laneResults = analyseSequentialSprayLanes(actualPosNED, actualVelNED, ...
        actualTime_s, laneData);
    laneResults = addPerformanceMetricsToLaneResults(laneResults, ...
        crossTrackTarget_m, altitudeTarget_m, steadyStateIgnore_s, sprayWidth_m);
end

function laneData = buildLaneDefinitions(sprayWaypointNEDIn)
    laneCount = size(sprayWaypointNEDIn, 1) / 2;
    laneData = repmat(struct( ...
        'laneNumber', NaN, ...
        'startNED', zeros(1,3), ...
        'endNED', zeros(1,3), ...
        'laneWaypointsNED', zeros(2,3), ...
        'length_m', NaN), laneCount, 1);

    for iLane = 1:laneCount
        rowStart = 2 * iLane - 1;
        rowEnd = 2 * iLane;
        laneData(iLane).laneNumber = iLane;
        laneData(iLane).startNED = sprayWaypointNEDIn(rowStart,:);
        laneData(iLane).endNED = sprayWaypointNEDIn(rowEnd,:);
        laneData(iLane).laneWaypointsNED = sprayWaypointNEDIn([rowStart rowEnd],:);
        laneData(iLane).length_m = norm(laneData(iLane).endNED(1:2) - ...
            laneData(iLane).startNED(1:2));
    end
end

function laneResults = analyseSequentialSprayLanes(actualPosNEDIn, actualVelNEDIn, actualTimeIn_s, laneDataIn)
    laneCount = numel(laneDataIn);
    laneResults = repmat(makeEmptyLaneResult(), laneCount, 1);
    searchStartIdx = 1;

    for iLane = 1:laneCount
        laneResults(iLane) = analyseSingleSprayLane(actualPosNEDIn, actualVelNEDIn, ...
            actualTimeIn_s, laneDataIn(iLane), searchStartIdx);
        searchStartIdx = min(max(laneResults(iLane).nextSearchIdx, searchStartIdx + 1), ...
            size(actualPosNEDIn, 1));
    end
end

function laneResult = makeEmptyLaneResult()
    emptyCross = struct('found', false, 'beforeIdx', NaN, 'afterIdx', NaN, ...
        'alpha', NaN, 'posNED', [NaN NaN NaN], 'time_s', NaN);
    laneResult = struct( ...
        'laneNumber', NaN, ...
        'entryStatus', '', ...
        'exitStatus', '', ...
        'hasValidEntry', false, ...
        'hasValidExit', false, ...
        'entryCross', emptyCross, ...
        'exitCross', emptyCross, ...
        'sectionTime_s', zeros(0,1), ...
        'sectionPosNED', zeros(0,3), ...
        'crossTrackSigned_m', zeros(0,1), ...
        'crossTrackAbs_m', zeros(0,1), ...
        'altitudeErrorSigned_m', zeros(0,1), ...
        'altitudeErrorAbs_m', zeros(0,1), ...
        'combinedErrorAbs_m', zeros(0,1), ...
        'duration_s', NaN, ...
        'sampleCount', 0, ...
        'meanAbsCTE_m', NaN, ...
        'maxAbsCTE_m', NaN, ...
        'rmsCTE_m', NaN, ...
        'p95AbsCTE_m', NaN, ...
        'meanSignedCTE_m', NaN, ...
        'stdCTE_m', NaN, ...
        'meanAbsAltitudeError_m', NaN, ...
        'maxAbsAltitudeError_m', NaN, ...
        'rmsAltitudeError_m', NaN, ...
        'p95AbsAltitudeError_m', NaN, ...
        'meanSignedAltitudeError_m', NaN, ...
        'stdAltitudeError_m', NaN, ...
        'meanAbsCombinedError_m', NaN, ...
        'maxAbsCombinedError_m', NaN, ...
        'rmsCombinedError_m', NaN, ...
        'p95AbsCombinedError_m', NaN, ...
        'crossTrackTimeInTarget_pct', NaN, ...
        'altitudeTimeInTarget_pct', NaN, ...
        'combinedTimeInTarget_pct', NaN, ...
        'settlingTimeCTE_s', NaN, ...
        'settlingTimeAltitude_s', NaN, ...
        'settlingTimeCombined_s', NaN, ...
        'steadyMeanAbsCTE_m', NaN, ...
        'steadyMeanAbsAltitudeError_m', NaN, ...
        'steadyRmsCTE_m', NaN, ...
        'steadyRmsAltitudeError_m', NaN, ...
        'coverageQuality_pct', NaN, ...
        'meanSwathOffset_m', NaN, ...
        'meanAbsSwathOffset_pct', NaN, ...
        'maxAbsSwathOffset_pct', NaN, ...
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

    entryCross = findFinishLineCrossing(actualPosNEDIn, actualTimeIn_s, ...
        alongTrack_m, 0.0, searchStartIdx, 1);
    if entryCross.found
        entryStatus = 'valid';
        sectionStartTime_s = entryCross.time_s;
        sectionStartSampleIdx = entryCross.afterIdx;
    else
        laneResult = makeUntrackedLaneResult(laneDataIn, entryCross, ...
            'no valid start gate crossing', min(searchStartIdx + 1, nSamples));
        return
    end

    endSearchIdx = min(max(sectionStartSampleIdx, searchStartIdx), nSamples);
    exitCross = findFinishLineCrossing(actualPosNEDIn, actualTimeIn_s, ...
        alongTrack_m, laneLength_m, endSearchIdx);
    if exitCross.found
        exitStatus = 'valid';
        sectionEndTime_s = exitCross.time_s;
        sectionEndSampleIdx = max(sectionStartSampleIdx, exitCross.beforeIdx);
        nextSearchIdx = exitCross.afterIdx;
    else
        inBandAfterStart = find(inBandMask(endSearchIdx:end), 1, 'last');
        if ~isempty(inBandAfterStart)
            sectionEndSampleIdx = endSearchIdx + inBandAfterStart - 1;
            sectionEndTime_s = actualTimeIn_s(sectionEndSampleIdx);
            exitStatus = 'no exit crossing';
            nextSearchIdx = sectionEndSampleIdx + 1;
        else
            sectionEndSampleIdx = sectionStartSampleIdx;
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

    laneResult = makeEmptyLaneResult();
    laneResult.laneNumber = laneDataIn.laneNumber;
    laneResult.entryStatus = entryStatus;
    laneResult.exitStatus = exitStatus;
    laneResult.hasValidEntry = strcmp(entryStatus, 'valid');
    laneResult.hasValidExit = strcmp(exitStatus, 'valid');
    laneResult.entryCross = entryCross;
    laneResult.exitCross = exitCross;
    laneResult.sectionTime_s = actualTimeIn_s(sectionSampleIdx);
    laneResult.sectionPosNED = actualPosNEDIn(sectionSampleIdx,:);
    laneResult.crossTrackSigned_m = laneMetrics.crossTrackSigned_m;
    laneResult.crossTrackAbs_m = laneMetrics.crossTrackAbs_m;
    laneResult.altitudeErrorSigned_m = laneMetrics.altitudeErrorSigned_m;
    laneResult.altitudeErrorAbs_m = laneMetrics.altitudeErrorAbs_m;
    laneResult.duration_s = max(sectionEndTime_s - sectionStartTime_s, 0.0);
    laneResult.sampleCount = numel(sectionSampleIdx);
    laneResult.meanAbsCTE_m = meanOrNaN(laneResult.crossTrackAbs_m);
    laneResult.maxAbsCTE_m = maxOrNaN(laneResult.crossTrackAbs_m);
    laneResult.rmsCTE_m = sqrt(meanOrNaN(laneResult.crossTrackSigned_m.^2));
    laneResult.meanAbsAltitudeError_m = meanOrNaN(laneResult.altitudeErrorAbs_m);
    laneResult.maxAbsAltitudeError_m = maxOrNaN(laneResult.altitudeErrorAbs_m);
    laneResult.rmsAltitudeError_m = sqrt(meanOrNaN(laneResult.altitudeErrorSigned_m.^2));
    laneResult.nextSearchIdx = nextSearchIdx;
    laneResult.statusSummary = buildLaneStatusSummary(entryStatus, exitStatus);
end

function laneResult = makeUntrackedLaneResult(laneDataIn, entryCross, entryStatus, nextSearchIdx)
    laneResult = makeEmptyLaneResult();
    laneResult.laneNumber = laneDataIn.laneNumber;
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
        cross.posNED = actualPosNEDIn(iSample,:) + ...
            alpha * (actualPosNEDIn(iSample + 1,:) - actualPosNEDIn(iSample,:));
        cross.time_s = actualTimeIn_s(iSample) + ...
            alpha * (actualTimeIn_s(iSample + 1) - actualTimeIn_s(iSample));
        return
    end
end

function laneMetrics = computeLaneCrossTrackMetrics(posNEDIn, velNEDIn, laneWaypointsNEDIn)
    sampleCount = size(posNEDIn, 1);
    laneMetrics = struct();
    laneMetrics.crossTrackSigned_m = zeros(sampleCount,1);
    laneMetrics.crossTrackAbs_m = zeros(sampleCount,1);
    laneMetrics.altitudeErrorSigned_m = zeros(sampleCount,1);
    laneMetrics.altitudeErrorAbs_m = zeros(sampleCount,1);
    closestPointNED = zeros(sampleCount,3);

    for iSample = 1:sampleCount
        [laneMetrics.crossTrackSigned_m(iSample), ~, ~, ~, ~, ~, closestPointNED(iSample,:)] = ...
            crossTrackErrorABNED(posNEDIn(iSample,:), velNEDIn(iSample,:), ...
            laneWaypointsNEDIn, 1, 0.0);
    end

    laneMetrics.crossTrackAbs_m = abs(laneMetrics.crossTrackSigned_m);
    actualAltitude_m = -posNEDIn(:,3);
    targetAltitude_m = -closestPointNED(:,3);
    laneMetrics.altitudeErrorSigned_m = actualAltitude_m - targetAltitude_m;
    laneMetrics.altitudeErrorAbs_m = abs(laneMetrics.altitudeErrorSigned_m);
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
        laneResultsOut(iLane).meanSwathOffset_m = laneResultsOut(iLane).meanSignedCTE_m;
        laneResultsOut(iLane).meanAbsSwathOffset_pct = swathOffsetPct(meanOrNaN(ctAbs_m), sprayWidth_m);
        laneResultsOut(iLane).maxAbsSwathOffset_pct = swathOffsetPct(laneResultsOut(iLane).maxAbsCTE_m, sprayWidth_m);
    end
end

function laneTable = laneResultsToTable(sortieIdx, laneResults)
    rowCount = numel(laneResults);
    laneTable = table();
    if rowCount == 0
        return
    end

    laneTable.Sortie = repmat(sortieIdx, rowCount, 1);
    laneTable.Lane = (1:rowCount).';
    laneTable.Status = string({laneResults.statusSummary}.');
    laneTable.ValidEntry = [laneResults.hasValidEntry].';
    laneTable.ValidExit = [laneResults.hasValidExit].';
    laneTable.Duration_s = [laneResults.duration_s].';
    laneTable.SampleCount = [laneResults.sampleCount].';
    laneTable.CT_MeanAbs_m = [laneResults.meanAbsCTE_m].';
    laneTable.CT_RMSE_m = [laneResults.rmsCTE_m].';
    laneTable.CT_MaxAbs_m = [laneResults.maxAbsCTE_m].';
    laneTable.CT_P95Abs_m = [laneResults.p95AbsCTE_m].';
    laneTable.CT_Bias_m = [laneResults.meanSignedCTE_m].';
    laneTable.CT_Std_m = [laneResults.stdCTE_m].';
    laneTable.CT_TimeInTarget_pct = [laneResults.crossTrackTimeInTarget_pct].';
    laneTable.CT_TargetPass = laneTable.CT_TimeInTarget_pct >= 95;
    laneTable.CT_SettlingTime_s = [laneResults.settlingTimeCTE_s].';
    laneTable.CT_SteadyMeanAbs_m = [laneResults.steadyMeanAbsCTE_m].';
    laneTable.Alt_MeanAbs_m = [laneResults.meanAbsAltitudeError_m].';
    laneTable.Alt_RMSE_m = [laneResults.rmsAltitudeError_m].';
    laneTable.Alt_MaxAbs_m = [laneResults.maxAbsAltitudeError_m].';
    laneTable.Alt_P95Abs_m = [laneResults.p95AbsAltitudeError_m].';
    laneTable.Alt_Bias_m = [laneResults.meanSignedAltitudeError_m].';
    laneTable.Alt_Std_m = [laneResults.stdAltitudeError_m].';
    laneTable.Alt_TimeInTarget_pct = [laneResults.altitudeTimeInTarget_pct].';
    laneTable.Alt_TargetPass = laneTable.Alt_TimeInTarget_pct >= 95;
    laneTable.Alt_SettlingTime_s = [laneResults.settlingTimeAltitude_s].';
    laneTable.Alt_SteadyMeanAbs_m = [laneResults.steadyMeanAbsAltitudeError_m].';
    laneTable.Combined_RMSE_m = [laneResults.rmsCombinedError_m].';
    laneTable.Combined_MaxAbs_m = [laneResults.maxAbsCombinedError_m].';
    laneTable.Combined_P95Abs_m = [laneResults.p95AbsCombinedError_m].';
    laneTable.Both_TimeInTarget_pct = [laneResults.combinedTimeInTarget_pct].';
    laneTable.Both_TargetPass = laneTable.Both_TimeInTarget_pct >= 95;
    laneTable.FailureReason = classifyLaneFailureReasons(laneTable.CT_TargetPass, ...
        laneTable.Alt_TargetPass);
    laneTable.Both_SettlingTime_s = [laneResults.settlingTimeCombined_s].';
    laneTable.CoverageQuality_pct = [laneResults.coverageQuality_pct].';
    laneTable.MeanSwathOffset_m = [laneResults.meanSwathOffset_m].';
    laneTable.MeanAbsSwathOffset_pct = [laneResults.meanAbsSwathOffset_pct].';
    laneTable.MaxAbsSwathOffset_pct = [laneResults.maxAbsSwathOffset_pct].';
end

function sampleTable = laneResultsToSampleTable(sortieIdx, laneResults, ...
        crossTrackTarget_m, altitudeTarget_m)
    sampleTables = cell(numel(laneResults), 1);

    for iLane = 1:numel(laneResults)
        lane = laneResults(iLane);
        sampleCount = numel(lane.sectionTime_s);
        if sampleCount == 0
            continue
        end

        ctSigned_m = lane.crossTrackSigned_m(:);
        altSigned_m = lane.altitudeErrorSigned_m(:);
        combinedAbs_m = hypot(ctSigned_m, altSigned_m);
        inCrossTrackTarget = abs(ctSigned_m) <= crossTrackTarget_m;
        inAltitudeTarget = abs(altSigned_m) <= altitudeTarget_m;

        thisTable = table();
        thisTable.Sortie = repmat(sortieIdx, sampleCount, 1);
        thisTable.Lane = repmat(iLane, sampleCount, 1);
        thisTable.SampleInLane = (1:sampleCount).';
        thisTable.Time_s = lane.sectionTime_s(:);
        thisTable.TimeFromLaneEntry_s = lane.sectionTime_s(:) - lane.sectionTime_s(1);
        thisTable.North_m = lane.sectionPosNED(:,1);
        thisTable.East_m = lane.sectionPosNED(:,2);
        thisTable.Down_m = lane.sectionPosNED(:,3);
        thisTable.Altitude_m = -lane.sectionPosNED(:,3);
        thisTable.CrossTrackError_m = ctSigned_m;
        thisTable.CrossTrackAbs_m = abs(ctSigned_m);
        thisTable.AltitudeError_m = altSigned_m;
        thisTable.AltitudeErrorAbs_m = abs(altSigned_m);
        thisTable.CombinedErrorAbs_m = combinedAbs_m;
    thisTable.InCrossTrackTarget = inCrossTrackTarget;
    thisTable.InAltitudeTarget = inAltitudeTarget;
    thisTable.InBothTargets = inCrossTrackTarget & inAltitudeTarget;
    thisTable.CT_TargetRatio = abs(ctSigned_m) ./ crossTrackTarget_m;
    thisTable.Alt_TargetRatio = abs(altSigned_m) ./ altitudeTarget_m;
    thisTable.WorstTargetRatio = max([thisTable.CT_TargetRatio, thisTable.Alt_TargetRatio], [], 2);
    sampleTables{iLane} = thisTable;
    end

    sampleTable = vertcatNonEmptyTables(sampleTables);
end

function savePerSortieResults(outputFolder, sortieIdx, actualTime_s, actualPosNED, ...
        laneResults, laneTable, sortieTable, laneSampleTable, simOut, ...
        storeActualTrajectoryTables, storePerSortieSampleTables, storeRawSimOutput)
    sortieFolder = fullfile(outputFolder, 'sorties', sprintf('sortie_%03d', sortieIdx));
    if ~exist(sortieFolder, 'dir')
        mkdir(sortieFolder);
    end

    writetable(laneTable, fullfile(sortieFolder, 'laneMetrics.csv'));
    writetable(sortieTable, fullfile(sortieFolder, 'sortieSummary.csv'));

    if storeActualTrajectoryTables
        actualTrajectoryTable = actualTrajectoryToTable(sortieIdx, actualTime_s, actualPosNED);
        writetable(actualTrajectoryTable, fullfile(sortieFolder, 'actualTrajectory.csv'));
    else
        actualTrajectoryTable = table();
    end

    if storePerSortieSampleTables && ~isempty(laneSampleTable)
        writetable(laneSampleTable, fullfile(sortieFolder, 'laneSamples.csv'));
    end

    save(fullfile(sortieFolder, 'sortieTrackingData.mat'), ...
        'sortieIdx', 'actualTime_s', 'actualPosNED', 'laneResults', ...
        'laneTable', 'sortieTable', 'laneSampleTable', 'actualTrajectoryTable');

    if storeRawSimOutput
        save(fullfile(sortieFolder, 'rawSimOutput.mat'), 'simOut', '-v7.3');
    end
end

function saveFailedSortieResults(outputFolder, sortieIdx, ME, sortieTable)
    sortieFolder = fullfile(outputFolder, 'sorties', sprintf('sortie_%03d', sortieIdx));
    if ~exist(sortieFolder, 'dir')
        mkdir(sortieFolder);
    end

    failureTable = failedRunToTable(sortieIdx, ME);
    writetable(sortieTable, fullfile(sortieFolder, 'sortieSummary.csv'));
    writetable(failureTable, fullfile(sortieFolder, 'failure.csv'));
    save(fullfile(sortieFolder, 'failure.mat'), 'sortieIdx', 'ME', 'sortieTable', 'failureTable');
end

function actualTrajectoryTable = actualTrajectoryToTable(sortieIdx, actualTime_s, actualPosNED)
    sampleCount = numel(actualTime_s);
    actualTrajectoryTable = table();
    actualTrajectoryTable.Sortie = repmat(sortieIdx, sampleCount, 1);
    actualTrajectoryTable.Sample = (1:sampleCount).';
    actualTrajectoryTable.Time_s = actualTime_s(:);
    actualTrajectoryTable.North_m = actualPosNED(:,1);
    actualTrajectoryTable.East_m = actualPosNED(:,2);
    actualTrajectoryTable.Down_m = actualPosNED(:,3);
    actualTrajectoryTable.Altitude_m = -actualPosNED(:,3);
end

function failureReason = classifyLaneFailureReasons(ctPass, altPass)
    failureReason = strings(numel(ctPass), 1);
    for iRow = 1:numel(ctPass)
        if ctPass(iRow) && altPass(iRow)
            failureReason(iRow) = "Within both targets";
        elseif ~ctPass(iRow) && altPass(iRow)
            failureReason(iRow) = "Cross-track only";
        elseif ctPass(iRow) && ~altPass(iRow)
            failureReason(iRow) = "Altitude only";
        else
            failureReason(iRow) = "Both targets";
        end
    end
end

function sortieTable = summarizeSortieToTable(sortieIdx, laneResults, simRunWallTime_s, errorMessage)
    if numel(laneResults) == 1 && isnan(laneResults.laneNumber) && laneResults.sampleCount == 0
        laneCount = 0;
        validLaneCount = 0;
    else
        laneCount = numel(laneResults);
        validLaneCount = sum([laneResults.hasValidEntry] & [laneResults.hasValidExit]);
    end

    sortieTable = table();
    sortieTable.Sortie = sortieIdx;
    sortieTable.RunSucceeded = strlength(string(errorMessage)) == 0;
    sortieTable.ErrorMessage = string(errorMessage);
    sortieTable.SimWallTime_s = simRunWallTime_s;
    sortieTable.LaneCount = laneCount;
    sortieTable.ValidLaneCount = validLaneCount;
    sortieTable.IncompleteLaneCount = laneCount - validLaneCount;
    sortieTable.TotalSprayDuration_s = sum([laneResults.duration_s], 'omitnan');
    sortieTable.TotalSpraySamples = sum([laneResults.sampleCount], 'omitnan');
    sortieTable.Mean_CT_RMSE_m = meanOrNaN([laneResults.rmsCTE_m].');
    sortieTable.DurationWeighted_CT_RMSE_m = weightedRmsFromRmse( ...
        [laneResults.rmsCTE_m].', [laneResults.duration_s].');
    sortieTable.Worst_CT_MaxAbs_m = maxOrNaN([laneResults.maxAbsCTE_m].');
    sortieTable.Mean_Alt_RMSE_m = meanOrNaN([laneResults.rmsAltitudeError_m].');
    sortieTable.DurationWeighted_Alt_RMSE_m = weightedRmsFromRmse( ...
        [laneResults.rmsAltitudeError_m].', [laneResults.duration_s].');
    sortieTable.Worst_Alt_MaxAbs_m = maxOrNaN([laneResults.maxAbsAltitudeError_m].');
    sortieTable.Mean_Combined_RMSE_m = meanOrNaN([laneResults.rmsCombinedError_m].');
    sortieTable.DurationWeighted_Combined_RMSE_m = weightedRmsFromRmse( ...
        [laneResults.rmsCombinedError_m].', [laneResults.duration_s].');
    sortieTable.Worst_Combined_MaxAbs_m = maxOrNaN([laneResults.maxAbsCombinedError_m].');
    sortieTable.Mean_Both_TimeInTarget_pct = meanOrNaN([laneResults.combinedTimeInTarget_pct].');
    sortieTable.DurationWeighted_Both_TimeInTarget_pct = weightedMeanOrNaN( ...
        [laneResults.combinedTimeInTarget_pct].', [laneResults.duration_s].');
    sortieTable.Worst_Both_TimeInTarget_pct = minOrNaN([laneResults.combinedTimeInTarget_pct].');
    sortieTable.Mean_CoverageQuality_pct = meanOrNaN([laneResults.coverageQuality_pct].');
    sortieTable.Mean_SettlingTimeBoth_s = meanOrNaN([laneResults.settlingTimeCombined_s].');
    laneQuality_pct = [laneResults.combinedTimeInTarget_pct].';
    sortieTable.Median_Both_TimeInTarget_pct = medianOrNaN(laneQuality_pct);
    sortieTable.LanesAtOrAbove95Pct = sum(isfinite(laneQuality_pct) & laneQuality_pct >= 95);
    sortieTable.LanesBelow80Pct = sum(isfinite(laneQuality_pct) & laneQuality_pct < 80);
    sortieTable.Worst_CT_P95Abs_m = maxOrNaN([laneResults.p95AbsCTE_m].');
    sortieTable.Worst_Alt_P95Abs_m = maxOrNaN([laneResults.p95AbsAltitudeError_m].');
    sortieTable.Worst_Combined_P95Abs_m = maxOrNaN([laneResults.p95AbsCombinedError_m].');
    sortieTable.Mean_CT_Bias_m = meanOrNaN([laneResults.meanSignedCTE_m].');
    sortieTable.Mean_Alt_Bias_m = meanOrNaN([laneResults.meanSignedAltitudeError_m].');
end

function failedTable = failedRunToTable(sortieIdx, ME)
    failedTable = table();
    failedTable.Sortie = sortieIdx;
    failedTable.Identifier = string(ME.identifier);
    failedTable.Message = string(ME.message);
end

function batchSummary = summarizeBatchTracking(laneTable, sortieTable, crossTrackTarget_m, altitudeTarget_m)
    batchSummary = table();
    batchSummary.SortiesAttempted = height(sortieTable);
    batchSummary.SortiesSucceeded = sum(sortieTable.RunSucceeded);
    batchSummary.SortiesFailed = sum(~sortieTable.RunSucceeded);
    if isempty(laneTable) || width(laneTable) == 0
        batchSummary.TotalLanes = 0;
        batchSummary.ValidLanes = 0;
        batchSummary.IncompleteLanes = 0;
        batchSummary.CrossTrackTarget_m = crossTrackTarget_m;
        batchSummary.AltitudeTarget_m = altitudeTarget_m;
        batchSummary.Overall_CT_RMSE_m = NaN;
        batchSummary.MeanLane_CT_RMSE_m = NaN;
        batchSummary.DurationWeighted_CT_RMSE_m = NaN;
        batchSummary.WorstLane_CT_MaxAbs_m = NaN;
        batchSummary.MeanLane_Alt_RMSE_m = NaN;
        batchSummary.DurationWeighted_Alt_RMSE_m = NaN;
        batchSummary.WorstLane_Alt_MaxAbs_m = NaN;
        batchSummary.MeanLane_Combined_RMSE_m = NaN;
        batchSummary.DurationWeighted_Combined_RMSE_m = NaN;
        batchSummary.WorstLane_Combined_MaxAbs_m = NaN;
        batchSummary.MeanLane_Both_TimeInTarget_pct = NaN;
        batchSummary.DurationWeighted_Both_TimeInTarget_pct = NaN;
        batchSummary.WorstSortie_Both_TimeInTarget_pct = ...
            minOrNaN(sortieTable.Mean_Both_TimeInTarget_pct);
        batchSummary.MeanSortie_CoverageQuality_pct = ...
            meanOrNaN(sortieTable.Mean_CoverageQuality_pct);
        batchSummary.MedianLane_Both_TimeInTarget_pct = NaN;
        batchSummary.LanesAtOrAbove95Pct = 0;
        batchSummary.LanesBelow80Pct = 0;
        batchSummary.LanePassRate95_pct = NaN;
        batchSummary.WorstLaneQuality_pct = NaN;
        batchSummary.WorstQualitySortie = NaN;
        batchSummary.WorstQualityLane = NaN;
        batchSummary.WorstCTSortie = NaN;
        batchSummary.WorstCTLane = NaN;
        batchSummary.WorstAltSortie = NaN;
        batchSummary.WorstAltLane = NaN;
        return
    end

    batchSummary.TotalLanes = height(laneTable);
    batchSummary.ValidLanes = sum(laneTable.ValidEntry & laneTable.ValidExit);
    batchSummary.IncompleteLanes = batchSummary.TotalLanes - batchSummary.ValidLanes;
    batchSummary.CrossTrackTarget_m = crossTrackTarget_m;
    batchSummary.AltitudeTarget_m = altitudeTarget_m;
    batchSummary.Overall_CT_RMSE_m = rmsFromColumn(laneTable.CT_RMSE_m);
    batchSummary.MeanLane_CT_RMSE_m = meanOrNaN(laneTable.CT_RMSE_m);
    batchSummary.DurationWeighted_CT_RMSE_m = weightedRmsFromRmse( ...
        laneTable.CT_RMSE_m, laneTable.Duration_s);
    batchSummary.WorstLane_CT_MaxAbs_m = maxOrNaN(laneTable.CT_MaxAbs_m);
    batchSummary.MeanLane_Alt_RMSE_m = meanOrNaN(laneTable.Alt_RMSE_m);
    batchSummary.DurationWeighted_Alt_RMSE_m = weightedRmsFromRmse( ...
        laneTable.Alt_RMSE_m, laneTable.Duration_s);
    batchSummary.WorstLane_Alt_MaxAbs_m = maxOrNaN(laneTable.Alt_MaxAbs_m);
    batchSummary.MeanLane_Combined_RMSE_m = meanOrNaN(laneTable.Combined_RMSE_m);
    batchSummary.DurationWeighted_Combined_RMSE_m = weightedRmsFromRmse( ...
        laneTable.Combined_RMSE_m, laneTable.Duration_s);
    batchSummary.WorstLane_Combined_MaxAbs_m = maxOrNaN(laneTable.Combined_MaxAbs_m);
    batchSummary.MeanLane_Both_TimeInTarget_pct = meanOrNaN(laneTable.Both_TimeInTarget_pct);
    batchSummary.DurationWeighted_Both_TimeInTarget_pct = weightedMeanOrNaN( ...
        laneTable.Both_TimeInTarget_pct, laneTable.Duration_s);
    batchSummary.WorstSortie_Both_TimeInTarget_pct = minOrNaN(sortieTable.Mean_Both_TimeInTarget_pct);
    batchSummary.MeanSortie_CoverageQuality_pct = meanOrNaN(sortieTable.Mean_CoverageQuality_pct);
    batchSummary.MedianLane_Both_TimeInTarget_pct = medianOrNaN(laneTable.Both_TimeInTarget_pct);
    batchSummary.LanesAtOrAbove95Pct = sum(isfinite(laneTable.Both_TimeInTarget_pct) & ...
        laneTable.Both_TimeInTarget_pct >= 95);
    batchSummary.LanesBelow80Pct = sum(isfinite(laneTable.Both_TimeInTarget_pct) & ...
        laneTable.Both_TimeInTarget_pct < 80);
    batchSummary.LanePassRate95_pct = 100.0 * batchSummary.LanesAtOrAbove95Pct / ...
        max(batchSummary.TotalLanes, 1);
    [worstLaneQuality_pct, worstLaneIdx] = minFiniteWithIndex(laneTable.Both_TimeInTarget_pct);
    [worstLaneCTMaxAbs_m, worstCTIdx] = maxFiniteWithIndex(laneTable.CT_MaxAbs_m);
    [worstLaneAltMaxAbs_m, worstAltIdx] = maxFiniteWithIndex(laneTable.Alt_MaxAbs_m);
    batchSummary.WorstLaneQuality_pct = worstLaneQuality_pct;
    batchSummary.WorstLane_CT_MaxAbs_m = worstLaneCTMaxAbs_m;
    batchSummary.WorstLane_Alt_MaxAbs_m = worstLaneAltMaxAbs_m;
    batchSummary.WorstQualitySortie = tableIndexValue(laneTable.Sortie, worstLaneIdx);
    batchSummary.WorstQualityLane = tableIndexValue(laneTable.Lane, worstLaneIdx);
    batchSummary.WorstCTSortie = tableIndexValue(laneTable.Sortie, worstCTIdx);
    batchSummary.WorstCTLane = tableIndexValue(laneTable.Lane, worstCTIdx);
    batchSummary.WorstAltSortie = tableIndexValue(laneTable.Sortie, worstAltIdx);
    batchSummary.WorstAltLane = tableIndexValue(laneTable.Lane, worstAltIdx);
end

function distributionStats = summarizeBatchDistributions(laneTable, sortieTable)
    distributionStats = table();
    if isempty(laneTable) || width(laneTable) == 0
        return
    end

    metricNames = {
        'Lane CT RMSE [m]', laneTable.CT_RMSE_m
        'Lane CT max abs [m]', laneTable.CT_MaxAbs_m
        'Lane CT P95 abs [m]', laneTable.CT_P95Abs_m
        'Lane altitude RMSE [m]', laneTable.Alt_RMSE_m
        'Lane altitude max abs [m]', laneTable.Alt_MaxAbs_m
        'Lane altitude P95 abs [m]', laneTable.Alt_P95Abs_m
        'Lane combined RMSE [m]', laneTable.Combined_RMSE_m
        'Lane combined max abs [m]', laneTable.Combined_MaxAbs_m
        'Lane both in target [%]', laneTable.Both_TimeInTarget_pct
        'Lane coverage quality [%]', laneTable.CoverageQuality_pct
        'Lane settling both [s]', laneTable.Both_SettlingTime_s
        'Sortie quality mean [%]', sortieTable.Mean_Both_TimeInTarget_pct
        'Sortie quality duration weighted [%]', sortieTable.DurationWeighted_Both_TimeInTarget_pct
        };

    rowCount = size(metricNames, 1);
    distributionStats.Metric = strings(rowCount, 1);
    distributionStats.Count = zeros(rowCount, 1);
    distributionStats.Mean = NaN(rowCount, 1);
    distributionStats.Median = NaN(rowCount, 1);
    distributionStats.Std = NaN(rowCount, 1);
    distributionStats.Min = NaN(rowCount, 1);
    distributionStats.P05 = NaN(rowCount, 1);
    distributionStats.P10 = NaN(rowCount, 1);
    distributionStats.P90 = NaN(rowCount, 1);
    distributionStats.P95 = NaN(rowCount, 1);
    distributionStats.P99 = NaN(rowCount, 1);
    distributionStats.Max = NaN(rowCount, 1);

    for iMetric = 1:rowCount
        values = metricNames{iMetric, 2};
        finiteValues = values(isfinite(values));
        distributionStats.Metric(iMetric) = string(metricNames{iMetric, 1});
        distributionStats.Count(iMetric) = numel(finiteValues);
        distributionStats.Mean(iMetric) = meanOrNaN(finiteValues);
        distributionStats.Median(iMetric) = medianOrNaN(finiteValues);
        distributionStats.Std(iMetric) = stdOrNaN(finiteValues);
        distributionStats.Min(iMetric) = minOrNaN(finiteValues);
        distributionStats.P05(iMetric) = percentileFinite(finiteValues, 5);
        distributionStats.P10(iMetric) = percentileFinite(finiteValues, 10);
        distributionStats.P90(iMetric) = percentileFinite(finiteValues, 90);
        distributionStats.P95(iMetric) = percentileFinite(finiteValues, 95);
        distributionStats.P99(iMetric) = percentileFinite(finiteValues, 99);
        distributionStats.Max(iMetric) = maxOrNaN(finiteValues);
    end
end

function plotBatchTrackingOverview(laneTable, sortieTable, crossTrackTarget_m, altitudeTarget_m)
    if isempty(laneTable) || width(laneTable) == 0 || isempty(sortieTable)
        return
    end

    figure('Color', 'w', 'Name', 'Batch Tracking Overview', 'NumberTitle', 'off');
    tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    axQuality = nexttile;
    bar(axQuality, sortieTable.Sortie, sortieTable.Mean_Both_TimeInTarget_pct);
    hold(axQuality, 'on')
    yline(axQuality, 95, 'k--', '95%', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylim(axQuality, [0 100])
    title(axQuality, 'Sortie spray quality')
    xlabel(axQuality, 'Sortie')
    ylabel(axQuality, 'Both in target [%]')
    grid(axQuality, 'on')

    axRmse = nexttile;
    plot(axRmse, sortieTable.Sortie, ...
        sortieTable.DurationWeighted_CT_RMSE_m ./ crossTrackTarget_m, '-o', ...
        'DisplayName', 'Cross-track');
    hold(axRmse, 'on')
    plot(axRmse, sortieTable.Sortie, ...
        sortieTable.DurationWeighted_Alt_RMSE_m ./ altitudeTarget_m, '-o', ...
        'DisplayName', 'Altitude');
    yline(axRmse, 1.0, 'k--', 'Target ratio = 1', 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
    title(axRmse, 'Duration-weighted RMSE / target')
    xlabel(axRmse, 'Sortie')
    ylabel(axRmse, 'RMSE / target [-]')
    legend(axRmse, 'Location', 'best')
    grid(axRmse, 'on')

    axFailure = nexttile;
    [failureCounts, failureLabels, sortieLabelsForFailure] = buildFailureReasonCounts(laneTable);
    bar(axFailure, sortieLabelsForFailure, failureCounts, 'stacked');
    title(axFailure, 'Lane failure reason by sortie')
    xlabel(axFailure, 'Sortie')
    ylabel(axFailure, 'Lane count')
    legend(axFailure, failureLabels, 'Location', 'best')
    grid(axFailure, 'on')

    axHist = nexttile;
    histogram(axHist, laneTable.Both_TimeInTarget_pct, 20);
    title(axHist, 'Lane quality distribution')
    xlabel(axHist, 'Both in target [%]')
    ylabel(axHist, 'Lane count')
    grid(axHist, 'on')

    axScatter = nexttile;
    scatter(axScatter, laneTable.CT_RMSE_m ./ crossTrackTarget_m, ...
        laneTable.Alt_RMSE_m ./ altitudeTarget_m, ...
        max(20, laneTable.Duration_s * 3), laneTable.Both_TimeInTarget_pct, 'filled', ...
        'MarkerFaceAlpha', 0.70);
    hold(axScatter, 'on')
    xline(axScatter, 1, 'k--', 'HandleVisibility', 'off');
    yline(axScatter, 1, 'k--', 'HandleVisibility', 'off');
    colorbar(axScatter)
    clim(axScatter, [0 100])
    title(axScatter, 'Lane CT vs altitude performance')
    xlabel(axScatter, 'CT RMSE / target [-]')
    ylabel(axScatter, 'Altitude RMSE / target [-]')
    grid(axScatter, 'on')

    axHeat = nexttile;
    [qualityMatrix, sortieLabels] = buildSortieLaneMatrix(laneTable, 'Both_TimeInTarget_pct');
    imagesc(axHeat, qualityMatrix);
    colorbar(axHeat)
    clim(axHeat, [0 100])
    title(axHeat, 'Lane quality heatmap')
    xlabel(axHeat, 'Sortie')
    ylabel(axHeat, 'Lane')
    xticks(axHeat, 1:numel(sortieLabels))
    xticklabels(axHeat, string(sortieLabels))
    xtickangle(axHeat, 90)

    axWorst = nexttile;
    worstLaneTable = sortrows(laneTable, 'Both_TimeInTarget_pct', 'ascend');
    worstLaneTable = worstLaneTable(1:min(10, height(worstLaneTable)), :);
    barh(axWorst, categorical(compose('S%d L%d', worstLaneTable.Sortie, worstLaneTable.Lane)), ...
        worstLaneTable.Both_TimeInTarget_pct);
    xlim(axWorst, [0 100])
    title(axWorst, 'Worst lanes')
    xlabel(axWorst, 'Both in target [%]')
    grid(axWorst, 'on')

    axKpi = nexttile;
    drawBatchKpiSummary(axKpi, laneTable, sortieTable);
end

function [qualityMatrix, sortieLabels] = buildSortieLaneMatrix(laneTable, fieldName)
    sortieLabels = unique(laneTable.Sortie, 'stable');
    maxLane = max(laneTable.Lane);
    qualityMatrix = NaN(maxLane, numel(sortieLabels));

    for iSortie = 1:numel(sortieLabels)
        thisSortie = sortieLabels(iSortie);
        rows = laneTable.Sortie == thisSortie;
        lanes = laneTable.Lane(rows);
        values = laneTable.(fieldName)(rows);
        qualityMatrix(lanes, iSortie) = values;
    end
end

function [failureCounts, failureLabels, sortieLabels] = buildFailureReasonCounts(laneTable)
    sortieLabels = unique(laneTable.Sortie, 'stable');
    failureLabels = ["Within both targets", "Cross-track only", "Altitude only", "Both targets"];
    failureCounts = zeros(numel(sortieLabels), numel(failureLabels));

    for iSortie = 1:numel(sortieLabels)
        rows = laneTable.Sortie == sortieLabels(iSortie);
        for iReason = 1:numel(failureLabels)
            failureCounts(iSortie, iReason) = sum(laneTable.FailureReason(rows) == failureLabels(iReason));
        end
    end
end

function drawBatchKpiSummary(axHandle, laneTable, sortieTable)
    cla(axHandle)
    axis(axHandle, 'off')
    title(axHandle, 'Batch KPI summary')

    [worstQuality, worstQualityIdx] = minFiniteWithIndex(laneTable.Both_TimeInTarget_pct);
    [worstCT, worstCTIdx] = maxFiniteWithIndex(laneTable.CT_MaxAbs_m);
    [worstAlt, worstAltIdx] = maxFiniteWithIndex(laneTable.Alt_MaxAbs_m);

    kpiLines = {
        sprintf('Sorties run: %d', height(sortieTable))
        sprintf('Total lanes: %d', height(laneTable))
        sprintf('Valid lanes: %d', sum(laneTable.ValidEntry & laneTable.ValidExit))
        sprintf('Mean lane quality: %.1f%%', meanOrNaN(laneTable.Both_TimeInTarget_pct))
        sprintf('Duration-weighted quality: %.1f%%', ...
            weightedMeanOrNaN(laneTable.Both_TimeInTarget_pct, laneTable.Duration_s))
        sprintf('Lanes >= 95%% quality: %d', ...
            sum(isfinite(laneTable.Both_TimeInTarget_pct) & laneTable.Both_TimeInTarget_pct >= 95))
        sprintf('Worst quality: S%d L%d = %.1f%%', ...
            tableIndexValue(laneTable.Sortie, worstQualityIdx), ...
            tableIndexValue(laneTable.Lane, worstQualityIdx), worstQuality)
        sprintf('Worst CT max: S%d L%d = %.2f m', ...
            tableIndexValue(laneTable.Sortie, worstCTIdx), ...
            tableIndexValue(laneTable.Lane, worstCTIdx), worstCT)
        sprintf('Worst altitude max: S%d L%d = %.2f m', ...
            tableIndexValue(laneTable.Sortie, worstAltIdx), ...
            tableIndexValue(laneTable.Lane, worstAltIdx), worstAlt)
        };

    text(axHandle, 0.02, 0.95, strjoin(kpiLines, newline), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontName', 'Consolas', 'FontSize', 9, 'Interpreter', 'none');
end

function sprayWidth_m = resolveSprayWidthForDiagnostics(missionCfgIn)
    if isstruct(missionCfgIn) && isfield(missionCfgIn, 'sprayWidth') && ...
            isfinite(missionCfgIn.sprayWidth) && missionCfgIn.sprayWidth > 0
        sprayWidth_m = missionCfgIn.sprayWidth;
    else
        sprayWidth_m = NaN;
    end
end

function outStr = buildLaneStatusSummary(entryStatus, exitStatus)
    if strcmp(entryStatus, 'valid') && strcmp(exitStatus, 'valid')
        outStr = 'valid';
    else
        outStr = sprintf('%s / %s', entryStatus, exitStatus);
    end
end

function outTable = vertcatNonEmptyTables(tableCells)
    nonEmptyMask = false(numel(tableCells), 1);
    for iCell = 1:numel(tableCells)
        nonEmptyMask(iCell) = ~isempty(tableCells{iCell});
    end

    if any(nonEmptyMask)
        outTable = vertcat(tableCells{nonEmptyMask});
    else
        outTable = table();
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

function valueOut = maxOrNaN(valueIn)
    finiteValues = valueIn(isfinite(valueIn));
    if isempty(finiteValues)
        valueOut = NaN;
    else
        valueOut = max(finiteValues);
    end
end

function valueOut = minOrNaN(valueIn)
    finiteValues = valueIn(isfinite(valueIn));
    if isempty(finiteValues)
        valueOut = NaN;
    else
        valueOut = min(finiteValues);
    end
end

function valueOut = medianOrNaN(valueIn)
    finiteValues = valueIn(isfinite(valueIn));
    if isempty(finiteValues)
        valueOut = NaN;
    else
        valueOut = median(finiteValues);
    end
end

function [valueOut, idxOut] = maxFiniteWithIndex(valueIn)
    finiteMask = isfinite(valueIn);
    if ~any(finiteMask)
        valueOut = NaN;
        idxOut = NaN;
        return
    end

    finiteIdx = find(finiteMask);
    [valueOut, localIdx] = max(valueIn(finiteMask));
    idxOut = finiteIdx(localIdx);
end

function [valueOut, idxOut] = minFiniteWithIndex(valueIn)
    finiteMask = isfinite(valueIn);
    if ~any(finiteMask)
        valueOut = NaN;
        idxOut = NaN;
        return
    end

    finiteIdx = find(finiteMask);
    [valueOut, localIdx] = min(valueIn(finiteMask));
    idxOut = finiteIdx(localIdx);
end

function valueOut = tableIndexValue(columnIn, idxIn)
    if ~isfinite(idxIn) || idxIn < 1 || idxIn > numel(columnIn)
        valueOut = NaN;
    else
        valueOut = columnIn(idxIn);
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
    valid = isfinite(time_s);
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

function pctOut = swathOffsetPct(offset_m, sprayWidth_m)
    if ~isfinite(offset_m) || ~isfinite(sprayWidth_m) || sprayWidth_m <= 0
        pctOut = NaN;
    else
        pctOut = 100.0 * min(abs(offset_m) / max(0.5 * sprayWidth_m, eps), 1.0);
    end
end

function valueOut = rmsFromColumn(valueIn)
    finiteValues = valueIn(isfinite(valueIn));
    if isempty(finiteValues)
        valueOut = NaN;
    else
        valueOut = sqrt(mean(finiteValues.^2));
    end
end

function valueOut = weightedMeanOrNaN(valueIn, weightsIn)
    valid = isfinite(valueIn) & isfinite(weightsIn) & weightsIn > 0;
    if ~any(valid)
        valueOut = meanOrNaN(valueIn);
    else
        valueOut = sum(valueIn(valid) .* weightsIn(valid)) / sum(weightsIn(valid));
    end
end

function valueOut = weightedRmsFromRmse(rmseIn, weightsIn)
    valid = isfinite(rmseIn) & isfinite(weightsIn) & weightsIn > 0;
    if ~any(valid)
        valueOut = rmsFromColumn(rmseIn);
    else
        valueOut = sqrt(sum((rmseIn(valid).^2) .* weightsIn(valid)) / sum(weightsIn(valid)));
    end
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

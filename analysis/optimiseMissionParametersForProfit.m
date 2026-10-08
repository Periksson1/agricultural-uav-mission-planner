%% Optimise mission parameters for maximum net profit
% Starts from the current main mission and searches candidate parameter
% values using a coordinate-search loop. This is much cheaper than a full
% factorial search across every parameter combination.
%
% Outputs:
%   - optimisation_history.csv
%   - optimisation_candidates_tested.csv
%   - optimisation_best_parameters.csv
%   - optimisation_profit_history.png / .fig

%clc
%close all

outputDir = fullfile(projectRoot(), 'outputs', 'economics', 'MissionParameterOptimisation');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Keep this false for aircraft/mission design optimisation. If true, the
% optimiser will also vary price and worker-rate assumptions, which tends
% to pick the highest price and lowest labour cost by construction.
includeEconomicAssumptions = false;

% Fast mode is usually best for report work: maxIterations = 1 tests each
% candidate once from the actual baseline and returns the best single-change
% improvement. Increase this to 12 for slower multi-pass coordinate search.
maxIterations = 1;
minProfitImprovement_GBP = 1.0;

fprintf('Running current main mission to capture the actual starting point...\n');
[baseCfg, basePlanningData, baseSimulationData] = runMainBaselineWithoutFigures();
baseCfg = seedOptionalTurnaroundFields(baseCfg);
baseResult = summariseOptimisationCase(baseCfg, basePlanningData, baseSimulationData);

parameters = buildOptimisationParameters(baseCfg, includeEconomicAssumptions);

currentCfg = baseCfg;
currentResult = baseResult;
history = makeHistoryTable(0, 'Actual baseline', 'actual', NaN, currentResult);
candidateRows = table();

fprintf('\nStarting net-profit optimisation from actual case: %.2f GBP\n', ...
    currentResult.NetProfit_GBP);

for iIteration = 1:maxIterations
    fprintf('\nIteration %d/%d\n', iIteration, maxIterations);

    bestIterationCfg = currentCfg;
    bestIterationResult = currentResult;
    bestIterationParameter = '';
    bestIterationCaseLabel = '';
    bestIterationValue = NaN;

    for iParam = 1:numel(parameters)
        param = parameters(iParam);
        for iCase = 1:numel(param.cases)
            caseDef = param.cases(iCase);
            trialCfg = param.apply(currentCfg, caseDef.value);
            trialCfg = refreshDependentMissionSettings(trialCfg);

            if isSameOptimisedCfg(trialCfg, currentCfg)
                continue
            end

            trialResult = evaluateMissionCfg(trialCfg);
            candidateRows = [candidateRows; makeCandidateRow(iIteration, param.name, ...
                caseDef.label, caseDef.value, trialResult)]; %#ok<AGROW>

            if isfinite(trialResult.NetProfit_GBP) && ...
                    trialResult.NetProfit_GBP > bestIterationResult.NetProfit_GBP + minProfitImprovement_GBP
                bestIterationCfg = trialCfg;
                bestIterationResult = trialResult;
                bestIterationParameter = param.name;
                bestIterationCaseLabel = caseDef.label;
                bestIterationValue = caseDef.value;
            end
        end
    end

    if isempty(bestIterationParameter)
        fprintf('  No parameter change improved net profit by at least %.2f GBP.\n', ...
            minProfitImprovement_GBP);
        break
    end

    currentCfg = bestIterationCfg;
    currentResult = bestIterationResult;
    fprintf('  Accepted %s = %s, net profit %.2f GBP\n', ...
        bestIterationParameter, bestIterationCaseLabel, currentResult.NetProfit_GBP);

    history = [history; makeHistoryTable(iIteration, bestIterationParameter, ...
        bestIterationCaseLabel, bestIterationValue, currentResult)]; %#ok<AGROW>
end

bestParameters = summariseBestParameters(currentCfg, currentResult);

writetable(history, fullfile(outputDir, 'optimisation_history.csv'));
writetable(candidateRows, fullfile(outputDir, 'optimisation_candidates_tested.csv'));
writetable(bestParameters, fullfile(outputDir, 'optimisation_best_parameters.csv'));
plotOptimisationHistory(history, outputDir);

fprintf('\nBest case found:\n');
disp(bestParameters);
fprintf('\nSaved optimisation outputs to:\n  %s\n', outputDir);

%% Local helpers

function [baselineCfg, baselinePlanningData, baselineSimulationData] = runMainBaselineWithoutFigures()
    [baselineCfg, baselinePlanningData, baselineSimulationData] = runBaselineMission();
end

function cfg = seedOptionalTurnaroundFields(cfg)
    cfg.payloadTurnaroundTime_s = getOptionalStructScalar(cfg, ...
        'payloadTurnaroundTime_s', cfg.turnaroundTime_s);
    cfg.batteryTurnaroundTime_s = getOptionalStructScalar(cfg, ...
        'batteryTurnaroundTime_s', cfg.turnaroundTime_s);
    cfg.sensitivityGroundStationOffset_m = getOptionalStructScalar(cfg, ...
        'sensitivityGroundStationOffset_m', 0);
end

function result = evaluateMissionCfg(cfg)
    try
        validateMissionCfg(cfg);
        planningData = runMissionPlanning(cfg);
        simulationData = runMissionSimulation(cfg, planningData);
        result = summariseOptimisationCase(cfg, planningData, simulationData);
    catch ME
        warning('Candidate failed: %s', ME.message);
        result = emptyOptimisationSummary(cfg);
    end
end

function result = summariseOptimisationCase(cfg, ~, simulationData)
    result = emptyOptimisationSummary(cfg);
    result.MissionTime_hr = simulationData.totalTime / 3600;
    result.HectaresPerHour = simulationData.overallFieldCapacity_ha_per_hr;
    result.NetProfit_GBP = simulationData.netProfit_GBP;
    result.SprayTime_pct = simulationData.sprayTimeFraction_pct;
    result.SprayedArea_ha = simulationData.sprayedArea_ha(end);
    result.Sorties = simulationData.returnCount;
end

function result = emptyOptimisationSummary(cfg)
    result = struct();
    result.MaxRollAngle_deg = rad2deg(cfg.maxRollAngle);
    result.Turnaround_min = cfg.turnaroundTime_s / 60;
    result.BatteryEnergy_kWh = cfg.batteryVoltage_V * cfg.batteryCapacity_Ah / 1000;
    result.Payload_L = cfg.payloadLitres;
    result.ApplicationRate_L_ha = cfg.applicationRate;
    result.SprayWidth_m = cfg.sprayWidth;
    result.AirSpeed_mps = cfg.uavAirSpeed;
    result.BatteryReserve_pct = cfg.batteryReturnThreshold_pct;
    result.Overlap_pct = 100 * cfg.overlap;
    result.GroundStationOffset_m = getOptionalStructScalar(cfg, 'sensitivityGroundStationOffset_m', 0);
    result.RevenuePerHa_GBP = cfg.revenuePerHa_GBP;
    result.WorkerRate_GBP_hr = cfg.workerRatePerHour_GBP;
    result.PayloadTurnaround_min = getOptionalStructScalar(cfg, 'payloadTurnaroundTime_s', cfg.turnaroundTime_s) / 60;
    result.BatteryTurnaround_min = getOptionalStructScalar(cfg, 'batteryTurnaroundTime_s', cfg.turnaroundTime_s) / 60;
    result.MissionTime_hr = NaN;
    result.HectaresPerHour = NaN;
    result.NetProfit_GBP = NaN;
    result.SprayTime_pct = NaN;
    result.SprayedArea_ha = NaN;
    result.Sorties = NaN;
end

function parameters = buildOptimisationParameters(baseCfg, includeEconomicAssumptions)
    baseBatteryEnergy_kWh = baseCfg.batteryVoltage_V * baseCfg.batteryCapacity_Ah / 1000;

    parameters = struct('name', {}, 'cases', {}, 'apply', {});
    parameters(end+1) = makeParameter('Max roll angle', ...
        [makeCase('20 deg', 20), makeCase('25 deg', 25), ...
         makeCase('30 deg actual', 30), makeCase('40 deg', 40)], ...
        @(cfg, value) setMaxRollAngleDeg(cfg, value));
    parameters(end+1) = makeParameter('Combined turnaround time', ...
        [makeCase('0 min', 0), makeCase('1 min', 1), ...
         makeCase('3 min actual', 3), makeCase('5 min', 5)], ...
        @(cfg, value) setTurnaroundMinutes(cfg, value));
    parameters(end+1) = makeParameter('Battery energy', ...
        [makeCase(sprintf('40%% current %.2f kWh', 0.40 * baseBatteryEnergy_kWh), 40), ...
         makeCase(sprintf('75%% current %.2f kWh', 0.75 * baseBatteryEnergy_kWh), 75), ...
         makeCase(sprintf('100%% actual %.2f kWh', baseBatteryEnergy_kWh), 100), ...
         makeCase(sprintf('120%% current %.2f kWh', 1.20 * baseBatteryEnergy_kWh), 120)], ...
        @(cfg, value) setBatteryEnergyPercent(cfg, value, baseCfg));
    parameters(end+1) = makeParameter('Payload', ...
        [makeCase('30 L', 30), makeCase('40 L', 40), ...
         makeCase('47 L actual', 47), makeCase('55 L', 55)], ...
        @(cfg, value) setPayloadLitres(cfg, value, baseCfg));
    parameters(end+1) = makeParameter('Application rate', ...
        [makeCase('8 L/ha', 8), makeCase('12 L/ha', 12), ...
         makeCase('15 L/ha actual', 15), makeCase('20 L/ha', 20)], ...
        @(cfg, value) setField(cfg, 'applicationRate', value));
    parameters(end+1) = makeParameter('UAV airspeed', ...
        [makeCase('18 m/s', 18), makeCase('22 m/s', 22), ...
         makeCase(sprintf('%.1f m/s actual', baseCfg.uavAirSpeed), baseCfg.uavAirSpeed), ...
         makeCase('30 m/s', 30)], ...
        @(cfg, value) setField(cfg, 'uavAirSpeed', value));
    parameters(end+1) = makeParameter('Battery reserve threshold', ...
        [makeCase('10%', 10), makeCase('15%', 15), ...
         makeCase('20% actual', 20), makeCase('25%', 25)], ...
        @(cfg, value) setField(cfg, 'batteryReturnThreshold_pct', value));
    parameters(end+1) = makeParameter('Spray overlap', ...
        [makeCase('0% actual', 0), makeCase('10%', 10), ...
         makeCase('20%', 20), makeCase('30%', 30)], ...
        @(cfg, value) setOverlapPercent(cfg, value));
    parameters(end+1) = makeParameter('Ground station offset', ...
        [makeCase('-250 m closer', -250), makeCase('0 m actual', 0), ...
         makeCase('+250 m farther', 250), makeCase('+500 m farther', 500)], ...
        @(cfg, value) setGroundStationOffsetFromActual(cfg, value, baseCfg));
    parameters(end+1) = makeParameter('Payload refill turnaround', ...
        [makeCase('1 min', 1), makeCase('2 min', 2), ...
         makeCase('3 min actual', 3), makeCase('5 min', 5)], ...
        @(cfg, value) setPayloadTurnaroundMinutes(cfg, value));
    parameters(end+1) = makeParameter('Battery turnaround', ...
        [makeCase('1 min', 1), makeCase('3 min actual', 3), ...
         makeCase('5 min', 5), makeCase('8 min', 8)], ...
        @(cfg, value) setBatteryTurnaroundMinutes(cfg, value));
    parameters(end+1) = makeParameter('Spray width', ...
        [makeCase('6 m actual', 6), makeCase('8 m', 8), ...
         makeCase('10 m', 10), makeCase('12 m', 12)], ...
        @(cfg, value) setField(cfg, 'sprayWidth', value));

    if includeEconomicAssumptions
        parameters(end+1) = makeParameter('Revenue per hectare', ...
            [makeCase('20 GBP/ha', 20), makeCase('27 GBP/ha actual', 27), ...
             makeCase('35 GBP/ha', 35), makeCase('45 GBP/ha', 45)], ...
            @(cfg, value) setField(cfg, 'revenuePerHa_GBP', value));
        parameters(end+1) = makeParameter('Worker rate', ...
            [makeCase('15 GBP/hr', 15), makeCase('20 GBP/hr actual', 20), ...
             makeCase('25 GBP/hr', 25), makeCase('30 GBP/hr', 30)], ...
            @(cfg, value) setField(cfg, 'workerRatePerHour_GBP', value));
    end
end

function parameter = makeParameter(name, cases, applyFcn)
    parameter = struct();
    parameter.name = name;
    parameter.cases = cases;
    parameter.apply = applyFcn;
end

function caseDef = makeCase(label, value)
    caseDef = struct();
    caseDef.label = label;
    caseDef.value = value;
end

function cfg = refreshDependentMissionSettings(cfg)
    cfg.fieldCount = numel(cfg.fields);
    cfg.fieldVisitOrder = 1:cfg.fieldCount;
    cfg.planningOptions.shortLaneAbsoluteThreshold_m = max(2 * cfg.sprayWidth, 12);
    cfg.planningOptions.shortLanePenalty_m = 4 * cfg.sprayWidth;
    cfg.planningOptions = sanitizePlanningOptions(cfg.planningOptions);
end

function cfg = setField(cfg, fieldName, value)
    cfg.(fieldName) = value;
end

function cfg = setMaxRollAngleDeg(cfg, value_deg)
    cfg.maxRollAngle = deg2rad(value_deg);
end

function cfg = setTurnaroundMinutes(cfg, value_min)
    cfg.turnaroundTime_s = value_min * 60;
    cfg.payloadTurnaroundTime_s = value_min * 60;
    cfg.batteryTurnaroundTime_s = value_min * 60;
end

function cfg = setBatteryEnergyPercent(cfg, value_pct, baseCfg)
    cfg.batteryVoltage_V = baseCfg.batteryVoltage_V;
    cfg.batteryCapacity_Ah = baseCfg.batteryCapacity_Ah * value_pct / 100;
end

function cfg = setPayloadLitres(cfg, value_L, baseCfg)
    cfg.payloadLitres = value_L;
    cfg.initialWeightKg = baseCfg.initialWeightKg + (value_L - baseCfg.payloadLitres);
end

function cfg = setOverlapPercent(cfg, value_pct)
    cfg.overlap = value_pct / 100;
end

function cfg = setGroundStationOffsetFromActual(cfg, offset_m, baseCfg)
    fieldCentreXY = estimateFieldCentroidXY(baseCfg.fields);
    actualGsXY = [baseCfg.groundStationEast_m, baseCfg.groundStationNorth_m];
    directionXY = actualGsXY - fieldCentreXY;
    distance_m = norm(directionXY);
    if distance_m <= eps
        directionXY = [1 0];
    else
        directionXY = directionXY / distance_m;
    end

    newGsXY = actualGsXY + offset_m * directionXY;
    cfg.groundStationEast_m = newGsXY(1);
    cfg.groundStationNorth_m = newGsXY(2);
    cfg.sensitivityGroundStationOffset_m = offset_m;
end

function fieldCentreXY = estimateFieldCentroidXY(fields)
    allXY = zeros(0, 2);
    for iField = 1:numel(fields)
        if isfield(fields(iField), 'boundaryXY') && ~isempty(fields(iField).boundaryXY)
            allXY = [allXY; fields(iField).boundaryXY(:,1:2)]; %#ok<AGROW>
        elseif isfield(fields(iField), 'centreEast_m') && isfield(fields(iField), 'centreNorth_m')
            allXY = [allXY; fields(iField).centreEast_m, fields(iField).centreNorth_m]; %#ok<AGROW>
        end
    end
    if isempty(allXY)
        fieldCentreXY = [0 0];
    else
        fieldCentreXY = mean(allXY, 1, 'omitnan');
    end
end

function cfg = setPayloadTurnaroundMinutes(cfg, value_min)
    cfg.payloadTurnaroundTime_s = value_min * 60;
end

function cfg = setBatteryTurnaroundMinutes(cfg, value_min)
    cfg.batteryTurnaroundTime_s = value_min * 60;
end

function value = getOptionalStructScalar(s, fieldName, defaultValue)
    value = defaultValue;
    if isstruct(s) && isfield(s, fieldName) && isscalar(s.(fieldName)) && isfinite(s.(fieldName))
        value = s.(fieldName);
    end
end

function tf = isSameOptimisedCfg(a, b)
    tf = max(abs(getOptimisedCfgVector(a) - getOptimisedCfgVector(b))) <= 1e-9;
end

function v = getOptimisedCfgVector(cfg)
    v = [ ...
        cfg.maxRollAngle, ...
        cfg.turnaroundTime_s, ...
        cfg.batteryVoltage_V, ...
        cfg.batteryCapacity_Ah, ...
        cfg.payloadLitres, ...
        cfg.initialWeightKg, ...
        cfg.applicationRate, ...
        cfg.sprayWidth, ...
        cfg.uavAirSpeed, ...
        cfg.batteryReturnThreshold_pct, ...
        cfg.overlap, ...
        cfg.groundStationEast_m, ...
        cfg.groundStationNorth_m, ...
        cfg.revenuePerHa_GBP, ...
        cfg.workerRatePerHour_GBP, ...
        getOptionalStructScalar(cfg, 'payloadTurnaroundTime_s', cfg.turnaroundTime_s), ...
        getOptionalStructScalar(cfg, 'batteryTurnaroundTime_s', cfg.turnaroundTime_s), ...
        getOptionalStructScalar(cfg, 'sensitivityGroundStationOffset_m', 0)];
end

function row = makeHistoryTable(iteration, changedParameter, caseLabel, parameterValue, result)
    row = table(iteration, {changedParameter}, {caseLabel}, parameterValue, ...
        result.NetProfit_GBP, result.MissionTime_hr, result.HectaresPerHour, ...
        result.SprayTime_pct, result.SprayedArea_ha, result.Sorties, ...
        'VariableNames', {'Iteration','ChangedParameter','CaseLabel','ParameterValue', ...
        'NetProfit_GBP','MissionTime_hr','HectaresPerHour','SprayTime_pct', ...
        'SprayedArea_ha','Sorties'});
end

function row = makeCandidateRow(iteration, parameterName, caseLabel, parameterValue, result)
    row = table(iteration, {parameterName}, {caseLabel}, parameterValue, ...
        result.NetProfit_GBP, result.MissionTime_hr, result.HectaresPerHour, ...
        result.SprayTime_pct, result.SprayedArea_ha, result.Sorties, ...
        'VariableNames', {'Iteration','Parameter','CaseLabel','ParameterValue', ...
        'NetProfit_GBP','MissionTime_hr','HectaresPerHour','SprayTime_pct', ...
        'SprayedArea_ha','Sorties'});
end

function bestParameters = summariseBestParameters(cfg, result)
    bestParameters = table( ...
        result.NetProfit_GBP, result.MissionTime_hr, result.HectaresPerHour, ...
        result.SprayTime_pct, result.SprayedArea_ha, result.Sorties, ...
        result.MaxRollAngle_deg, result.Turnaround_min, result.BatteryEnergy_kWh, ...
        result.Payload_L, result.ApplicationRate_L_ha, result.SprayWidth_m, ...
        result.AirSpeed_mps, result.BatteryReserve_pct, result.Overlap_pct, ...
        result.GroundStationOffset_m, result.PayloadTurnaround_min, ...
        result.BatteryTurnaround_min, result.RevenuePerHa_GBP, ...
        result.WorkerRate_GBP_hr, cfg.groundStationEast_m, cfg.groundStationNorth_m, ...
        'VariableNames', {'NetProfit_GBP','MissionTime_hr','HectaresPerHour', ...
        'SprayTime_pct','SprayedArea_ha','Sorties','MaxRollAngle_deg', ...
        'Turnaround_min','BatteryEnergy_kWh','Payload_L','ApplicationRate_L_ha', ...
        'SprayWidth_m','AirSpeed_mps','BatteryReserve_pct','Overlap_pct', ...
        'GroundStationOffset_m','PayloadTurnaround_min','BatteryTurnaround_min', ...
        'RevenuePerHa_GBP','WorkerRate_GBP_hr','GroundStationEast_m', ...
        'GroundStationNorth_m'});
end

function plotOptimisationHistory(history, outputDir)
    fig = figure('Name', 'Mission parameter optimisation history', ...
        'Color', 'w', 'Position', [90 90 980 520]);
    ax = axes(fig);
    plot(ax, history.Iteration, history.NetProfit_GBP, '-o', ...
        'LineWidth', 1.6, 'MarkerFaceColor', [0.18 0.48 0.72]);
    grid(ax, 'on');
    box(ax, 'off');
    xlabel(ax, 'Accepted optimisation step');
    ylabel(ax, 'Net profit [GBP]');
    title(ax, 'Coordinate-search optimisation history', 'FontWeight', 'bold');
    exportgraphics(fig, fullfile(outputDir, 'optimisation_profit_history.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'optimisation_profit_history.fig'));
end

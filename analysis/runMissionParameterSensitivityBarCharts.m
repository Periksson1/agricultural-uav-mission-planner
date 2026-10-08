%% Mission parameter sensitivity bar charts
% Runs the current main mission once to capture the actual baseline, then
% reruns the same planner/simulator with one-at-a-time parameter changes.
%
% Output metrics:
%   - Time taken to complete mission [hr]
%   - Hectares per hour [ha/hr]
%   - Net profit [GBP]
%   - Percent of mission time spent spraying [%]

clc
close all

outputDir = fullfile(projectRoot(), 'outputs', 'economics', 'MissionParameterSensitivity');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

fprintf('Running current main mission once to capture the actual baseline...\n');
[baseCfg, basePlanningData, baseSimulationData] = runMainBaselineWithoutFigures();
baseResult = summariseCase(baseCfg, basePlanningData, baseSimulationData);

fprintf('\nActual main parameters used:\n');
fprintf('  Max roll angle      %.1f deg\n', baseResult.UsedRollAngle_deg);
fprintf('  Turnaround time     %.1f min\n', baseResult.UsedTurnaround_min);
fprintf('  Battery energy      %.3f kWh (%.1f V, %.1f Ah)\n', ...
    baseResult.UsedBatteryEnergy_kWh, baseCfg.batteryVoltage_V, baseCfg.batteryCapacity_Ah);
fprintf('  Payload             %.1f L\n', baseResult.UsedPayload_L);
fprintf('  Application rate    %.1f L/ha\n', baseResult.UsedApplicationRate_L_ha);
fprintf('  Airspeed            %.1f m/s\n', baseResult.UsedAirSpeed_mps);
fprintf('  Battery reserve     %.1f %%\n', baseResult.UsedBatteryReserve_pct);
fprintf('  Spray overlap       %.1f %%\n', baseResult.UsedOverlap_pct);
fprintf('  Spray width/nozzles %.1f m\n\n', baseResult.UsedSprayWidth_m);

sweeps = buildParameterSweeps(baseCfg);
allResults = table();

for iSweep = 1:numel(sweeps)
    sweep = sweeps(iSweep);
    fprintf('Running sweep: %s\n', sweep.name);

    sweepResults = runParameterSweep(baseCfg, baseResult, sweep);
    allResults = [allResults; sweepResults]; %#ok<AGROW>

    csvPath = fullfile(outputDir, sprintf('%s_results.csv', sweep.fileStem));
    writetable(sweepResults, csvPath);
    plotSweepBars(sweepResults, sweep, outputDir);
end

plotTornadoCharts(allResults, outputDir);
plotNetProfitDirectionCharts(allResults, outputDir);
writeNetProfitDirectionSummary(allResults, outputDir);

allCsvPath = fullfile(outputDir, 'all_parameter_sensitivity_results.csv');
writetable(allResults, allCsvPath);

fprintf('\nSaved parameter sensitivity results to:\n  %s\n', outputDir);
disp(allResults(:, {'Sweep','CaseLabel','ActualCase','VariedParameter', ...
    'UsedParameterValue','UsedParameterUnit','MissionTime_hr', ...
    'HectaresPerHour','NetProfit_GBP','SprayTime_pct'}));

%% Local helpers

function [baselineCfg, baselinePlanningData, baselineSimulationData] = runMainBaselineWithoutFigures()
    [baselineCfg, baselinePlanningData, baselineSimulationData] = runBaselineMission();
end

function sweepResults = runParameterSweep(baseCfg, baseResult, sweep)
    nCases = numel(sweep.cases);

    Sweep = repmat({sweep.name}, nCases, 1);
    CaseLabel = cell(nCases, 1);
    PlotLabel = cell(nCases, 1);
    VariedParameter = repmat({sweep.parameterName}, nCases, 1);
    RequestedParameterValue = nan(nCases, 1);
    RequestedParameterUnit = repmat({sweep.parameterUnit}, nCases, 1);
    ActualCase = false(nCases, 1);
    UsedParameterValue = nan(nCases, 1);
    UsedParameterUnit = repmat({sweep.usedParameterUnit}, nCases, 1);

    UsedRollAngle_deg = nan(nCases, 1);
    UsedTurnaround_min = nan(nCases, 1);
    UsedBatteryEnergy_kWh = nan(nCases, 1);
    UsedPayload_L = nan(nCases, 1);
    UsedApplicationRate_L_ha = nan(nCases, 1);
    UsedSprayWidth_m = nan(nCases, 1);
    UsedAirSpeed_mps = nan(nCases, 1);
    UsedBatteryReserve_pct = nan(nCases, 1);
    UsedOverlap_pct = nan(nCases, 1);
    UsedGroundStationOffset_m = nan(nCases, 1);
    UsedRevenuePerHa_GBP = nan(nCases, 1);
    UsedWorkerRate_GBP_hr = nan(nCases, 1);
    UsedPayloadTurnaround_min = nan(nCases, 1);
    UsedBatteryTurnaround_min = nan(nCases, 1);

    MissionTime_hr = nan(nCases, 1);
    HectaresPerHour = nan(nCases, 1);
    NetProfit_GBP = nan(nCases, 1);
    SprayTime_pct = nan(nCases, 1);
    SprayedArea_ha = nan(nCases, 1);
    Sorties = nan(nCases, 1);

    for iCase = 1:nCases
        thisCase = sweep.cases(iCase);
        CaseLabel{iCase} = thisCase.label;
        PlotLabel{iCase} = thisCase.plotLabel;
        RequestedParameterValue(iCase) = thisCase.value;
        ActualCase(iCase) = thisCase.isActual;

        if thisCase.isActual
            result = baseResult;
        else
            cfg = sweep.apply(baseCfg, thisCase.value);
            cfg = refreshDependentMissionSettings(cfg);

            try
                validateMissionCfg(cfg);
                planningData = runMissionPlanning(cfg);
                simulationData = runMissionSimulation(cfg, planningData);
                result = summariseCase(cfg, planningData, simulationData);
            catch ME
                warning('Sweep "%s", case "%s" failed: %s', ...
                    sweep.name, thisCase.label, ME.message);
                result = emptyCaseSummary(cfg);
            end
        end

        UsedParameterValue(iCase) = sweep.usedParameterValue(result);
        UsedRollAngle_deg(iCase) = result.UsedRollAngle_deg;
        UsedTurnaround_min(iCase) = result.UsedTurnaround_min;
        UsedBatteryEnergy_kWh(iCase) = result.UsedBatteryEnergy_kWh;
        UsedPayload_L(iCase) = result.UsedPayload_L;
        UsedApplicationRate_L_ha(iCase) = result.UsedApplicationRate_L_ha;
        UsedSprayWidth_m(iCase) = result.UsedSprayWidth_m;
        UsedAirSpeed_mps(iCase) = result.UsedAirSpeed_mps;
        UsedBatteryReserve_pct(iCase) = result.UsedBatteryReserve_pct;
        UsedOverlap_pct(iCase) = result.UsedOverlap_pct;
        UsedGroundStationOffset_m(iCase) = result.UsedGroundStationOffset_m;
        UsedRevenuePerHa_GBP(iCase) = result.UsedRevenuePerHa_GBP;
        UsedWorkerRate_GBP_hr(iCase) = result.UsedWorkerRate_GBP_hr;
        UsedPayloadTurnaround_min(iCase) = result.UsedPayloadTurnaround_min;
        UsedBatteryTurnaround_min(iCase) = result.UsedBatteryTurnaround_min;

        MissionTime_hr(iCase) = result.MissionTime_hr;
        HectaresPerHour(iCase) = result.HectaresPerHour;
        NetProfit_GBP(iCase) = result.NetProfit_GBP;
        SprayTime_pct(iCase) = result.SprayTime_pct;
        SprayedArea_ha(iCase) = result.SprayedArea_ha;
        Sorties(iCase) = result.Sorties;
    end

    sweepResults = table(Sweep, CaseLabel, PlotLabel, VariedParameter, ...
        RequestedParameterValue, RequestedParameterUnit, ActualCase, ...
        UsedParameterValue, UsedParameterUnit, UsedRollAngle_deg, ...
        UsedTurnaround_min, UsedBatteryEnergy_kWh, UsedPayload_L, ...
        UsedApplicationRate_L_ha, UsedSprayWidth_m, UsedAirSpeed_mps, ...
        UsedBatteryReserve_pct, UsedOverlap_pct, UsedGroundStationOffset_m, ...
        UsedRevenuePerHa_GBP, UsedWorkerRate_GBP_hr, UsedPayloadTurnaround_min, ...
        UsedBatteryTurnaround_min, MissionTime_hr, HectaresPerHour, ...
        NetProfit_GBP, SprayTime_pct, SprayedArea_ha, Sorties);
end

function result = summariseCase(cfg, ~, simulationData)
    result = emptyCaseSummary(cfg);
    result.MissionTime_hr = simulationData.totalTime / 3600;
    result.HectaresPerHour = simulationData.overallFieldCapacity_ha_per_hr;
    result.NetProfit_GBP = simulationData.netProfit_GBP;
    result.SprayTime_pct = simulationData.sprayTimeFraction_pct;
    result.SprayedArea_ha = simulationData.sprayedArea_ha(end);
    result.Sorties = simulationData.returnCount;
end

function result = emptyCaseSummary(cfg)
    result = struct();
    result.UsedRollAngle_deg = rad2deg(cfg.maxRollAngle);
    result.UsedTurnaround_min = cfg.turnaroundTime_s / 60;
    result.UsedBatteryEnergy_kWh = cfg.batteryVoltage_V * cfg.batteryCapacity_Ah / 1000;
    result.UsedPayload_L = cfg.payloadLitres;
    result.UsedApplicationRate_L_ha = cfg.applicationRate;
    result.UsedSprayWidth_m = cfg.sprayWidth;
    result.UsedAirSpeed_mps = cfg.uavAirSpeed;
    result.UsedBatteryReserve_pct = cfg.batteryReturnThreshold_pct;
    result.UsedOverlap_pct = 100 * cfg.overlap;
    result.UsedGroundStationOffset_m = getOptionalStructScalar(cfg, 'sensitivityGroundStationOffset_m', 0);
    result.UsedRevenuePerHa_GBP = cfg.revenuePerHa_GBP;
    result.UsedWorkerRate_GBP_hr = cfg.workerRatePerHour_GBP;
    result.UsedPayloadTurnaround_min = getOptionalStructScalar(cfg, 'payloadTurnaroundTime_s', cfg.turnaroundTime_s) / 60;
    result.UsedBatteryTurnaround_min = getOptionalStructScalar(cfg, 'batteryTurnaroundTime_s', cfg.turnaroundTime_s) / 60;
    result.MissionTime_hr = NaN;
    result.HectaresPerHour = NaN;
    result.NetProfit_GBP = NaN;
    result.SprayTime_pct = NaN;
    result.SprayedArea_ha = NaN;
    result.Sorties = NaN;
end

function cfg = refreshDependentMissionSettings(cfg)
    cfg.fieldCount = numel(cfg.fields);
    cfg.fieldVisitOrder = 1:cfg.fieldCount;
    cfg.planningOptions.shortLaneAbsoluteThreshold_m = max(2 * cfg.sprayWidth, 12);
    cfg.planningOptions.shortLanePenalty_m = 4 * cfg.sprayWidth;
    cfg.planningOptions = sanitizePlanningOptions(cfg.planningOptions);
end

function sweeps = buildParameterSweeps(baseCfg)
    baseBatteryEnergy_kWh = baseCfg.batteryVoltage_V * baseCfg.batteryCapacity_Ah / 1000;

    sweeps = struct('name', {}, 'fileStem', {}, 'parameterName', {}, ...
        'parameterUnit', {}, 'usedParameterUnit', {}, 'cases', {}, ...
        'apply', {}, 'usedParameterValue', {});

    sweeps(end+1) = makeSweep( ...
        'Impact of changing roll angle', 'roll_angle', ...
        'Max roll angle', 'deg requested', 'deg used', ...
        [makeCase('20 deg', '20 deg', 20, false), ...
         makeCase('25 deg', '25 deg', 25, false), ...
         makeCase('30 deg (actual)', '30 deg (actual)', 30, true), ...
         makeCase('40 deg', '40 deg', 40, false)], ...
        @(cfg, value) setMaxRollAngleDeg(cfg, value), ...
        @(result) result.UsedRollAngle_deg);

    sweeps(end+1) = makeSweep( ...
        'Impact of changing turnaround time', 'turnaround_time', ...
        'Turnaround time', 'min requested', 'min used', ...
        [makeCase('0 min', '0 min', 0, false), ...
         makeCase('1 min', '1 min', 1, false), ...
         makeCase('3 min (actual)', '3 min (actual)', 3, true), ...
         makeCase('5 min', '5 min', 5, false)], ...
        @(cfg, value) setTurnaroundMinutes(cfg, value), ...
        @(result) result.UsedTurnaround_min);

    sweeps(end+1) = makeSweep( ...
        'Impact of changing battery energy', 'battery_energy', ...
        'Battery energy', 'percent of current energy requested', 'kWh used', ...
        [makeCase(sprintf('100%% current (actual, %.2f kWh)', baseBatteryEnergy_kWh), ...
                  sprintf('100%% current\n(actual)'), 100, true), ...
         makeCase(sprintf('40%% current (%.2f kWh)', 0.40 * baseBatteryEnergy_kWh), ...
                  sprintf('40%% current\n%.2f kWh', 0.40 * baseBatteryEnergy_kWh), 40, false), ...
         makeCase(sprintf('75%% current (%.2f kWh)', 0.75 * baseBatteryEnergy_kWh), ...
                  sprintf('75%% current\n%.2f kWh', 0.75 * baseBatteryEnergy_kWh), 75, false), ...
         makeCase(sprintf('120%% current (%.2f kWh)', 1.20 * baseBatteryEnergy_kWh), ...
                  sprintf('120%% current\n%.2f kWh', 1.20 * baseBatteryEnergy_kWh), 120, false)], ...
        @(cfg, value) setBatteryEnergyPercent(cfg, value, baseCfg), ...
        @(result) result.UsedBatteryEnergy_kWh);

    sweeps(end+1) = makeSweep( ...
        'Impact of increasing payload', 'payload', ...
        'Payload', 'L requested', 'L used', ...
        [makeCase('30 L payload', '30 L', 30, false), ...
         makeCase('40 L payload', '40 L', 40, false), ...
         makeCase('47 L payload (actual)', '47 L (actual)', 47, true), ...
         makeCase('55 L payload', '55 L', 55, false)], ...
        @(cfg, value) setPayloadLitres(cfg, value, baseCfg), ...
        @(result) result.UsedPayload_L);

    sweeps(end+1) = makeSweep( ...
        'Impact of changing application rate', 'application_rate', ...
        'Application rate', 'L/ha requested', 'L/ha used', ...
        [makeCase('8 L/ha', '8 L/ha', 8, false), ...
         makeCase('12 L/ha', '12 L/ha', 12, false), ...
         makeCase('15 L/ha (actual)', '15 L/ha (actual)', 15, true), ...
         makeCase('20 L/ha', '20 L/ha', 20, false)], ...
        @(cfg, value) setField(cfg, 'applicationRate', value), ...
        @(result) result.UsedApplicationRate_L_ha);

    sweeps(end+1) = makeSweep( ...
        'Impact of changing UAV airspeed', 'airspeed', ...
        'UAV airspeed', 'm/s requested', 'm/s used', ...
        [makeCase('18 m/s', '18 m/s', 18, false), ...
         makeCase('22 m/s', '22 m/s', 22, false), ...
         makeCase(sprintf('%.1f m/s (actual)', baseCfg.uavAirSpeed), ...
                  sprintf('%.1f m/s (actual)', baseCfg.uavAirSpeed), baseCfg.uavAirSpeed, true), ...
         makeCase('30 m/s', '30 m/s', 30, false)], ...
        @(cfg, value) setField(cfg, 'uavAirSpeed', value), ...
        @(result) result.UsedAirSpeed_mps);

    sweeps(end+1) = makeSweep( ...
        'Impact of changing battery reserve threshold', 'battery_reserve_threshold', ...
        'Battery reserve threshold', '% requested', '% used', ...
        [makeCase('10%', '10%', 10, false), ...
         makeCase('15%', '15%', 15, false), ...
         makeCase('20% (actual)', '20% (actual)', 20, true), ...
         makeCase('25%', '25%', 25, false)], ...
        @(cfg, value) setField(cfg, 'batteryReturnThreshold_pct', value), ...
        @(result) result.UsedBatteryReserve_pct);

    sweeps(end+1) = makeSweep( ...
        'Impact of spray overlap', 'spray_overlap', ...
        'Spray overlap', '% requested', '% used', ...
        [makeCase('0% overlap (actual)', '0% (actual)', 0, true), ...
         makeCase('10% overlap', '10%', 10, false), ...
         makeCase('20% overlap', '20%', 20, false), ...
         makeCase('30% overlap', '30%', 30, false)], ...
        @(cfg, value) setOverlapPercent(cfg, value), ...
        @(result) result.UsedOverlap_pct);

    sweeps(end+1) = makeSweep( ...
        'Impact of ground station position', 'ground_station_position', ...
        'Ground station offset from actual', 'm requested', 'm used', ...
        [makeCase('250 m closer to field centre', '-250 m', -250, false), ...
         makeCase('Current GS position (actual)', '0 m (actual)', 0, true), ...
         makeCase('250 m farther from field centre', '+250 m', 250, false), ...
         makeCase('500 m farther from field centre', '+500 m', 500, false)], ...
        @(cfg, value) setGroundStationOffsetFromActual(cfg, value, baseCfg), ...
        @(result) result.UsedGroundStationOffset_m);

    sweeps(end+1) = makeSweep( ...
        'Impact of changing revenue per hectare', 'revenue_per_ha', ...
        'Revenue per hectare', 'GBP/ha requested', 'GBP/ha used', ...
        [makeCase('20 GBP/ha', '20 GBP/ha', 20, false), ...
         makeCase('27 GBP/ha (actual)', '27 GBP/ha (actual)', 27, true), ...
         makeCase('35 GBP/ha', '35 GBP/ha', 35, false), ...
         makeCase('45 GBP/ha', '45 GBP/ha', 45, false)], ...
        @(cfg, value) setField(cfg, 'revenuePerHa_GBP', value), ...
        @(result) result.UsedRevenuePerHa_GBP);

    sweeps(end+1) = makeSweep( ...
        'Impact of changing worker rate', 'worker_rate', ...
        'Worker rate', 'GBP/hr requested', 'GBP/hr used', ...
        [makeCase('15 GBP/hr', '15 GBP/hr', 15, false), ...
         makeCase('20 GBP/hr (actual)', '20 GBP/hr (actual)', 20, true), ...
         makeCase('25 GBP/hr', '25 GBP/hr', 25, false), ...
         makeCase('30 GBP/hr', '30 GBP/hr', 30, false)], ...
        @(cfg, value) setField(cfg, 'workerRatePerHour_GBP', value), ...
        @(result) result.UsedWorkerRate_GBP_hr);

    sweeps(end+1) = makeSweep( ...
        'Impact of payload refill turnaround time', 'payload_turnaround_time', ...
        'Payload refill turnaround time', 'min requested', 'min used', ...
        [makeCase('1 min payload refill', '1 min', 1, false), ...
         makeCase('2 min payload refill', '2 min', 2, false), ...
         makeCase('3 min payload refill (actual)', '3 min (actual)', 3, true), ...
         makeCase('5 min payload refill', '5 min', 5, false)], ...
        @(cfg, value) setPayloadTurnaroundMinutes(cfg, value, baseCfg), ...
        @(result) result.UsedPayloadTurnaround_min);

    sweeps(end+1) = makeSweep( ...
        'Impact of battery turnaround time', 'battery_turnaround_time', ...
        'Battery turnaround time', 'min requested', 'min used', ...
        [makeCase('1 min battery turnaround', '1 min', 1, false), ...
         makeCase('3 min battery turnaround (actual)', '3 min (actual)', 3, true), ...
         makeCase('5 min battery turnaround', '5 min', 5, false), ...
         makeCase('8 min battery turnaround', '8 min', 8, false)], ...
        @(cfg, value) setBatteryTurnaroundMinutes(cfg, value, baseCfg), ...
        @(result) result.UsedBatteryTurnaround_min);

    sweeps(end+1) = makeSweep( ...
        'Impact of wider spray nozzles', 'spray_width', ...
        'Spray width/nozzle width', 'm requested', 'm used', ...
        [makeCase('6 m (actual)', '6 m (actual)', 6, true), ...
         makeCase('8 m', '8 m', 8, false), ...
         makeCase('10 m', '10 m', 10, false), ...
         makeCase('12 m', '12 m', 12, false)], ...
        @(cfg, value) setField(cfg, 'sprayWidth', value), ...
        @(result) result.UsedSprayWidth_m);
end

function sweep = makeSweep(name, fileStem, parameterName, parameterUnit, ...
    usedParameterUnit, cases, applyFcn, usedParameterValueFcn)
    sweep = struct();
    sweep.name = name;
    sweep.fileStem = fileStem;
    sweep.parameterName = parameterName;
    sweep.parameterUnit = parameterUnit;
    sweep.usedParameterUnit = usedParameterUnit;
    sweep.cases = cases;
    sweep.apply = applyFcn;
    sweep.usedParameterValue = usedParameterValueFcn;
end

function caseDef = makeCase(label, plotLabel, value, isActual)
    caseDef = struct();
    caseDef.label = label;
    caseDef.plotLabel = plotLabel;
    caseDef.value = value;
    caseDef.isActual = isActual;
end

function cfg = setField(cfg, fieldName, value)
    cfg.(fieldName) = value;
end

function cfg = setMaxRollAngleDeg(cfg, value_deg)
    cfg.maxRollAngle = deg2rad(value_deg);
end

function cfg = setTurnaroundMinutes(cfg, value_min)
    cfg.turnaroundTime_s = value_min * 60;
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

function cfg = setPayloadTurnaroundMinutes(cfg, value_min, baseCfg)
    cfg.turnaroundTime_s = baseCfg.turnaroundTime_s;
    cfg.payloadTurnaroundTime_s = value_min * 60;
    cfg.batteryTurnaroundTime_s = getOptionalStructScalar(baseCfg, ...
        'batteryTurnaroundTime_s', baseCfg.turnaroundTime_s);
end

function cfg = setBatteryTurnaroundMinutes(cfg, value_min, baseCfg)
    cfg.turnaroundTime_s = baseCfg.turnaroundTime_s;
    cfg.payloadTurnaroundTime_s = getOptionalStructScalar(baseCfg, ...
        'payloadTurnaroundTime_s', baseCfg.turnaroundTime_s);
    cfg.batteryTurnaroundTime_s = value_min * 60;
end

function value = getOptionalStructScalar(s, fieldName, defaultValue)
    value = defaultValue;
    if isstruct(s) && isfield(s, fieldName) && isscalar(s.(fieldName)) && isfinite(s.(fieldName))
        value = s.(fieldName);
    end
end

function plotTornadoCharts(allResults, outputDir)
    metricFields = {'MissionTime_hr', 'HectaresPerHour', 'NetProfit_GBP', 'SprayTime_pct'};
    metricTitles = {'Mission time', 'Hectares per hour', 'Net profit', 'Time spent spraying'};

    sweepNames = unique(allResults.Sweep, 'stable');
    summary = table();

    fig = figure('Name', 'Sensitivity tornado charts', ...
        'Color', 'w', 'Position', [60 60 1420 920]);
    t = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Sensitivity tornado charts: percent change from actual case', ...
        'FontWeight', 'bold');

    for iMetric = 1:numel(metricFields)
        metricField = metricFields{iMetric};
        sweepLabels = cell(numel(sweepNames), 1);
        lowPct = nan(numel(sweepNames), 1);
        highPct = nan(numel(sweepNames), 1);
        baselineValue = nan(numel(sweepNames), 1);
        lowCase = cell(numel(sweepNames), 1);
        highCase = cell(numel(sweepNames), 1);

        for iSweep = 1:numel(sweepNames)
            rows = strcmp(allResults.Sweep, sweepNames{iSweep});
            actualRows = rows & allResults.ActualCase;
            values = allResults.(metricField)(rows);
            labels = allResults.CaseLabel(rows);
            sweepLabels{iSweep} = sweepNames{iSweep};

            if ~any(actualRows) || isempty(values)
                continue
            end

            actualValue = allResults.(metricField)(find(actualRows, 1));
            if ~isfinite(actualValue)
                continue
            end

            denominator = max(abs(actualValue), eps);
            pctChange = 100 * (values - actualValue) / denominator;
            finiteChange = isfinite(pctChange);
            if ~any(finiteChange)
                continue
            end
            finitePctChange = pctChange(finiteChange);
            finiteLabels = labels(finiteChange);
            [lowPct(iSweep), lowIdx] = min(finitePctChange);
            [highPct(iSweep), highIdx] = max(finitePctChange);
            baselineValue(iSweep) = actualValue;

            if ~isempty(lowIdx) && isfinite(lowPct(iSweep))
                lowCase{iSweep} = finiteLabels{lowIdx};
            end
            if ~isempty(highIdx) && isfinite(highPct(iSweep))
                highCase{iSweep} = finiteLabels{highIdx};
            end
        end

        thisMetricSummary = table( ...
            repmat(metricFields(iMetric), numel(sweepNames), 1), sweepLabels, ...
            baselineValue, lowPct, highPct, lowCase, highCase, ...
            'VariableNames', {'Metric','Sweep','ActualValue','MinChange_pct', ...
            'MaxChange_pct','MinCase','MaxCase'});
        summary = [summary; thisMetricSummary]; %#ok<AGROW>

        ax = nexttile(t);
        plotOneTornadoPanel(ax, sweepLabels, lowPct, highPct, lowCase, highCase, metricTitles{iMetric});
    end

    writetable(summary, fullfile(outputDir, 'tornado_sensitivity_summary.csv'));
    exportgraphics(fig, fullfile(outputDir, 'tornado_sensitivity_charts.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'tornado_sensitivity_charts.fig'));
end

function plotOneTornadoPanel(ax, sweepLabels, lowPct, highPct, lowCase, highCase, titleText)
    valid = isfinite(lowPct) & isfinite(highPct);
    sweepLabels = sweepLabels(valid);
    lowPct = lowPct(valid);
    highPct = highPct(valid);
    lowCase = lowCase(valid);
    highCase = highCase(valid);

    if isempty(lowPct)
        title(ax, titleText);
        text(ax, 0.5, 0.5, 'No valid sensitivity data', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
        return
    end

    impact = max(abs([lowPct highPct]), [], 2);
    [~, order] = sort(impact, 'ascend');
    lowPct = lowPct(order);
    highPct = highPct(order);
    sweepLabels = sweepLabels(order);
    lowCase = lowCase(order);
    highCase = highCase(order);
    y = 1:numel(sweepLabels);

    hold(ax, 'on');
    lowHandle = gobjects(1);
    highHandle = gobjects(1);
    for i = 1:numel(y)
        plot(ax, [lowPct(i) highPct(i)], [y(i) y(i)], '-', ...
            'Color', [0.30 0.42 0.58], 'LineWidth', 5);
        lowHandle = plot(ax, lowPct(i), y(i), 'o', 'MarkerFaceColor', [0.88 0.42 0.24], ...
            'MarkerEdgeColor', 'w', 'MarkerSize', 5);
        highHandle = plot(ax, highPct(i), y(i), 'o', 'MarkerFaceColor', [0.18 0.55 0.36], ...
            'MarkerEdgeColor', 'w', 'MarkerSize', 5);
        addEndpointLabel(ax, lowPct(i), y(i), lowCase{i}, -1);
        addEndpointLabel(ax, highPct(i), y(i), highCase{i}, 1);
    end
    xline(ax, 0, 'k-', 'LineWidth', 1.0);
    grid(ax, 'on');
    box(ax, 'off');
    set(ax, 'YTick', y, 'YTickLabel', sweepLabels);
    xlabel(ax, 'Change from actual [%]');
    title(ax, titleText, 'FontWeight', 'bold');
    ylim(ax, [0.5 numel(y) + 0.5]);
    legend(ax, [lowHandle highHandle], {'Lowest result case', 'Highest result case'}, ...
        'Location', 'best', 'Box', 'off');
end

function addEndpointLabel(ax, xValue, yValue, labelText, side)
    if isempty(labelText) || ~isfinite(xValue)
        return
    end
    if side < 0
        horizontalAlignment = 'right';
        xOffset = -0.8;
    else
        horizontalAlignment = 'left';
        xOffset = 0.8;
    end
    text(ax, xValue + xOffset, yValue, compactEndpointLabel(labelText), ...
        'HorizontalAlignment', horizontalAlignment, ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 7, ...
        'Color', [0.16 0.16 0.16], ...
        'Interpreter', 'none');
end

function labelText = compactEndpointLabel(labelText)
    labelText = regexprep(labelText, '\s*\(actual\)', '');
    labelText = regexprep(labelText, '\s*actual', '');
    labelText = regexprep(labelText, 'Impact of ', '');
    maxChars = 22;
    if numel(labelText) > maxChars
        labelText = [labelText(1:maxChars - 3) '...'];
    end
end

function plotNetProfitDirectionCharts(allResults, outputDir)
    sweepNames = unique(allResults.Sweep, 'stable');
    nSweeps = numel(sweepNames);
    nCols = 4;
    nRows = ceil(nSweeps / nCols);

    fig = figure('Name', 'Net profit directional response', ...
        'Color', 'w', 'Position', [60 60 1500 900]);
    t = tiledlayout(fig, nRows, nCols, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Net profit response by parameter value', 'FontWeight', 'bold');

    for iSweep = 1:nSweeps
        ax = nexttile(t);
        rows = strcmp(allResults.Sweep, sweepNames{iSweep});
        x = allResults.UsedParameterValue(rows);
        y = allResults.NetProfit_GBP(rows);
        labels = allResults.CaseLabel(rows);
        actualMask = allResults.ActualCase(rows);
        unitText = allResults.UsedParameterUnit{find(rows, 1)};

        valid = isfinite(x) & isfinite(y);
        x = x(valid);
        y = y(valid);
        labels = labels(valid);
        actualMask = actualMask(valid);

        if isempty(x)
            title(ax, sweepNames{iSweep}, 'Interpreter', 'none');
            text(ax, 0.5, 0.5, 'No valid data', ...
                'Units', 'normalized', 'HorizontalAlignment', 'center');
            continue
        end

        [x, order] = sort(x);
        y = y(order);
        labels = labels(order);
        actualMask = actualMask(order);

        plot(ax, x, y, '-o', 'Color', [0.25 0.43 0.65], ...
            'MarkerFaceColor', [0.25 0.43 0.65], 'LineWidth', 1.4, 'MarkerSize', 4);
        hold(ax, 'on');
        if any(actualMask)
            plot(ax, x(actualMask), y(actualMask), 'o', ...
                'MarkerFaceColor', [0.91 0.48 0.16], ...
                'MarkerEdgeColor', 'k', 'MarkerSize', 6);
        end
        [bestProfit, bestIdx] = max(y);
        plot(ax, x(bestIdx), bestProfit, 's', ...
            'MarkerFaceColor', [0.18 0.55 0.36], ...
            'MarkerEdgeColor', 'k', 'MarkerSize', 6);
        grid(ax, 'on');
        box(ax, 'off');
        xlabel(ax, unitText);
        ylabel(ax, 'Profit [GBP]');
        title(ax, sweepNames{iSweep}, 'Interpreter', 'none', 'FontSize', 9);
        text(ax, x(bestIdx), bestProfit, compactEndpointLabel(labels{bestIdx}), ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', ...
            'FontSize', 7, 'Interpreter', 'none');
    end

    exportgraphics(fig, fullfile(outputDir, 'net_profit_direction_charts.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'net_profit_direction_charts.fig'));
end

function writeNetProfitDirectionSummary(allResults, outputDir)
    sweepNames = unique(allResults.Sweep, 'stable');
    Sweep = cell(numel(sweepNames), 1);
    ActualCase = cell(numel(sweepNames), 1);
    ActualValue = nan(numel(sweepNames), 1);
    ActualProfit_GBP = nan(numel(sweepNames), 1);
    BestCase = cell(numel(sweepNames), 1);
    BestValue = nan(numel(sweepNames), 1);
    BestProfit_GBP = nan(numel(sweepNames), 1);
    WorstCase = cell(numel(sweepNames), 1);
    WorstValue = nan(numel(sweepNames), 1);
    WorstProfit_GBP = nan(numel(sweepNames), 1);
    DirectionFromActual = cell(numel(sweepNames), 1);

    for iSweep = 1:numel(sweepNames)
        rows = strcmp(allResults.Sweep, sweepNames{iSweep});
        x = allResults.UsedParameterValue(rows);
        y = allResults.NetProfit_GBP(rows);
        labels = allResults.CaseLabel(rows);
        actualMask = allResults.ActualCase(rows);

        Sweep{iSweep} = sweepNames{iSweep};
        valid = isfinite(x) & isfinite(y);
        x = x(valid);
        y = y(valid);
        labels = labels(valid);
        actualMask = actualMask(valid);
        if isempty(x)
            DirectionFromActual{iSweep} = 'No valid data';
            continue
        end

        [bestProfit, bestIdx] = max(y);
        [worstProfit, worstIdx] = min(y);
        BestCase{iSweep} = labels{bestIdx};
        BestValue(iSweep) = x(bestIdx);
        BestProfit_GBP(iSweep) = bestProfit;
        WorstCase{iSweep} = labels{worstIdx};
        WorstValue(iSweep) = x(worstIdx);
        WorstProfit_GBP(iSweep) = worstProfit;

        if any(actualMask)
            actualIdx = find(actualMask, 1);
            ActualCase{iSweep} = labels{actualIdx};
            ActualValue(iSweep) = x(actualIdx);
            ActualProfit_GBP(iSweep) = y(actualIdx);
            DirectionFromActual{iSweep} = describeProfitDirection(x, y, actualIdx);
        else
            DirectionFromActual{iSweep} = 'No actual case marker';
        end
    end

    summary = table(Sweep, ActualCase, ActualValue, ActualProfit_GBP, ...
        BestCase, BestValue, BestProfit_GBP, WorstCase, WorstValue, ...
        WorstProfit_GBP, DirectionFromActual);
    writetable(summary, fullfile(outputDir, 'net_profit_direction_summary.csv'));
end

function description = describeProfitDirection(x, y, actualIdx)
    actualX = x(actualIdx);
    actualY = y(actualIdx);
    lowerRows = x < actualX & isfinite(y);
    higherRows = x > actualX & isfinite(y);

    lowerBest = NaN;
    higherBest = NaN;
    if any(lowerRows)
        lowerBest = max(y(lowerRows));
    end
    if any(higherRows)
        higherBest = max(y(higherRows));
    end

    lowerImproves = isfinite(lowerBest) && lowerBest > actualY;
    higherImproves = isfinite(higherBest) && higherBest > actualY;

    if lowerImproves && higherImproves
        description = 'Both lower and higher tested values improved profit; response may be non-monotonic.';
    elseif lowerImproves
        description = 'Decreasing this parameter improved profit within the tested range.';
    elseif higherImproves
        description = 'Increasing this parameter improved profit within the tested range.';
    elseif isfinite(lowerBest) || isfinite(higherBest)
        description = 'Neither increasing nor decreasing improved profit within the tested range.';
    else
        description = 'No lower/higher cases available around actual.';
    end
end

function plotSweepBars(sweepResults, sweep, outputDir)
    metricFields = {'MissionTime_hr', 'HectaresPerHour', 'NetProfit_GBP', 'SprayTime_pct'};
    metricTitles = {'Time taken to complete mission', 'Hectares per hour', ...
        'Net profit', 'Time spent spraying'};
    metricYLabels = {'Time [hr]', 'Capacity [ha/hr]', 'Net profit [GBP]', ...
        'Spraying time [%]'};

    fig = figure('Name', sweep.name, 'Color', 'w', 'Position', [80 80 1280 760]);
    t = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, sprintf('%s (orange bar = actual main value used)', sweep.name), ...
        'FontWeight', 'bold');

    for iMetric = 1:numel(metricFields)
        ax = nexttile(t);
        y = sweepResults.(metricFields{iMetric});
        plotOneMetricBar(ax, y, sweepResults.PlotLabel, sweepResults.ActualCase, ...
            metricTitles{iMetric}, metricYLabels{iMetric});
    end

    pngPath = fullfile(outputDir, sprintf('%s_bar_charts.png', sweep.fileStem));
    figPath = fullfile(outputDir, sprintf('%s_bar_charts.fig', sweep.fileStem));
    exportgraphics(fig, pngPath, 'Resolution', 220);
    savefig(fig, figPath);
end

function plotOneMetricBar(ax, y, plotLabels, actualCase, titleText, yLabelText)
    x = 1:numel(y);
    b = bar(ax, x, y, 'FaceColor', 'flat');
    otherColor = [0.28 0.49 0.72];
    actualColor = [0.91 0.48 0.16];
    cData = repmat(otherColor, numel(y), 1);
    cData(actualCase, :) = repmat(actualColor, sum(actualCase), 1);
    b.CData = cData;

    grid(ax, 'on');
    box(ax, 'off');
    set(ax, 'XTick', x, 'XTickLabel', plotLabels);
    xtickangle(ax, 20);
    ylabel(ax, yLabelText);
    title(ax, titleText, 'FontWeight', 'bold');

    if contains(yLabelText, 'profit', 'IgnoreCase', true)
        yline(ax, 0, 'k-', 'LineWidth', 0.9);
    end

    finiteY = y(isfinite(y));
    if isempty(finiteY)
        return
    end

    yMin = min([finiteY(:); 0]);
    yMax = max([finiteY(:); 0]);
    ySpan = max(yMax - yMin, max(abs([yMin yMax])) * 0.1);
    if ySpan <= 0
        ySpan = 1;
    end
    ylim(ax, [yMin - 0.10 * ySpan, yMax + 0.18 * ySpan]);

    actualIdx = find(actualCase);
    for i = actualIdx(:)'
        if ~isfinite(y(i))
            continue
        end
        if y(i) >= 0
            yText = y(i) + 0.04 * ySpan;
            verticalAlignment = 'bottom';
        else
            yText = y(i) - 0.04 * ySpan;
            verticalAlignment = 'top';
        end
        text(ax, i, yText, 'actual', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', verticalAlignment, ...
            'FontWeight', 'bold', ...
            'Color', actualColor);
    end
end

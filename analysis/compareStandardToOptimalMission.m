%% Compare the baseline with a selected one-minute turnaround scenario.
% Aircraft parameters stay at their baseline values. This is a scenario
% comparison, not a claim of a globally optimal aircraft design.

clc
close all

outputDir = fullfile(projectRoot(), 'outputs', 'economics', 'StandardVsOptimal');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

fprintf('Running standard mission from main...\n');
[standardCfg, standardPlanningData, standardSimulationData] = runMainBaselineWithoutFigures();
standardSummary = summariseMissionCase('Standard', standardCfg, standardSimulationData);

fprintf('Running selected optimal mission settings...\n');
optimalCfg = buildSelectedOptimalCfg(standardCfg);
validateMissionCfg(optimalCfg);
optimalPlanningData = runMissionPlanning(optimalCfg);
optimalSimulationData = runMissionSimulation(optimalCfg, optimalPlanningData);
optimalSummary = summariseMissionCase('1-minute turnaround', optimalCfg, optimalSimulationData);

summary = [standardSummary; optimalSummary];
comparison = buildComparisonTable(summary);

writetable(summary, fullfile(outputDir, 'standard_vs_optimal_summary.csv'));
writetable(comparison, fullfile(outputDir, 'standard_vs_optimal_change.csv'));
plotStandardVsOptimalBars(summary, outputDir);

fprintf('\nSaved standard-vs-optimal comparison outputs to:\n  %s\n', outputDir);
disp(summary(:, {'Case','MaxRollAngle_deg','Turnaround_min','Payload_L', ...
    'AirSpeed_mps','SprayWidth_m','MissionTime_hr','HectaresPerHour', ...
    'NetProfit_GBP','SprayTime_pct'}));
disp(comparison);

%% Local helpers

function [baselineCfg, baselinePlanningData, baselineSimulationData] = runMainBaselineWithoutFigures()
    [baselineCfg, baselinePlanningData, baselineSimulationData] = runBaselineMission();
end

function cfg = buildSelectedOptimalCfg(baseCfg)
    cfg = baseCfg;

    cfg.maxRollAngle = deg2rad(30);
    cfg.turnaroundTime_s = 60;
    cfg.payloadTurnaroundTime_s = 60;
    cfg.batteryTurnaroundTime_s = 60;
    cfg.payloadLitres = 47;
    cfg.initialWeightKg = baseCfg.initialWeightKg + (cfg.payloadLitres - baseCfg.payloadLitres);
    cfg.uavAirSpeed = 25.8;
    cfg.sprayWidth = 6;

    cfg.planningOptions.shortLaneAbsoluteThreshold_m = max(2 * cfg.sprayWidth, 12);
    cfg.planningOptions.shortLanePenalty_m = 4 * cfg.sprayWidth;
    cfg.planningOptions = sanitizePlanningOptions(cfg.planningOptions);
end

function summary = summariseMissionCase(caseName, cfg, simulationData)
    Case = {caseName};
    MaxRollAngle_deg = rad2deg(cfg.maxRollAngle);
    Turnaround_min = cfg.turnaroundTime_s / 60;
    Payload_L = cfg.payloadLitres;
    AirSpeed_mps = cfg.uavAirSpeed;
    SprayWidth_m = cfg.sprayWidth;
    BatteryEnergy_kWh = cfg.batteryVoltage_V * cfg.batteryCapacity_Ah / 1000;
    ApplicationRate_L_ha = cfg.applicationRate;
    Overlap_pct = 100 * cfg.overlap;
    BatteryReserve_pct = cfg.batteryReturnThreshold_pct;
    MissionTime_hr = simulationData.totalTime / 3600;
    HectaresPerHour = simulationData.overallFieldCapacity_ha_per_hr;
    NetProfit_GBP = simulationData.netProfit_GBP;
    SprayTime_pct = simulationData.sprayTimeFraction_pct;
    SprayedArea_ha = simulationData.sprayedArea_ha(end);
    Sorties = simulationData.returnCount;
    Revenue_GBP = simulationData.totalRevenue_GBP;
    TotalCost_GBP = simulationData.totalCost_GBP;
    LabourCost_GBP = simulationData.labourCost_GBP;
    BatteryChargingCost_GBP = simulationData.batteryChargingCost_GBP;
    MaintenanceCost_GBP = simulationData.maintenanceCost_GBP;

    summary = table(Case, MaxRollAngle_deg, Turnaround_min, Payload_L, ...
        AirSpeed_mps, SprayWidth_m, BatteryEnergy_kWh, ApplicationRate_L_ha, ...
        Overlap_pct, BatteryReserve_pct, MissionTime_hr, HectaresPerHour, ...
        NetProfit_GBP, SprayTime_pct, SprayedArea_ha, Sorties, Revenue_GBP, ...
        TotalCost_GBP, LabourCost_GBP, BatteryChargingCost_GBP, MaintenanceCost_GBP);
end

function comparison = buildComparisonTable(summary)
    metricNames = {'MissionTime_hr'; 'HectaresPerHour'; 'NetProfit_GBP'; ...
        'SprayTime_pct'; 'SprayedArea_ha'; 'Sorties'; 'Revenue_GBP'; 'TotalCost_GBP'};
    StandardValue = nan(numel(metricNames), 1);
    OptimalValue = nan(numel(metricNames), 1);
    AbsoluteChange = nan(numel(metricNames), 1);
    PercentChange = nan(numel(metricNames), 1);

    for iMetric = 1:numel(metricNames)
        metric = metricNames{iMetric};
        StandardValue(iMetric) = summary.(metric)(1);
        OptimalValue(iMetric) = summary.(metric)(2);
        AbsoluteChange(iMetric) = OptimalValue(iMetric) - StandardValue(iMetric);
        PercentChange(iMetric) = 100 * AbsoluteChange(iMetric) / max(abs(StandardValue(iMetric)), eps);
    end

    comparison = table(metricNames, StandardValue, OptimalValue, ...
        AbsoluteChange, PercentChange, ...
        'VariableNames', {'Metric','StandardValue','OptimalValue', ...
        'AbsoluteChange','PercentChange'});
end

function plotStandardVsOptimalBars(summary, outputDir)
    metricFields = {'MissionTime_hr', 'HectaresPerHour', 'NetProfit_GBP', 'SprayTime_pct'};
    metricTitles = {'Mission time', 'Hectares per hour', 'Net profit', 'Time spent spraying'};
    metricYLabels = {'Time [hr]', 'Capacity [ha/hr]', 'Net profit [GBP]', 'Spraying time [%]'};

    fig = figure('Name', 'Standard vs selected optimal mission', ...
        'Color', 'w', 'Position', [80 80 1220 720]);
    t = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Standard mission vs selected optimal settings', 'FontWeight', 'bold');

    for iMetric = 1:numel(metricFields)
        ax = nexttile(t);
        y = summary.(metricFields{iMetric});
        x = 1:height(summary);
        b = bar(ax, x, y, 'FaceColor', 'flat');
        b.CData = [0.28 0.49 0.72; 0.18 0.55 0.36];
        grid(ax, 'on');
        box(ax, 'off');
        set(ax, 'XTick', x, 'XTickLabel', summary.Case);
        ylabel(ax, metricYLabels{iMetric});
        title(ax, metricTitles{iMetric}, 'FontWeight', 'bold');
        labelBars(ax, x, y);
    end

    exportgraphics(fig, fullfile(outputDir, 'standard_vs_optimal_bar_charts.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'standard_vs_optimal_bar_charts.fig'));
end

function labelBars(ax, x, y)
    finiteY = y(isfinite(y));
    if isempty(finiteY)
        return
    end
    yMin = min([finiteY; 0]);
    yMax = max([finiteY; 0]);
    ySpan = yMax - yMin;
    if ySpan <= 0
        ySpan = max(abs(finiteY));
    end
    if ySpan <= 0
        ySpan = 1;
    end
    ylim(ax, [yMin - 0.08 * ySpan, yMax + 0.18 * ySpan]);
    for i = 1:numel(y)
        if ~isfinite(y(i))
            continue
        end
        text(ax, x(i), y(i) + 0.035 * ySpan, sprintf('%.2f', y(i)), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontSize', 8);
    end
end

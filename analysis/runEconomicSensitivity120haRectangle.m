%% Sensitivity analysis for a 120 ha rectangular spraying mission
% Uses the existing mission planner/simulator with one-at-a-time changes
% around a 120 ha, 2:1 aspect-ratio rectangular baseline.

clc
close all

outputDir = fullfile(projectRoot(), 'outputs', 'economics', 'Sensitivity120haRectangle');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

baseCfg = buildSensitivityBaseMissionCfg();

sweeps = buildSensitivitySweeps();
allResults = table();

for iSweep = 1:numel(sweeps)
    sweep = sweeps(iSweep);
    fprintf('Running sensitivity sweep: %s\n', sweep.name);
    sweepResults = runSweep(baseCfg, sweep);
    allResults = [allResults; sweepResults]; %#ok<AGROW>
    writetable(sweepResults, fullfile(outputDir, sprintf('%s_results.csv', sweep.fileStem)));
end

writetable(allResults, fullfile(outputDir, 'all_sensitivity_results.csv'));

plotSensitivityProfit(allResults, outputDir);
plotSensitivityBreakEven(allResults, outputDir);
plotCombinedSensitivitySummary(allResults, outputDir);

fprintf('\nSaved 120 ha rectangle sensitivity outputs to:\n  %s\n', outputDir);
disp(allResults(:, {'Sweep','CaseLabel','Area_ha','MissionTime_hr','GrossProfit_GBP', ...
    'GrossProfitPerHa_GBP','BreakEvenPricePerHa_GBP','NonProductiveTime_pct'}));

%% Local helpers

function sweepResults = runSweep(baseCfg, sweep)
    nCases = numel(sweep.values);

    Sweep = repmat({sweep.name}, nCases, 1);
    CaseLabel = cell(nCases, 1);
    ParameterValue = nan(nCases, 1);
    Area_ha = nan(nCases, 1);
    MissionTime_hr = nan(nCases, 1);
    Revenue_GBP = nan(nCases, 1);
    TotalCost_GBP = nan(nCases, 1);
    LabourCost_GBP = nan(nCases, 1);
    MaintenanceCost_GBP = nan(nCases, 1);
    GrossProfit_GBP = nan(nCases, 1);
    GrossProfitPerHa_GBP = nan(nCases, 1);
    BreakEvenPricePerHa_GBP = nan(nCases, 1);
    OverallCapacity_ha_per_hr = nan(nCases, 1);
    NonProductiveTime_pct = nan(nCases, 1);
    TurnsPerHa = nan(nCases, 1);
    Sorties = nan(nCases, 1);

    for iCase = 1:nCases
        cfg = baseCfg;
        value = sweep.values(iCase);
        cfg = sweep.apply(cfg, value);
        cfg.planningOptions.shortLaneAbsoluteThreshold_m = max(2*cfg.sprayWidth, 12);
        cfg.planningOptions.shortLanePenalty_m = 4 * cfg.sprayWidth;
        cfg.planningOptions = sanitizePlanningOptions(cfg.planningOptions);

        CaseLabel{iCase} = sprintf(sweep.labelFormat, value);
        ParameterValue(iCase) = value;

        try
            validateMissionCfg(cfg);
            planningData = runMissionPlanning(cfg);
            simulationData = runMissionSimulation(cfg, planningData);

            Area_ha(iCase) = simulationData.sprayedArea_ha(end);
            MissionTime_hr(iCase) = simulationData.totalTime / 3600;
            Revenue_GBP(iCase) = simulationData.totalRevenue_GBP;
            TotalCost_GBP(iCase) = simulationData.totalCost_GBP;
            LabourCost_GBP(iCase) = simulationData.labourCost_GBP;
            MaintenanceCost_GBP(iCase) = simulationData.maintenanceCost_GBP;
            GrossProfit_GBP(iCase) = simulationData.netProfit_GBP;
            GrossProfitPerHa_GBP(iCase) = simulationData.netProfit_GBP / max(Area_ha(iCase), eps);
            BreakEvenPricePerHa_GBP(iCase) = simulationData.totalCost_GBP / max(Area_ha(iCase), eps);
            OverallCapacity_ha_per_hr(iCase) = simulationData.overallFieldCapacity_ha_per_hr;
            NonProductiveTime_pct(iCase) = simulationData.nonProductiveTime_pct;
            TurnsPerHa(iCase) = simulationData.turnsPerHa;
            Sorties(iCase) = simulationData.returnCount;
        catch ME
            warning('Sweep "%s", case "%s" failed: %s', sweep.name, CaseLabel{iCase}, ME.message);
        end
    end

    sweepResults = table(Sweep, CaseLabel, ParameterValue, Area_ha, MissionTime_hr, ...
        Revenue_GBP, TotalCost_GBP, LabourCost_GBP, MaintenanceCost_GBP, ...
        GrossProfit_GBP, GrossProfitPerHa_GBP, BreakEvenPricePerHa_GBP, ...
        OverallCapacity_ha_per_hr, NonProductiveTime_pct, TurnsPerHa, Sorties);
end

function sweeps = buildSensitivitySweeps()
    sweeps = repmat(struct('name', '', 'fileStem', '', 'values', [], ...
        'labelFormat', '', 'apply', []), 2, 1);

    sweeps(1).name = 'Spraying price';
    sweeps(1).fileStem = 'spraying_price';
    sweeps(1).values = [8 10 12 14 16 18 20 24];
    sweeps(1).labelFormat = '%.0f GBP/ha';
    sweeps(1).apply = @(cfg, value) setField(cfg, 'revenuePerHa_GBP', value);

    sweeps(2).name = 'Worker rate';
    sweeps(2).fileStem = 'worker_rate';
    sweeps(2).values = [10 15 20 25 30 35 40];
    sweeps(2).labelFormat = '%.0f GBP/hr';
    sweeps(2).apply = @(cfg, value) setField(cfg, 'workerRatePerHour_GBP', value);
end

function cfg = setField(cfg, fieldName, value)
    cfg.(fieldName) = value;
end

function plotSensitivityProfit(allResults, outputDir)
    fig = figure('Name', '120 ha rectangle sensitivity - gross profit', ...
        'Color', 'w', 'Position', [90 90 1120 520]);
    t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, '120 ha rectangle sensitivity: mission gross profit');

    plotSweepPanel(t, allResults, 'Spraying price', 'Price [GBP/ha]', 'GrossProfit_GBP', ...
        'Gross profit [GBP]');
    plotSweepPanel(t, allResults, 'Worker rate', 'Worker rate [GBP/hr]', 'GrossProfit_GBP', ...
        'Gross profit [GBP]');

    exportgraphics(fig, fullfile(outputDir, 'sensitivity_gross_profit.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'sensitivity_gross_profit.fig'));
end

function plotSensitivityBreakEven(allResults, outputDir)
    fig = figure('Name', '120 ha rectangle sensitivity - break-even price', ...
        'Color', 'w', 'Position', [100 100 1120 520]);
    t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, '120 ha rectangle sensitivity: break-even price');

    plotSweepPanel(t, allResults, 'Spraying price', 'Price [GBP/ha]', 'BreakEvenPricePerHa_GBP', ...
        'Break-even price [GBP/ha]');
    plotSweepPanel(t, allResults, 'Worker rate', 'Worker rate [GBP/hr]', 'BreakEvenPricePerHa_GBP', ...
        'Break-even price [GBP/ha]');

    exportgraphics(fig, fullfile(outputDir, 'sensitivity_break_even_price.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'sensitivity_break_even_price.fig'));
end

function plotCombinedSensitivitySummary(allResults, outputDir)
    fig = figure('Name', '120 ha rectangle sensitivity summary', ...
        'Color', 'w', 'Position', [100 100 1180 620]);
    t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, '120 ha rectangle sensitivity summary');

    plotSweepPanel(t, allResults, 'Spraying price', 'Price [GBP/ha]', 'GrossProfit_GBP', ...
        'Gross profit [GBP]');
    plotSweepPanel(t, allResults, 'Worker rate', 'Worker rate [GBP/hr]', 'GrossProfit_GBP', ...
        'Gross profit [GBP]');

    exportgraphics(fig, fullfile(outputDir, 'sensitivity_summary.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'sensitivity_summary.fig'));
end

function plotSweepPanel(t, allResults, sweepName, xLabelText, yVariable, yLabelText)
    ax = nexttile(t);
    rows = strcmp(allResults.Sweep, sweepName);
    x = allResults.ParameterValue(rows);
    y = allResults.(yVariable)(rows);
    plot(ax, x, y, '-o', 'LineWidth', 1.5, 'MarkerSize', 5);
    yline(ax, 0, 'k-', 'LineWidth', 1.0);
    grid(ax, 'on');
    xlabel(ax, xLabelText);
    ylabel(ax, yLabelText);
    title(ax, sweepName);
end

function cfg = buildSensitivityBaseMissionCfg()
    % Share aircraft/planner defaults; retain the assumptions of this study.
    cfg = defaultMissionConfig();
    cfg.fields = makeAspectRatioField('120 ha rectangle 2:1 baseline', 120, 2);
    cfg.fieldCount = 1;
    cfg.fieldVisitOrder = 1;
    cfg.useInteractiveLayoutDesigner = false;
    cfg.groundStationNorth_m = -1300;
    cfg.groundStationEast_m = 0;
    cfg.geodata = struct();
    cfg.revenuePerHa_GBP = 16;
    cfg.displayDsTransit = 5;
    cfg.playbackFPS = 30;
    cfg.targetPlaybackDurationFull_s = 120;
    cfg.viewAzimuth_deg = -35;
    cfg.viewElevation_deg = 30;
    cfg.verticalExaggeration = 1;
end

function field = makeAspectRatioField(name, area_ha, aspectRatio)
    area_m2 = area_ha * 10000;
    width_m = sqrt(area_m2 / aspectRatio);
    length_m = aspectRatio * width_m;
    field = makeRectangleField(name, 0, 0, length_m, width_m, 0);
end

function field = makeRectangleField(name, centreEast_m, centreNorth_m, length_m, width_m, rotation_deg)
    field = struct( ...
        'name', name, ...
        'type', 'rectangle', ...
        'boundaryXY', [], ...
        'centreEast_m', centreEast_m, ...
        'centreNorth_m', centreNorth_m, ...
        'length_m', length_m, ...
        'width_m', width_m, ...
        'rotation_deg', rotation_deg, ...
        'area_ha', NaN);
end

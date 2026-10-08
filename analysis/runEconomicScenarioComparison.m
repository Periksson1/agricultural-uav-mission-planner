%% Economic scenario comparison for per-mission spraying profitability
% Runs a fixed set of field layouts through the existing planner/simulator
% and produces report-ready figures plus a CSV summary table.

clc
close all

outputDir = fullfile(projectRoot(), 'outputs', 'economics');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

baseCfg = buildBaseMissionCfg();
targetArea_ha = 120.0;
scenarios = buildEconomicScenarios(targetArea_ha);

nScenarios = numel(scenarios);
scenarioNames = cell(nScenarios, 1);
scenarioDescriptions = cell(nScenarios, 1);
area_ha = nan(nScenarios, 1);
missionTime_hr = nan(nScenarios, 1);
sprayTime_hr = nan(nScenarios, 1);
overallCapacity_ha_per_hr = nan(nScenarios, 1);
nonProductiveTime_pct = nan(nScenarios, 1);
turnsPerHa = nan(nScenarios, 1);
sorties = nan(nScenarios, 1);
revenue_GBP = nan(nScenarios, 1);
transportCost_GBP = nan(nScenarios, 1);
labourCost_GBP = nan(nScenarios, 1);
batteryChargingCost_GBP = nan(nScenarios, 1);
maintenanceCost_GBP = nan(nScenarios, 1);
totalCost_GBP = nan(nScenarios, 1);
grossProfit_GBP = nan(nScenarios, 1);
grossProfitPerHa_GBP = nan(nScenarios, 1);
grossMargin_pct = nan(nScenarios, 1);
breakEvenPricePerHa_GBP = nan(nScenarios, 1);

planningResults = cell(nScenarios, 1);
simulationResults = cell(nScenarios, 1);

fprintf('Running %d economic field scenarios...\n', nScenarios);

for iScenario = 1:nScenarios
    scenario = scenarios(iScenario);
    cfg = baseCfg;
    cfg.fields = scenario.fields;
    cfg.fieldCount = numel(scenario.fields);
    cfg.fieldVisitOrder = 1:cfg.fieldCount;

    scenarioNames{iScenario} = scenario.name;
    scenarioDescriptions{iScenario} = scenario.description;
    transportCost_GBP(iScenario) = cfg.transportCost_GBP;

    fprintf('  %d/%d: %s\n', iScenario, nScenarios, scenario.name);

    try
        validateMissionCfg(cfg);
        planningData = runMissionPlanning(cfg);
        simulationData = runMissionSimulation(cfg, planningData);

        planningResults{iScenario} = planningData;
        simulationResults{iScenario} = simulationData;

        area_ha(iScenario) = simulationData.sprayedArea_ha(end);
        missionTime_hr(iScenario) = simulationData.totalTime / 3600;
        sprayTime_hr(iScenario) = simulationData.sprayTime / 3600;
        overallCapacity_ha_per_hr(iScenario) = simulationData.overallFieldCapacity_ha_per_hr;
        nonProductiveTime_pct(iScenario) = simulationData.nonProductiveTime_pct;
        turnsPerHa(iScenario) = simulationData.turnsPerHa;
        sorties(iScenario) = simulationData.returnCount;
        revenue_GBP(iScenario) = simulationData.totalRevenue_GBP;
        labourCost_GBP(iScenario) = simulationData.labourCost_GBP;
        batteryChargingCost_GBP(iScenario) = simulationData.batteryChargingCost_GBP;
        maintenanceCost_GBP(iScenario) = simulationData.maintenanceCost_GBP;
        totalCost_GBP(iScenario) = simulationData.totalCost_GBP;
        grossProfit_GBP(iScenario) = simulationData.netProfit_GBP;
        grossProfitPerHa_GBP(iScenario) = simulationData.netProfit_GBP / max(area_ha(iScenario), eps);
        grossMargin_pct(iScenario) = 100 * simulationData.netProfit_GBP / max(simulationData.totalRevenue_GBP, eps);
        breakEvenPricePerHa_GBP(iScenario) = simulationData.totalCost_GBP / max(area_ha(iScenario), eps);
    catch ME
        warning('Scenario "%s" failed: %s', scenario.name, ME.message);
    end
end

targetIdx = find(contains(scenarioNames, 'rectangle baseline'), 1);
if isempty(targetIdx)
    targetIdx = find(contains(scenarioNames, 'square 8hr target'), 1);
end
if isempty(targetIdx)
    targetIdx = 1;
end
grossProfitChangeVsTarget_GBP = grossProfit_GBP - grossProfit_GBP(targetIdx);
grossProfitChangeVsTarget_pct = 100 * grossProfitChangeVsTarget_GBP / max(abs(grossProfit_GBP(targetIdx)), eps);
profitPerHaChangeVsTarget_GBP = grossProfitPerHa_GBP - grossProfitPerHa_GBP(targetIdx);
profitPerHaChangeVsTarget_pct = 100 * profitPerHaChangeVsTarget_GBP / max(abs(grossProfitPerHa_GBP(targetIdx)), eps);

results = table( ...
    scenarioNames, scenarioDescriptions, area_ha, missionTime_hr, sprayTime_hr, ...
    overallCapacity_ha_per_hr, nonProductiveTime_pct, turnsPerHa, sorties, ...
    revenue_GBP, transportCost_GBP, labourCost_GBP, batteryChargingCost_GBP, ...
    maintenanceCost_GBP, totalCost_GBP, grossProfit_GBP, grossProfitPerHa_GBP, ...
    grossMargin_pct, breakEvenPricePerHa_GBP, grossProfitChangeVsTarget_GBP, ...
    grossProfitChangeVsTarget_pct, profitPerHaChangeVsTarget_GBP, profitPerHaChangeVsTarget_pct, ...
    'VariableNames', {'Scenario','Description','Area_ha','MissionTime_hr','SprayTime_hr', ...
    'OverallCapacity_ha_per_hr','NonProductiveTime_pct','TurnsPerHa','Sorties', ...
    'Revenue_GBP','TransportCost_GBP','LabourCost_GBP','BatteryChargingCost_GBP', ...
    'MaintenanceCost_GBP','TotalCost_GBP','GrossProfit_GBP','GrossProfitPerHa_GBP', ...
    'GrossMargin_pct','BreakEvenPricePerHa_GBP','GrossProfitChangeVsTarget_GBP', ...
    'GrossProfitChangeVsTarget_pct','ProfitPerHaChangeVsTarget_GBP','ProfitPerHaChangeVsTarget_pct'});

csvPath = fullfile(outputDir, 'economic_scenario_results.csv');
writetable(results, csvPath);

plotScenarioLayouts(scenarios, baseCfg, outputDir);
plotProfitComparison(results, outputDir);
plotCostBreakdown(results, outputDir);
plotCapacityComparison(results, outputDir);
plotBreakEvenComparison(results, baseCfg, outputDir);
plotPriceSensitivity(results, outputDir);
plotTargetBenchmarkComparison(results, outputDir);
plotScatterPack(results, outputDir);

fprintf('\nSaved economic scenario results to:\n  %s\n', csvPath);
disp(results(:, {'Scenario','Area_ha','MissionTime_hr','Revenue_GBP','TotalCost_GBP', ...
    'GrossProfit_GBP','GrossProfitPerHa_GBP','BreakEvenPricePerHa_GBP','NonProductiveTime_pct'}));

%% Local helpers

function cfg = buildBaseMissionCfg()
    % Share aircraft/planner defaults; retain the assumptions of this study.
    cfg = defaultMissionConfig();
    cfg.fields = makeRectangleField('Baseline field', 0, 0, 316.23, 316.23, 0);
    cfg.fieldCount = 1;
    cfg.fieldVisitOrder = 1;
    cfg.useInteractiveLayoutDesigner = false;
    cfg.groundStationNorth_m = -1300;
    cfg.groundStationEast_m = 0;
    cfg.geodata = struct();
    cfg.revenuePerHa_GBP = 20;
    cfg.displayDsTransit = 5;
    cfg.playbackFPS = 30;
    cfg.targetPlaybackDurationFull_s = 120;
    cfg.viewAzimuth_deg = -35;
    cfg.viewElevation_deg = 30;
    cfg.verticalExaggeration = 1;
    cfg.transportCost_GBP = 150;
    cfg.batteryChargeCostPerReturn_GBP = 2;
    cfg.maintenanceRatePerFlightHour_GBP = 50;
end

function scenarios = buildEconomicScenarios(targetArea_ha)
    scenarios = repmat(struct('name', '', 'description', '', 'fields', []), 12, 1);

    scenarios(1).name = '2 ha square';
    scenarios(1).description = 'Very small square field; tests minimum viable job size.';
    scenarios(1).fields = makeSquareField('2 ha square', 2);

    scenarios(2).name = '5 ha square';
    scenarios(2).description = 'Small square field.';
    scenarios(2).fields = makeSquareField('5 ha square', 5);

    scenarios(3).name = '10 ha square';
    scenarios(3).description = 'Small-medium square field.';
    scenarios(3).fields = makeSquareField('10 ha square', 10);

    scenarios(4).name = '20 ha square';
    scenarios(4).description = 'Medium square field.';
    scenarios(4).fields = makeSquareField('20 ha square', 20);

    scenarios(5).name = '40 ha square';
    scenarios(5).description = 'Large square field below target.';
    scenarios(5).fields = makeSquareField('40 ha square', 40);

    scenarios(6).name = '80 ha square';
    scenarios(6).description = 'Large square field at the original 80 ha target scale.';
    scenarios(6).fields = makeSquareField('80 ha square', 80);

    scenarios(7).name = sprintf('%.0f ha square', targetArea_ha);
    scenarios(7).description = 'Fixed large square comparison field.';
    scenarios(7).fields = makeSquareField(scenarios(7).name, targetArea_ha);

    scenarios(8).name = sprintf('%.0f ha rectangle baseline', targetArea_ha);
    scenarios(8).description = 'Same 120 ha area as a typical 2:1 rectangular farm block.';
    scenarios(8).fields = makeAspectRatioField(scenarios(8).name, targetArea_ha, 2);

    scenarios(9).name = sprintf('%.0f ha long rectangle', targetArea_ha);
    scenarios(9).description = 'Same 120 ha area as a long 8:1 rectangular block.';
    scenarios(9).fields = makeAspectRatioField(scenarios(9).name, targetArea_ha, 8);

    scenarios(10).name = sprintf('%.0f ha trapezium', targetArea_ha);
    scenarios(10).description = 'Same 120 ha area with a tapered non-rectangular boundary.';
    scenarios(10).fields = makeTrapeziumField(scenarios(10).name, targetArea_ha);

    scenarios(11).name = sprintf('%.0f ha L-shape', targetArea_ha);
    scenarios(11).description = 'Same 120 ha area with a concave L-shaped boundary.';
    scenarios(11).fields = makeLShapeField(scenarios(11).name, targetArea_ha);

    scenarios(12).name = sprintf('%.0f ha irregular polygon', targetArea_ha);
    scenarios(12).description = 'Same 120 ha area with an irregular realistic field boundary.';
    scenarios(12).fields = makeIrregularField(scenarios(12).name, targetArea_ha);

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

function field = makeSquareField(name, area_ha)
    side_m = sqrt(area_ha * 10000);
    field = makeRectangleField(name, 0, 0, side_m, side_m, 0);
end

function field = makeAspectRatioField(name, area_ha, aspectRatio)
    area_m2 = area_ha * 10000;
    width_m = sqrt(area_m2 / aspectRatio);
    length_m = aspectRatio * width_m;
    field = makeRectangleField(name, 0, 0, length_m, width_m, 0);
end

function field = makeTrapeziumField(name, targetArea_ha)
    h = 900;
    xy = [-650 -h/2; 650 -h/2; 330 h/2; -420 h/2];
    field = makePolygonField(name, xy, targetArea_ha);
end

function field = makeLShapeField(name, targetArea_ha)
    xy = [-620 -460; 620 -460; 620 -80; 190 -80; 190 460; -620 460];
    field = makePolygonField(name, xy, targetArea_ha);
end

function field = makeIrregularField(name, targetArea_ha)
    xy = [-650 -360; -300 -540; 360 -470; 670 -120; 500 350; 80 560; -520 320; -760 -40];
    field = makePolygonField(name, xy, targetArea_ha);
end

function field = makePolygonField(name, boundaryXY, targetArea_ha)
    targetArea_m2 = targetArea_ha * 10000;
    boundaryXY = boundaryXY - mean(boundaryXY, 1);
    currentArea_m2 = abs(polyarea(boundaryXY(:,1), boundaryXY(:,2)));
    assert(currentArea_m2 > 0, 'Polygon area must be positive.');
    scaleFactor = sqrt(targetArea_m2 / currentArea_m2);
    boundaryXY = boundaryXY * scaleFactor;

    field = makeRectangleField(name, NaN, NaN, NaN, NaN, NaN);
    field.type = 'polygon';
    field.boundaryXY = boundaryXY;
end

function fields = makeSplitSquareFields(totalArea_ha, nFields, prefix)
    perFieldArea_ha = totalArea_ha / nFields;
    side_m = sqrt(perFieldArea_ha * 10000);
    gap_m = 250;

    if nFields == 2
        gridCols = 2;
        gridRows = 1;
    elseif nFields == 4
        gridCols = 2;
        gridRows = 2;
    elseif nFields == 8
        gridCols = 4;
        gridRows = 2;
    else
        error('Unsupported split count %d. Use 2, 4, or 8.', nFields);
    end

    spacing_m = side_m + gap_m;
    fields = repmat(makeRectangleField('Split field', 0, 0, side_m, side_m, 0), nFields, 1);

    idx = 0;
    for iRow = 1:gridRows
        for iCol = 1:gridCols
            idx = idx + 1;
            centreEast_m = (iCol - (gridCols + 1)/2) * spacing_m;
            centreNorth_m = (iRow - (gridRows + 1)/2) * spacing_m;
            fields(idx) = makeRectangleField(sprintf('%s field %d', prefix, idx), ...
                centreEast_m, centreNorth_m, side_m, side_m, 0);
        end
    end
end

function plotScenarioLayouts(scenarios, cfg, outputDir)
    fig = figure('Name', 'Economic scenario field layouts', 'Color', 'w', 'Position', [80 80 1250 760]);
    nScenarios = numel(scenarios);
    nCols = min(4, ceil(sqrt(nScenarios)));
    nRows = ceil(nScenarios / nCols);
    t = tiledlayout(fig, nRows, nCols, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Field layouts used for per-mission economic comparison');

    for iScenario = 1:numel(scenarios)
        ax = nexttile(t);
        hold(ax, 'on')
        axis(ax, 'equal')
        grid(ax, 'on')

        fields = scenarios(iScenario).fields;
        for iField = 1:numel(fields)
            xy = rectangleBoundary(fields(iField));
            patch(ax, xy(:,1), xy(:,2), [0.75 0.88 0.73], ...
                'EdgeColor', [0.05 0.35 0.12], 'LineWidth', 1.4, 'FaceAlpha', 0.82);
        end

        gsHalf = cfg.groundStationSize / 2;
        gsX = cfg.groundStationEast_m + gsHalf * [-1 1 1 -1];
        gsY = cfg.groundStationNorth_m + gsHalf * [-1 -1 1 1];
        patch(ax, gsX, gsY, [0.15 0.25 0.42], 'EdgeColor', 'k', 'LineWidth', 1.0);

        title(ax, scenarios(iScenario).name, 'Interpreter', 'none');
        xlabel(ax, 'East [m]');
        ylabel(ax, 'North [m]');
        hold(ax, 'off')
    end

    exportgraphics(fig, fullfile(outputDir, 'field_layouts.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'field_layouts.fig'));
end

function plotProfitComparison(results, outputDir)
    labels = categorical(results.Scenario);
    labels = reordercats(labels, results.Scenario);

    fig = figure('Name', 'Economic scenario profit comparison', 'Color', 'w', 'Position', [90 90 1150 620]);
    t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Per-mission gross profit comparison');

    ax1 = nexttile(t);
    bar(ax1, labels, results.GrossProfit_GBP, 'FaceColor', [0.16 0.55 0.42]);
    yline(ax1, 0, 'k-', 'LineWidth', 1.0);
    grid(ax1, 'on');
    ylabel(ax1, 'Gross profit [GBP]');
    title(ax1, 'Total mission gross profit');
    xtickangle(ax1, 35);

    ax2 = nexttile(t);
    bar(ax2, labels, results.GrossProfitPerHa_GBP, 'FaceColor', [0.24 0.44 0.78]);
    yline(ax2, 0, 'k-', 'LineWidth', 1.0);
    grid(ax2, 'on');
    ylabel(ax2, 'Gross profit [GBP/ha]');
    title(ax2, 'Gross profit per hectare');
    xtickangle(ax2, 35);

    exportgraphics(fig, fullfile(outputDir, 'profit_comparison.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'profit_comparison.fig'));
end

function plotCostBreakdown(results, outputDir)
    labels = categorical(results.Scenario);
    labels = reordercats(labels, results.Scenario);

    costMatrix = [ ...
        results.TransportCost_GBP, ...
        results.LabourCost_GBP, ...
        results.BatteryChargingCost_GBP, ...
        results.MaintenanceCost_GBP];

    fig = figure('Name', 'Economic scenario cost breakdown', 'Color', 'w', 'Position', [100 100 1180 640]);
    t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Mission revenue and direct operating cost breakdown');

    ax1 = nexttile(t);
    bar(ax1, labels, [results.Revenue_GBP, -results.TotalCost_GBP, results.GrossProfit_GBP], 'grouped');
    yline(ax1, 0, 'k-', 'LineWidth', 1.0);
    grid(ax1, 'on');
    legend(ax1, {'Revenue', 'Total direct cost', 'Gross profit'}, 'Location', 'best');
    ylabel(ax1, 'GBP');
    title(ax1, 'Revenue, cost and gross profit');
    xtickangle(ax1, 35);

    ax2 = nexttile(t);
    bar(ax2, labels, costMatrix, 'stacked');
    grid(ax2, 'on');
    legend(ax2, {'Transport', 'Labour', 'Battery charge', 'Maintenance'}, 'Location', 'best');
    ylabel(ax2, 'Direct mission cost [GBP]');
    title(ax2, 'Direct cost components');
    xtickangle(ax2, 35);

    exportgraphics(fig, fullfile(outputDir, 'cost_breakdown.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'cost_breakdown.fig'));
end

function plotCapacityComparison(results, outputDir)
    labels = categorical(results.Scenario);
    labels = reordercats(labels, results.Scenario);

    fig = figure('Name', 'Economic scenario operational capacity comparison', 'Color', 'w', 'Position', [110 110 1180 640]);
    t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Operational capacity by scenario');

    ax1 = nexttile(t);
    bar(ax1, labels, results.MissionTime_hr, 'FaceColor', [0.36 0.55 0.75]);
    yline(ax1, 8, 'k--', 'LineWidth', 1.2, 'Label', '8 hr', 'LabelHorizontalAlignment', 'left');
    grid(ax1, 'on');
    ylabel(ax1, 'Mission time [hr]');
    title(ax1, 'Mission time');
    xtickangle(ax1, 35);

    ax2 = nexttile(t);
    bar(ax2, labels, results.OverallCapacity_ha_per_hr, 'FaceColor', [0.35 0.65 0.45]);
    grid(ax2, 'on');
    ylabel(ax2, 'Overall field capacity [ha/hr]');
    title(ax2, 'Area completed per hour');
    xtickangle(ax2, 35);

    exportgraphics(fig, fullfile(outputDir, 'capacity_comparison.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'capacity_comparison.fig'));
end

function plotBreakEvenComparison(results, cfg, outputDir)
    labels = categorical(results.Scenario);
    labels = reordercats(labels, results.Scenario);

    fig = figure('Name', 'Break-even spraying price comparison', 'Color', 'w', 'Position', [100 100 1120 600]);
    bar(labels, results.BreakEvenPricePerHa_GBP, 'FaceColor', [0.55 0.38 0.72]);
    hold on
    yline(cfg.revenuePerHa_GBP, 'k--', 'LineWidth', 1.4, ...
        'Label', sprintf('Model price %.2f GBP/ha', cfg.revenuePerHa_GBP), ...
        'LabelHorizontalAlignment', 'left');
    hold off
    grid on
    ylabel('Break-even spraying price [GBP/ha]');
    title('Minimum price per hectare required to cover direct mission costs');
    xtickangle(35);

    exportgraphics(fig, fullfile(outputDir, 'break_even_price.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'break_even_price.fig'));
end

function plotPriceSensitivity(results, outputDir)
    priceCases_GBP_per_ha = [8 10 12 14 16 18 20];
    profitMatrix = results.Area_ha * priceCases_GBP_per_ha - results.TotalCost_GBP;

    fig = figure('Name', 'Price sensitivity comparison', 'Color', 'w', 'Position', [100 100 1180 660]);
    plot(priceCases_GBP_per_ha, profitMatrix.', '-o', 'LineWidth', 1.4, 'MarkerSize', 4);
    yline(0, 'k-', 'LineWidth', 1.0);
    grid on
    xlabel('Spraying price [GBP/ha]');
    ylabel('Mission gross profit [GBP]');
    title('Sensitivity of mission gross profit to assumed spraying price');
    legend(results.Scenario, 'Location', 'eastoutside', 'Interpreter', 'none');

    exportgraphics(fig, fullfile(outputDir, 'price_sensitivity.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'price_sensitivity.fig'));

    sensitivityTable = array2table(profitMatrix, ...
        'VariableNames', matlab.lang.makeValidName(compose('Price_%g_GBP_per_ha', priceCases_GBP_per_ha)));
    sensitivityTable = addvars(sensitivityTable, results.Scenario, 'Before', 1, 'NewVariableNames', 'Scenario');
    writetable(sensitivityTable, fullfile(outputDir, 'price_sensitivity_results.csv'));
end

function plotTargetBenchmarkComparison(results, outputDir)
    labels = categorical(results.Scenario);
    labels = reordercats(labels, results.Scenario);

    fig = figure('Name', 'Benchmark comparison against 8-hour rectangle baseline', 'Color', 'w', 'Position', [100 100 1160 620]);
    t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Performance relative to the 8-hour rectangle baseline');

    ax1 = nexttile(t);
    bar(ax1, labels, results.GrossProfitChangeVsTarget_pct, 'FaceColor', [0.28 0.58 0.65]);
    yline(ax1, 0, 'k-', 'LineWidth', 1.0);
    grid(ax1, 'on');
    ylabel(ax1, 'Gross profit change [%]');
    title(ax1, 'Total gross profit vs target');
    xtickangle(ax1, 35);

    ax2 = nexttile(t);
    bar(ax2, labels, results.ProfitPerHaChangeVsTarget_pct, 'FaceColor', [0.71 0.48 0.28]);
    yline(ax2, 0, 'k-', 'LineWidth', 1.0);
    grid(ax2, 'on');
    ylabel(ax2, 'Profit per hectare change [%]');
    title(ax2, 'Profit per hectare vs target');
    xtickangle(ax2, 35);

    exportgraphics(fig, fullfile(outputDir, 'target_benchmark_comparison.png'), 'Resolution', 220);
    savefig(fig, fullfile(outputDir, 'target_benchmark_comparison.fig'));
end

function plotScatterPack(results, outputDir)
    fig = figure('Name', 'Economic scenario scatter pack', 'Color', 'w', ...
        'Position', [70 70 1500 980]);
    t = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Economic scatter summary by scenario');

    pointColors = buildScenarioColors(results);
    pointNumbers = (1:height(results)).';

    ax1 = nexttile(t);
    drawNumberedScatter(ax1, results.Area_ha, results.GrossProfitPerHa_GBP, ...
        pointColors, pointNumbers);
    xlabel(ax1, 'Sprayed area [ha]');
    ylabel(ax1, 'Gross profit [GBP/ha]');
    title(ax1, 'Scale effect');
    yline(ax1, 0, 'k-', 'LineWidth', 1.0);

    ax2 = nexttile(t);
    drawNumberedScatter(ax2, results.MissionTime_hr, results.BreakEvenPricePerHa_GBP, ...
        pointColors, pointNumbers);
    xlabel(ax2, 'Mission time [hr]');
    ylabel(ax2, 'Break-even price [GBP/ha]');
    title(ax2, 'Time and minimum viable price');
    xline(ax2, 8, 'k--', 'LineWidth', 1.0, 'Label', '8 hr', ...
        'LabelHorizontalAlignment', 'left');

    ax3 = nexttile(t);
    drawNumberedScatter(ax3, results.OverallCapacity_ha_per_hr, results.GrossProfitPerHa_GBP, ...
        pointColors, pointNumbers);
    xlabel(ax3, 'Overall field capacity [ha/hr]');
    ylabel(ax3, 'Gross profit [GBP/ha]');
    title(ax3, 'Field capacity and profit');
    yline(ax3, 0, 'k-', 'LineWidth', 1.0);

    ax4 = nexttile(t);
    axis(ax4, 'off');
    keyLines = buildScenarioKeyLines(results);
    text(ax4, 0.02, 0.98, keyLines, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
        'FontName', 'Consolas', 'FontSize', 10, 'Interpreter', 'none');
    title(ax4, 'Scenario key');

    scatterPath = fullfile(outputDir, 'scatter_summary_pack.png');
    exportgraphics(fig, scatterPath, 'Resolution', 240);
    savefig(fig, fullfile(outputDir, 'scatter_summary_pack.fig'));

    reportPackDir = fullfile(outputDir, 'ReportFigurePack');
    if ~exist(reportPackDir, 'dir')
        mkdir(reportPackDir);
    end
    exportgraphics(fig, fullfile(reportPackDir, 'figure_3_scatter_summary_pack.png'), 'Resolution', 240);
    savefig(fig, fullfile(reportPackDir, 'figure_3_scatter_summary_pack.fig'));
end

function drawNumberedScatter(ax, x, y, colors, pointNumbers)
    hold(ax, 'on');
    for i = 1:numel(pointNumbers)
        scatter(ax, x(i), y(i), 185, colors(i,:), 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
        text(ax, x(i), y(i), sprintf('%d', pointNumbers(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontWeight', 'bold', 'FontSize', 8, 'Color', 'w');
    end
    hold(ax, 'off');
    grid(ax, 'on');
    box(ax, 'on');
end

function colors = buildScenarioColors(results)
    colors = repmat([0.30 0.45 0.80], height(results), 1);
    for i = 1:height(results)
        scenarioName = results.Scenario{i};
        if contains(scenarioName, 'rectangle')
            colors(i,:) = [0.15 0.58 0.38];
        elseif contains(scenarioName, 'trapezium') || contains(scenarioName, 'L-shape') || contains(scenarioName, 'irregular')
            colors(i,:) = [0.78 0.40 0.18];
        elseif contains(scenarioName, '120 ha square') || contains(scenarioName, '80 ha square')
            colors(i,:) = [0.50 0.38 0.75];
        end
    end
end

function keyLines = buildScenarioKeyLines(results)
    keyLines = cell(height(results) + 4, 1);
    keyLines{1} = 'No.  Scenario                         Area    Time    Profit/ha   Break-even';
    keyLines{2} = '---  -------------------------------  ------  ------  ----------  ----------';
    for i = 1:height(results)
        keyLines{i+2} = sprintf('%2d.  %-31s  %5.1f   %5.2f   %8.2f    %8.2f', ...
            i, truncateScenarioName(results.Scenario{i}, 31), results.Area_ha(i), ...
            results.MissionTime_hr(i), results.GrossProfitPerHa_GBP(i), ...
            results.BreakEvenPricePerHa_GBP(i));
    end
    keyLines{end-1} = '';
    keyLines{end} = 'Colours: blue/small squares, purple/large squares, green/rectangles, orange/non-rectangles.';
end

function out = truncateScenarioName(in, maxLen)
    out = in;
    if strlength(out) > maxLen
        out = char(extractBefore(string(out), maxLen - 2));
        out = [out '...'];
    end
end

function xy = rectangleBoundary(field)
    if isfield(field, 'type') && strcmpi(field.type, 'polygon')
        xy = field.boundaryXY;
        return
    end

    L = field.length_m;
    W = field.width_m;
    localXY = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2];
    psi = deg2rad(field.rotation_deg);
    R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    xy = localXY * R.' + [field.centreEast_m, field.centreNorth_m];
end

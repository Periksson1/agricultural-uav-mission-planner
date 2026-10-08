function createResultsDashboard(missionCfg, planningData, simulationData)
    resultsFig = figure('Color', 'w');
    t = tiledlayout(resultsFig, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    sgtitle(resultsFig, 'Mission Results Dashboard');

    avgDivisor = max(simulationData.nSorties, 1);
    averageSortieTime_s = simulationData.totalTime / avgDivisor;

    axR1 = nexttile(t, 1);
    plotTextPanel(axR1, 'Key mission metrics', {
        sprintf('Original fields drawn: %d', planningData.originalFieldCount)
        sprintf('Combined spray blocks: %d', planningData.combinedFieldCount)
        sprintf('Fields in plan: %d', planningData.fieldCount)
        sprintf('Visit order: %s', mat2str(planningData.fieldVisitOrder))
        sprintf('Total field area: %.3f ha', planningData.totalFieldArea_ha)
        sprintf('Planned spray area: %.3f ha', planningData.plannedSprayArea_ha)
        sprintf('Area sprayed: %.3f ha', simulationData.sprayedArea_ha(end))
        sprintf('Unsprayed area: %.3f ha', simulationData.areaNotSprayed_ha)
        sprintf('Total lanes: %d', planningData.totalLaneCount)
        sprintf('Lane changes: %d', planningData.laneTurnCount)
        sprintf('Turns per hectare: %.3f', simulationData.turnsPerHa)
        sprintf('Sorties: %d', simulationData.nSorties)
        sprintf('Average sortie time: %s', formatDurationHM(averageSortieTime_s))
        sprintf('Returns to ground station: %d', simulationData.returnCount)
        sprintf('Mission time: %s', formatDurationHM(simulationData.totalTime))
        sprintf('Total distance: %.1f m', simulationData.totalDistance)
        sprintf('Spray rate: %.3f ha/hr', simulationData.sprayRate_ha_per_hr)
        sprintf('Overall field capacity: %.3f ha/hr', simulationData.overallFieldCapacity_ha_per_hr)
        });

    axR2 = nexttile(t, 2);
    plotPieBreakdown(axR2, ...
        [simulationData.sprayTime, simulationData.laneTurnTime, simulationData.transferTimeTotal, simulationData.vtolTimeTotal, simulationData.turnaroundTimeTotal], ...
        {'Spraying', 'Inter-pass', 'Transfer', 'VTOL', 'Turnaround'}, ...
        'Detailed time usage');

    axR3 = nexttile(t, 3);
    plotPieBreakdown(axR3, ...
        [simulationData.sprayTime, simulationData.laneTurnTime, simulationData.transferTimeTotal, simulationData.vtolTimeTotal, simulationData.turnaroundTimeTotal] ./ avgDivisor, ...
        {'Spraying', 'Inter-pass', 'Transfer', 'VTOL', 'Turnaround'}, ...
        'Average time usage per sortie');

    axR4 = nexttile(t, 4);
    plotTextPanel(axR4, 'Battery / payload / operations', {
        sprintf('Payload used: %.2f L', simulationData.totalPayloadUsed_L)
        sprintf('Payload remaining: %.2f L', simulationData.payloadRemaining_L(end))
        sprintf('Final battery: %.1f %%', simulationData.finalBatteryPct)
        sprintf('Final mass: %.2f kg', simulationData.finalWeightKg)
        sprintf('Mission energy demand: %.3f kWh', simulationData.missionEnergyDemand_kWh)
        sprintf('Energy per hectare: %.3f kWh/ha', simulationData.energyPerHa_kWh)
        sprintf('Average battery at RTB trigger: %.1f %%', simulationData.avgBatteryPctAtRTBTrigger)
        sprintf('Minimum battery at RTB trigger: %.1f %%', simulationData.minBatteryPctAtRTBTrigger)
        sprintf('Average battery at GS arrival: %.1f %%', simulationData.avgBatteryPctAtGSArrival)
        sprintf('Average sortie time: %s', formatDurationHM(averageSortieTime_s))
        sprintf('Payload-triggered RTBs: %d', simulationData.payloadTriggerCount)
        sprintf('Battery-triggered RTBs: %d', simulationData.batteryTriggerCount)
        sprintf('Completed sortie count: %d', simulationData.completeSortieCount)
        sprintf('Labour billed hours: %.1f hr', simulationData.missionLabourBillableHours)
        });

    axR5 = nexttile(t, 5);
    plotPiePercentBreakdown(axR5, ...
        [simulationData.sprayEnergy_Wh, simulationData.laneTurnEnergy_Wh, simulationData.transferEnergy_Wh, simulationData.vtolEnergy_Wh], ...
        {'Spraying', 'Inter-pass', 'Transfer', 'VTOL'}, ...
        'Battery energy usage by phase');

    axR6 = nexttile(t, 6);
    economicOrder = {'Revenue', 'Transport', 'Labour', 'Battery charge', 'Maintenance', 'Total costs', 'Net profit'};
    economicLabels = categorical(economicOrder, economicOrder, 'Ordinal', true);
    economicValues = [simulationData.totalRevenue_GBP, missionCfg.transportCost_GBP, simulationData.labourCost_GBP, simulationData.batteryChargingCost_GBP, simulationData.maintenanceCost_GBP, simulationData.totalCost_GBP, simulationData.netProfit_GBP];
    hEconomicBar = bar(axR6, economicLabels, economicValues, 'FaceColor', 'flat');
    barColors = zeros(numel(economicValues), 3);
    barColors(1,:) = [0.10 0.70 0.10];
    barColors(2:6,:) = repmat([0.85 0.15 0.15], 5, 1);
    barColors(7,:) = ternary(simulationData.netProfit_GBP >= 0, [0.10 0.70 0.10], [0.85 0.15 0.15]);
    hEconomicBar.CData = barColors;
    grid(axR6, 'on')
    yline(axR6, 0, 'k-', 'LineWidth', 1.0)
    ylabel(axR6, 'GBP')
    title(axR6, 'Revenue and costs')

    axR7 = nexttile(t, 7);
    if isempty(simulationData.payloadAtRTBTrigger_L)
        axis(axR7, 'off')
        text(axR7, 0.5, 0.5, 'No RTB events', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12);
    else
        payloadAtRTBDisplay_L = simulationData.payloadAtRTBTrigger_L(:);
        payloadAtRTBDisplay_L(payloadAtRTBDisplay_L < 0.1) = 0;
        eIdx = (1:numel(simulationData.payloadAtRTBTrigger_L))';
        triggerColors = buildRTBTriggerColors(simulationData.rtbTriggerTypes);
        hPayload = bar(axR7, eIdx, payloadAtRTBDisplay_L, 0.62, 'FaceColor', 'flat', ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        hPayload.CData = triggerColors;
        grid(axR7, 'on')
        xlabel(axR7, 'RTB event number')
        ylabel(axR7, 'Payload remaining [L]')
        ylim(axR7, [0 max(1.0, 1.05 * max(payloadAtRTBDisplay_L))])
        set(axR7, 'XTick', eIdx, 'XTickLabel', buildNumericEventLabels(numel(eIdx)))
        title(axR7, 'Payload at RTB trigger')
    end

    axR8 = nexttile(t, 8);
    if isempty(simulationData.batteryPctAtRTBTrigger)
        axis(axR8, 'off')
        text(axR8, 0.5, 0.5, 'No RTB events', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12);
    else
        eIdx = (1:numel(simulationData.batteryPctAtRTBTrigger))';
        triggerColors = buildRTBTriggerColors(simulationData.rtbTriggerTypes);
        hold(axR8, 'on')
        hBars = bar(axR8, eIdx, simulationData.batteryPctAtRTBTrigger(:), 0.62, 'FaceColor', 'flat', ...
            'EdgeColor', 'k', 'LineWidth', 1.0);
        hBars.CData = triggerColors;
        hGSArrival = plot(axR8, eIdx, simulationData.batteryPctAtGSArrival(:), 'ks-', 'LineWidth', 1.5, ...
            'MarkerFaceColor', 'w', 'DisplayName', 'GS arrival');
        hBatteryTriggerLegend = patch(axR8, NaN, NaN, [0.20 0.45 0.85], ...
            'EdgeColor', 'k', 'DisplayName', 'Battery trigger');
        hPayloadTriggerLegend = patch(axR8, NaN, NaN, [0.85 0.20 0.70], ...
            'EdgeColor', 'k', 'DisplayName', 'Payload trigger');
        hold(axR8, 'off')
        grid(axR8, 'on')
        xlabel(axR8, 'RTB event number')
        ylabel(axR8, 'Battery [%]')
        ylim(axR8, [0 100])
        set(axR8, 'XTick', eIdx, 'XTickLabel', buildNumericEventLabels(numel(eIdx)))
        legend(axR8, [hBatteryTriggerLegend, hPayloadTriggerLegend, hGSArrival], 'Location', 'best')
        title(axR8, 'Battery at RTB trigger vs GS arrival')
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function plotTextPanel(ax, titleStr, linesCell)
    cla(ax);
    axis(ax, 'off');
    box(ax, 'on');
    text(ax, 0.02, 0.98, sprintf('%s\n\n%s', titleStr, strjoin(linesCell, newline)), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
        'Interpreter', 'none', 'FontSize', 10.2, 'FontName', 'Consolas');
end

function plotPieBreakdown(ax, values, labels, titleStr)
    cla(ax)
    keep = values > 0;
    values = values(keep);
    labels = labels(keep);
    if isempty(values)
        axis(ax, 'off')
        text(ax, 0.5, 0.5, 'No data available', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12);
        title(ax, titleStr)
        return
    end
    pct = 100 * values / sum(values);
    pieLabels = cell(size(labels));
    for i = 1:numel(labels)
        pieLabels{i} = sprintf('%s\n%s\n%.1f%%', labels{i}, formatDurationCompact(values(i)), pct(i));
    end
    pie(ax, values, pieLabels);
    title(ax, titleStr)
end

function plotPiePercentBreakdown(ax, values, labels, titleStr)
    cla(ax)
    keep = values > 0;
    values = values(keep);
    labels = labels(keep);
    if isempty(values)
        axis(ax, 'off')
        text(ax, 0.5, 0.5, 'No data available', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12);
        title(ax, titleStr)
        return
    end
    pct = 100 * values / sum(values);
    pieLabels = cell(size(labels));
    for i = 1:numel(labels)
        pieLabels{i} = sprintf('%s\n%.1f%%', labels{i}, pct(i));
    end
    pie(ax, values, pieLabels);
    title(ax, titleStr)
end

function out = formatDurationHM(seconds)
    seconds = max(seconds, 0);
    h = floor(seconds / 3600);
    m = round((seconds - 3600*h) / 60);
    if m == 60
        h = h + 1;
        m = 0;
    end
    if h > 0
        out = sprintf('%d h %d min', h, m);
    else
        out = sprintf('%d min', m);
    end
end

function out = formatDurationCompact(seconds)
    seconds = max(seconds, 0);
    if seconds >= 3600
        out = formatDurationHM(seconds);
    elseif seconds >= 60
        out = sprintf('%.1f min', seconds/60);
    else
        out = sprintf('%.0f s', seconds);
    end
end

function labels = buildNumericEventLabels(n)
    labels = arrayfun(@num2str, (1:n).', 'UniformOutput', false);
end

function colorMat = buildRTBTriggerColors(triggerTypes)
    n = numel(triggerTypes);
    colorMat = zeros(n,3);
    for i = 1:n
        if strcmpi(triggerTypes{i}, 'battery')
            colorMat(i,:) = [0.20 0.45 0.85];
        elseif strcmpi(triggerTypes{i}, 'payload')
            colorMat(i,:) = [0.85 0.20 0.70];
        else
            colorMat(i,:) = [0.55 0.55 0.55];
        end
    end
end

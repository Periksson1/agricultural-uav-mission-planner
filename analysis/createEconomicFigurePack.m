%% Package economic analysis figures into report-ready multi-panel layouts
% Creates two 4-panel PNGs from the existing economic scenario outputs so the
% report can refer to Figure 1a-d and Figure 2a-d instead of many separate plots.

clc
close all

rootDir = fullfile(projectRoot(), 'outputs', 'economics');
packDir = fullfile(rootDir, 'ReportFigurePack');
if ~exist(packDir, 'dir')
    mkdir(packDir);
end

figure1Panels = { ...
    fullfile(rootDir, 'field_layouts.png'), ...
    'a) Scenario layouts'; ...
    fullfile(rootDir, 'profit_comparison.png'), ...
    'b) Mission gross profit'; ...
    fullfile(rootDir, 'break_even_price.png'), ...
    'c) Break-even price'; ...
    fullfile(rootDir, 'capacity_comparison.png'), ...
    'd) Mission time and capacity'};

figure2Panels = { ...
    fullfile(rootDir, 'cost_breakdown.png'), ...
    'a) Direct cost breakdown'; ...
    fullfile(rootDir, 'target_benchmark_comparison.png'), ...
    'b) Change versus 120 ha target'; ...
    fullfile(rootDir, 'Sensitivity120haRectangle', 'sensitivity_gross_profit.png'), ...
    'c) Sensitivity: gross profit'; ...
    fullfile(rootDir, 'Sensitivity120haRectangle', 'sensitivity_break_even_price.png'), ...
    'd) Sensitivity: break-even price'};

makePanelFigure(figure1Panels, ...
    'Figure 1. Field-size and shape effects on mission profitability', ...
    fullfile(packDir, 'figure_1_scenario_profitability_pack.png'));

makePanelFigure(figure2Panels, ...
    'Figure 2. Cost structure and 120 ha rectangle sensitivity analysis', ...
    fullfile(packDir, 'figure_2_cost_sensitivity_pack.png'));

captionPath = fullfile(packDir, 'suggested_captions.txt');
fid = fopen(captionPath, 'w');
assert(fid > 0, 'Could not open caption output file.');
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, 'Suggested figure use for 400-word economics section\n');
fprintf(fid, '====================================================\n\n');
fprintf(fid, 'Figure 1: Use this as the main scenario-comparison figure.\n');
fprintf(fid, 'Caption: Per-mission economic comparison for square fields from 2-120 ha, followed by 120 ha rectangular and non-rectangular boundaries. Panels show the scenario layouts, gross profit, break-even price, mission time and field capacity.\n\n');
fprintf(fid, 'Figure 2: Use this as the supporting cost/sensitivity figure.\n');
fprintf(fid, 'Caption: Direct mission cost structure and sensitivity analysis for the 120 ha rectangular baseline. Panels show the operating cost components, percentage change relative to the 120 ha rectangular baseline, and the sensitivity of gross profit and break-even price to spraying price and worker hourly rate.\n\n');
fprintf(fid, 'Recommended wording: Refer to Figure 1 first for the main finding, then use Figure 2 to justify which assumptions drive the result.\n');

fprintf('Created report figure pack:\n');
fprintf('  %s\n', fullfile(packDir, 'figure_1_scenario_profitability_pack.png'));
fprintf('  %s\n', fullfile(packDir, 'figure_2_cost_sensitivity_pack.png'));
fprintf('  %s\n', captionPath);

function makePanelFigure(panelList, figureTitle, outputPath)
    fig = figure('Name', figureTitle, 'Color', 'w', 'Position', [60 60 1600 1050]);
    t = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, figureTitle, 'FontWeight', 'bold');

    for iPanel = 1:size(panelList, 1)
        imagePath = panelList{iPanel, 1};
        panelTitle = panelList{iPanel, 2};
        assert(isfile(imagePath), 'Missing source figure: %s', imagePath);

        ax = nexttile(t);
        img = imread(imagePath);
        image(ax, img);
        axis(ax, 'image');
        axis(ax, 'off');
        title(ax, panelTitle, 'Interpreter', 'none', 'FontWeight', 'bold');
    end

    exportgraphics(fig, outputPath, 'Resolution', 240);
    savefig(fig, replace(outputPath, '.png', '.fig'));
end

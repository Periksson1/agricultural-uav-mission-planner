%EXPORTREADMEFIGURES Create the README figures from the current mission.
% First run runMission; run runFlightSimulation for an optional tracking figure.
setupProject;
assert(exist('simulationData','var') == 1, 'Run runMission first.');
imageFolder = fullfile(projectRoot(),'docs','images');
if ~isfolder(imageFolder), mkdir(imageFolder); end
plotColors = [0.10 0.38 0.62; 0.88 0.36 0.12; 0.13 0.57 0.40; 0.58 0.32 0.67; 0.73 0.55 0.12];
codesForExport = missionCodes();

fOverview = figure('Color','w','Position',[80 80 1280 680], 'Name','SA farm mission overview');
tiledlayout(fOverview,1,2,'TileSpacing','loose','Padding','loose');
axMap = nexttile; hold(axMap,'on');
for kField = 1:numel(planningData.fieldInfo)
    boundary = planningData.fieldInfo(kField).boundaryXYZ;
    fill(axMap,boundary(:,1),boundary(:,2),[0.85 0.93 0.83], ...
        'FaceAlpha',0.7,'EdgeColor',[0.3 0.5 0.3],'HandleVisibility','off');
end
route = simulationData.pathXYZ;
overviewIdx = unique([1:20:size(route,1), size(route,1)]);
plot(axMap,route(overviewIdx,1),route(overviewIdx,2),'Color',[0.70 0.77 0.83],'LineWidth',0.5, 'DisplayName','Flight path');
spraySegments = find(simulationData.segmentStages == codesForExport.stage.sprayLeg);
sprayStart = spraySegments([true; diff(spraySegments)>1]);
sprayEnd = spraySegments([diff(spraySegments)>1; true])+1;
sprayX = [route(sprayStart,1),route(sprayEnd,1),nan(numel(sprayStart),1)].';
sprayY = [route(sprayStart,2),route(sprayEnd,2),nan(numel(sprayStart),1)].';
plot(axMap,sprayX(:),sprayY(:),'Color',plotColors(3,:),'LineWidth',0.75,'DisplayName','Spray passes');
plot(axMap,missionCfg.groundStationEast_m,missionCfg.groundStationNorth_m,'p', ...
    'MarkerSize',13,'MarkerFaceColor',plotColors(2,:),'Color','k','DisplayName','Ground station');
axis(axMap,'equal'); axis(axMap,'tight'); grid(axMap,'on');
xlabel(axMap,'East [m]'); ylabel(axMap,'North [m]');
title(axMap,'Five-field SA farm mission');
legend(axMap,'Location','southoutside','Orientation','horizontal');
axSortie = nexttile; hold(axSortie,'on');
idxExport = find(simulationData.segmentSortie == selectedSortie);
sortieXYZ = route(idxExport(1):idxExport(end)+1,:);
plot3(axSortie,sortieXYZ(:,1),sortieXYZ(:,2),sortieXYZ(:,3), ...
    'Color',plotColors(1,:),'LineWidth',1.3);
grid(axSortie,'on'); view(axSortie,35,25);
xlabel(axSortie,'East [m]'); ylabel(axSortie,'North [m]'); zlabel(axSortie,'Up [m]');
title(axSortie,sprintf('Sortie %d: climb, spray and return',selectedSortie));
subtitle(axSortie,'Altitude axis expanded for visibility');
exportgraphics(fOverview,fullfile(imageFolder,'mission-overview.png'),'Resolution',160);

fSummary = figure('Color','w','Position',[90 90 1240 650],'Name','Mission resource use');
tiledlayout(fSummary,2,2,'TileSpacing','compact','Padding','loose');
ax = nexttile; plot(ax,simulationData.pathTime/3600,simulationData.batteryPct,'Color',plotColors(1,:));
ylabel(ax,'Battery [%]'); xlabel(ax,'Mission time [h]'); grid(ax,'on'); title(ax,'Battery use across sorties');
ax = nexttile; plot(ax,simulationData.pathTime/3600,simulationData.payloadRemaining_L,'Color',plotColors(3,:));
ylabel(ax,'Payload [L]'); xlabel(ax,'Mission time [h]'); grid(ax,'on'); title(ax,'Spray payload and refills');
ax = nexttile; plot(ax,simulationData.pathTime/3600,simulationData.sprayedArea_ha,'Color',plotColors(2,:),'LineWidth',1.5);
ylabel(ax,'Sprayed area [ha]'); xlabel(ax,'Mission time [h]'); grid(ax,'on'); title(ax,'Cumulative coverage');
ax = nexttile;
costs = [simulationData.totalRevenue_GBP simulationData.totalCost_GBP simulationData.netProfit_GBP];
bar(ax,categorical({'Revenue','Modelled costs','Modelled profit'},{'Revenue','Modelled costs','Modelled profit'}),costs,'FaceColor',plotColors(1,:));
ylabel(ax,'GBP per mission'); grid(ax,'on'); title(ax,'Baseline economics');
exportgraphics(fSummary,fullfile(imageFolder,'mission-summary.png'),'Resolution',160);

if exist('out','var') && exist('simulatedSortie','var')
    assert(selectedSortie == simulatedSortie, 'Select the simulated sortie before exporting tracking results.');
    actualNED = squeeze(out.Position.Data);
    if size(actualNED,2) ~= 3, actualNED = actualNED.'; end
    flightExport = simulationData.sortieCommandExports(selectedSortie);
    flightHorizon_s = flightExport.endMissionTime - flightExport.startMissionTime;
    flightTime = out.Position.Time;
    flightMask = flightTime <= flightHorizon_s;
    flightTime = flightTime(flightMask);
    actualNED = actualNED(flightMask,:);
    actualENU = [actualNED(:,2),actualNED(:,1),-actualNED(:,3)];
    fTracking = figure('Color','w','Position',[100 100 1240 560],'Name','Flight tracking comparison');
    tiledlayout(fTracking,1,2,'TileSpacing','compact','Padding','loose');
    ax = nexttile; hold(ax,'on');
    plot(ax,sortieXYZ(:,1),sortieXYZ(:,2),'--','Color',plotColors(1,:),'LineWidth',1.4,'DisplayName','Planned');
    plot(ax,actualENU(:,1),actualENU(:,2),'Color',plotColors(2,:),'LineWidth',1.1,'DisplayName','Simulink');
    axis(ax,'equal'); axis(ax,'tight'); grid(ax,'on'); xlabel(ax,'East [m]'); ylabel(ax,'North [m]');
    title(ax,sprintf('Sortie %d: planned and simulated route',selectedSortie)); legend(ax,'Location','best');
    ax = nexttile; plot(ax,flightTime,actualENU(:,3),'Color',plotColors(2,:));
    yline(ax,missionCfg.flightHeight,'--','Spray height','Color',plotColors(3,:));
    grid(ax,'on'); xlabel(ax,'Flight time [s]'); ylabel(ax,'Altitude [m]'); title(ax,'Simulated altitude: cruise interval');
    exportgraphics(fTracking,fullfile(imageFolder,'flight-tracking.png'),'Resolution',160);
end
economicFigure = fullfile(projectRoot(),'outputs','economics','profit_comparison.png');
if isfile(economicFigure)
    copyfile(economicFigure,fullfile(imageFolder,'economic-scenarios.png'));
end
fprintf('README figures saved to %s\n',imageFolder);

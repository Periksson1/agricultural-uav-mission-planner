%% plotElevationProfile
% Plot terrain elevation and planned flight altitude against distance.
%
% Expected workflow:
% 1. Run runMission.m so missionCfg and
%    simulationData exist in the workspace.
% 2. Run this script.

assert(exist('missionCfg', 'var') == 1, ...
    'missionCfg was not found. Run runMission.m first.');
assert(exist('simulationData', 'var') == 1, ...
    'simulationData was not found. Run runMission.m first.');

validateSimulationData(simulationData);

pathXYZForProfile = simulationData.pathXYZ;
distanceAlongPath_m = [0; cumsum(vecnorm(diff(pathXYZForProfile, 1, 1), 2, 2))];
terrainElevation_m = zeros(size(distanceAlongPath_m));

if isfield(missionCfg, 'geodata') && isfield(missionCfg.geodata, 'terrainModel') && ...
        isfield(missionCfg.geodata.terrainModel, 'pointsXY') && ~isempty(missionCfg.geodata.terrainModel.pointsXY)
    terrainElevation_m = sampleTerrainProfile(missionCfg.geodata.terrainModel, pathXYZForProfile(:,1:2));
else
    warning('No elevation samples are loaded. Plotting against flat ground at z = 0 m.');
end

clearance_m = pathXYZForProfile(:,3) - terrainElevation_m;

fig = figure('Color', 'w', 'Name', 'Elevation Profile', 'NumberTitle', 'off');
ax = axes('Parent', fig);
hold(ax, 'on')
grid(ax, 'on')

plot(ax, distanceAlongPath_m, terrainElevation_m, '-', ...
    'Color', [0.40 0.28 0.16], 'LineWidth', 1.6, 'DisplayName', 'Terrain elevation');
plot(ax, distanceAlongPath_m, pathXYZForProfile(:,3), '-', ...
    'Color', [0.18 0.43 0.82], 'LineWidth', 1.8, 'DisplayName', 'Planned altitude');
plot(ax, distanceAlongPath_m, clearance_m, '--', ...
    'Color', [0.10 0.60 0.25], 'LineWidth', 1.3, 'DisplayName', 'Ground clearance');

xlabel(ax, 'Distance along mission path [m]')
ylabel(ax, 'Height [m]')
title(ax, 'Mission elevation profile')
legend(ax, 'Location', 'best')

function terrainZ = sampleTerrainProfile(terrainModel, xy)
    interpolant = scatteredInterpolant(terrainModel.pointsXY(:,1), terrainModel.pointsXY(:,2), ...
        terrainModel.elevationM(:), 'natural', 'nearest');
    terrainZ = interpolant(xy(:,1), xy(:,2));
end

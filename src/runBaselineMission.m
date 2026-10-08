function [cfg, planningData, simulationData, plotData] = runBaselineMission(overrides, layoutOptions)
%RUNBASELINEMISSION Run the common baseline without figures or workspace resets.
if nargin < 1, overrides = struct(); end
if nargin < 2, layoutOptions = struct(); end
cfg = defaultMissionConfig(layoutOptions);
cfg = mergeOptions(cfg, overrides);
cfg.fieldCount = numel(cfg.fields);
cfg.planningOptions.shortLaneAbsoluteThreshold_m = max(2*cfg.sprayWidth, 12);
cfg.planningOptions.shortLanePenalty_m = 4*cfg.sprayWidth;
cfg.planningOptions = sanitizePlanningOptions(cfg.planningOptions);
validateMissionCfg(cfg);
planningData = runMissionPlanning(cfg);
simulationData = runMissionSimulation(cfg, planningData);
if nargout >= 4
    plotData = buildMissionPlotData(cfg, planningData, simulationData);
end
end

function config = mergeOptions(config, overrides)
names = fieldnames(overrides);
for k = 1:numel(names)
    name = names{k};
    assert(isfield(config, name), 'Unknown mission setting: %s', name);
    if isstruct(config.(name)) && isscalar(config.(name)) && ...
            isstruct(overrides.(name)) && isscalar(overrides.(name))
        config.(name) = mergeOptions(config.(name), overrides.(name));
    else
        config.(name) = overrides.(name);
    end
end
end

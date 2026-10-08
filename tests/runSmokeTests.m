function runSmokeTests()
%RUNSMOKETESTS Check bundled layouts, overrides and a complete small mission.
setupProject;
cfg = defaultMissionConfig();
assert(cfg.fieldCount == 5, 'The default example should contain five SA farm fields.');
assert(isfile(cfg.geodata.sourceFieldPath), 'Default field data is missing.');
for k = 1:numel(cfg.fields)
    xy = cfg.fields(k).boundaryXY;
    assert(polyarea(xy(:,1),xy(:,2)) > 0, 'Field areas must be positive.');
end

[smallCfg, planning, simulation] = runBaselineMission( ...
    struct('planningOptions',struct('optimizeFieldVisitOrder',false)), struct('mode','saved'));
assert(~smallCfg.planningOptions.optimizeFieldVisitOrder);
assert(isfield(smallCfg.planningOptions,'candidateLaneAngleStep_deg'));
assert(all(isfinite(simulation.pathXYZ(:))));
assert(all(diff(simulation.pathTime) >= 0));
assert(simulation.totalTime > 0 && simulation.sprayedArea_ha(end) > 0);
assert(size(simulation.pathXYZ,1) == numel(simulation.pathTime));
assert(numel(simulation.segmentSortie) == size(simulation.pathXYZ,1)-1);
validatePlanningData(planning);
validateSimulationData(simulation);

failedAsExpected = false;
try
    runBaselineMission(struct('misspelledSetting',1));
catch exception
    failedAsExpected = contains(exception.message,'Unknown mission setting');
end
assert(failedAsExpected, 'Unknown settings must fail rather than silently being ignored.');
fprintf('PASS: bundled layouts, nested overrides, small mission and validation.\n');
end

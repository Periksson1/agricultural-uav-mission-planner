function modelName = configureFlightModel(sortieNumber, simulationData)
%CONFIGUREFLIGHTMODEL Keep path, spray gates, initial state and duration aligned.
validateattributes(sortieNumber, {'numeric'}, {'scalar','integer','positive'});
assert(sortieNumber <= size(simulationData.InitialPosBySortie,1), ...
    'Sortie %d does not exist. Run runMission first.', sortieNumber);
assert(evalin('base', 'exist(''waypoints'',''var'') && exist(''spraywaypoints'',''var'')'), ...
    'Run plotSortieWaypoints before preparing the flight model.');
modelName = 'SIXDOF28';
modelPath = fullfile(projectRoot(), 'models', [modelName '.slx']);
if bdIsLoaded(modelName)
    assert(strcmpi(get_param(modelName,'FileName'), modelPath), ...
        'A different SIXDOF28 is already open. Close it before using this repository.');
else
    load_system(modelPath);
end
assignin('base', 'selectedSortie', sortieNumber);
idx = find(simulationData.segmentSortie == sortieNumber);
assert(~isempty(idx), 'No path segments found for sortie %d.', sortieNumber);
flightExport = simulationData.sortieCommandExports(sortieNumber);
stopTime = flightExport.endMissionTime - flightExport.startMissionTime;
assert(isfinite(stopTime) && stopTime > 0, 'Invalid sortie duration.');
assignin('base', 'sortieStopTime_s', stopTime);
blocks = find_system(modelName, 'LookUnderMasks','all', 'FollowLinks','on', ...
    'MatchFilter',@Simulink.match.allVariants, 'Type','Block');
updated = 0;
for k = 1:numel(blocks)
    params = get_param(blocks{k}, 'DialogParameters');
    if isempty(params), continue; end
    names = fieldnames(params);
    for j = 1:numel(names)
        expression = get_param(blocks{k}, names{j});
        if ~ischar(expression), continue; end
        replacement = regexprep(expression, '(spraywaypoints|waypoints)\{[^}]+\}', '$1{selectedSortie}');
        replacement = regexprep(replacement, 'InitialPosBySortie\([^)]*\)', 'InitialPosBySortie(selectedSortie,:)');
        replacement = regexprep(replacement, 'InitialHeadingRadBySortie\([^)]*\)', 'InitialHeadingRadBySortie(selectedSortie)');
        if ~strcmp(replacement, expression)
            set_param(blocks{k}, names{j}, replacement);
            updated = updated+1;
        end
    end
end
set_param(modelName, 'StopTime', 'sortieStopTime_s', 'ReturnWorkspaceOutputs','on');
fprintf('Prepared %s, sortie %d, duration %g s (%d expressions updated).\n', ...
    modelName, sortieNumber, stopTime, updated);
end

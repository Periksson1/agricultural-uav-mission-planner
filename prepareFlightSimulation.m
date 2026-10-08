%PREPAREFLIGHTSIMULATION Initialise aircraft parameters and the selected sortie.
% Run runMission and plotSortieWaypoints first. Set selectedSortie to choose a flight.
setupProject;
assert(exist('simulationData','var') == 1, 'Run runMission first.');
assert(exist('waypoints','var') == 1 && exist('spraywaypoints','var') == 1, ...
    'Run plotSortieWaypoints first.');
if ~exist('selectedSortie','var'), selectedSortie = 1; end
assert(selectedSortie <= numel(waypoints) && ~isempty(waypoints{selectedSortie}), ...
    'No waypoints are available for the selected sortie.');
initialiseAircraft;
simModelName = configureFlightModel(selectedSortie, simulationData);

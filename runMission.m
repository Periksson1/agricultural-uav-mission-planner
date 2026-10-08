%RUNMISSION Plan and simulate the default SA farm spraying mission.
% Optional inputs in the workspace: layoutOptions, missionCfgOverrides,
% missionRunOptions (createFigures, printSummary).
setupProject;
if ~exist('layoutOptions', 'var'), layoutOptions = struct(); end
if ~exist('missionCfgOverrides', 'var'), missionCfgOverrides = struct(); end
if ~exist('missionRunOptions', 'var'), missionRunOptions = struct(); end
[missionCfg, planningData, simulationData, plotData] = ...
    runBaselineMission(missionCfgOverrides, layoutOptions);
selectedSortie = 1;
% Avoid comparing a new mission to an old flight output.
clear out pos t simulatedSortie waypoints spraywaypoints selectedWaypointSortie sortieWaypointExport

%% -------------------- EXPORTED VARIABLES --------------------
% Planned / simulated paths
basePathXYZ = planningData.basePathXYZ;
basePathRollAngle = planningData.basePathRollAngle;
basePathFlightPathAngle = planningData.basePathFlightPathAngle;
basePathTime = planningData.basePathTime;
baseSegmentModes = planningData.baseSegmentModes;
baseSegmentStages = planningData.baseSegmentStages;
baseSegmentPass = planningData.baseSegmentPass;
baseSegmentPowerW = planningData.baseSegmentPowerW;
fieldInfo = planningData.fieldInfo;
fieldFirstCheckpoints = planningData.fieldFirstCheckpoints;
fieldSummaryLines = planningData.fieldSummaryLines;
fieldVisitOrder = planningData.fieldVisitOrder;
laneMissionStructs = planningData.laneMissionStructs;

pathXYZ = simulationData.pathXYZ;
pathRollAngle = simulationData.pathRollAngle;
pathFlightPathAngle = simulationData.pathFlightPathAngle;
pathTime = simulationData.pathTime;
segmentTimes = simulationData.segmentTimes;
segmentModes = simulationData.segmentModes;
segmentStages = simulationData.segmentStages;
segmentPass = simulationData.segmentPass;
segmentSortie = simulationData.segmentSortie;
segmentPowerW = simulationData.segmentPowerW;
pathHeadingAngle = simulationData.pathHeadingAngle;
pathHeadingAngleUnwrapped = simulationData.pathHeadingAngleUnwrapped;

payloadRemaining_L = simulationData.payloadRemaining_L;
batteryRemaining_Wh = simulationData.batteryRemaining_Wh;
batteryPct = simulationData.batteryPct;
totalLitresDispensed = simulationData.totalLitresDispensed;
sprayedArea_ha = simulationData.sprayedArea_ha;
droneWeightKg = simulationData.droneWeightKg;

sortieLog = simulationData.sortieLog;
payloadInterruptPoints = simulationData.payloadInterruptPoints;
batteryInterruptPoints = simulationData.batteryInterruptPoints;
payloadInterruptSorties = simulationData.payloadInterruptSorties;
batteryInterruptSorties = simulationData.batteryInterruptSorties;

sortieCommandExports = simulationData.sortieCommandExports;
rollCmdTimeBySortie = simulationData.rollCmdTimeBySortie;
rollCmdRadBySortie = simulationData.rollCmdRadBySortie;
rollCmdDegBySortie = simulationData.rollCmdDegBySortie;
rollCmdMatrixBySortie = simulationData.rollCmdMatrixBySortie;
rollCmdTSBySortie = simulationData.rollCmdTSBySortie;
flightPathCmdTimeBySortie = simulationData.flightPathCmdTimeBySortie;
flightPathCmdRadBySortie = simulationData.flightPathCmdRadBySortie;
flightPathCmdDegBySortie = simulationData.flightPathCmdDegBySortie;
flightPathCmdMatrixBySortie = simulationData.flightPathCmdMatrixBySortie;
flightPathCmdTSBySortie = simulationData.flightPathCmdTSBySortie;
headingCmdTimeBySortie = simulationData.headingCmdTimeBySortie;
headingCmdRadBySortie = simulationData.headingCmdRadBySortie;
headingCmdDegBySortie = simulationData.headingCmdDegBySortie;
headingCmdMatrixBySortie = simulationData.headingCmdMatrixBySortie;
headingCmdTSBySortie = simulationData.headingCmdTSBySortie;
altitudeCmdTimeBySortie = simulationData.altitudeCmdTimeBySortie;
altitudeCmdMBySortie = simulationData.altitudeCmdMBySortie;
altitudeCmdMatrixBySortie = simulationData.altitudeCmdMatrixBySortie;
altitudeCmdTSBySortie = simulationData.altitudeCmdTSBySortie;
upCmdTimeBySortie = simulationData.upCmdTimeBySortie;
upCmdMBySortie = simulationData.upCmdMBySortie;
upCmdMatrixBySortie = simulationData.upCmdMatrixBySortie;
upCmdTSBySortie = simulationData.upCmdTSBySortie;
initialHeadingAfterVTOLRadBySortie = simulationData.initialHeadingAfterVTOLRadBySortie;
initialHeadingAfterVTOLDegBySortie = simulationData.initialHeadingAfterVTOLDegBySortie;
initialHeightAfterVTOL_mBySortie = simulationData.initialHeightAfterVTOL_mBySortie;
initialEastAfterVTOL_mBySortie = simulationData.initialEastAfterVTOL_mBySortie;
initialNorthAfterVTOL_mBySortie = simulationData.initialNorthAfterVTOL_mBySortie;
initialUpAfterVTOL_mBySortie = simulationData.initialUpAfterVTOL_mBySortie;
initialCruiseStartMissionTimeBySortie = simulationData.initialCruiseStartMissionTimeBySortie;
initialCruiseStartXYZBySortie = simulationData.initialCruiseStartXYZBySortie;
InitialPosBySortie = simulationData.InitialPosBySortie;
InitialHeadingRadBySortie = simulationData.InitialHeadingRadBySortie;
InitialHeadingDegBySortie = simulationData.InitialHeadingDegBySortie;
sortieInitialConditionExports = simulationData.sortieInitialConditionExports;
commandExportBundle = simulationData.commandExportBundle;
missionExport = simulationData.missionExport;

% Backward-compatible first-sortie aliases
rollCmdTime = simulationData.rollCmdTime;
rollCmdRad = simulationData.rollCmdRad;
rollCmdDeg = simulationData.rollCmdDeg;
rollCmdMatrix = simulationData.rollCmdMatrix;
rollCmdTS = simulationData.rollCmdTS;
flightPathCmdTime = simulationData.flightPathCmdTime;
flightPathCmdRad = simulationData.flightPathCmdRad;
flightPathCmdDeg = simulationData.flightPathCmdDeg;
flightPathCmdMatrix = simulationData.flightPathCmdMatrix;
flightPathCmdTS = simulationData.flightPathCmdTS;
headingCmdTime = simulationData.headingCmdTime;
headingCmdRad = simulationData.headingCmdRad;
headingCmdDeg = simulationData.headingCmdDeg;
headingCmdMatrix = simulationData.headingCmdMatrix;
headingCmdTS = simulationData.headingCmdTS;
altitudeCmdTime = simulationData.altitudeCmdTime;
altitudeCmdM = simulationData.altitudeCmdM;
altitudeCmdMatrix = simulationData.altitudeCmdMatrix;
altitudeCmdTS = simulationData.altitudeCmdTS;
upCmdTime = simulationData.upCmdTime;
upCmdM = simulationData.upCmdM;
upCmdMatrix = simulationData.upCmdMatrix;
upCmdTS = simulationData.upCmdTS;
initialHeadingAfterVTOLRad = simulationData.initialHeadingAfterVTOLRad;
initialHeadingAfterVTOLDeg = simulationData.initialHeadingAfterVTOLDeg;
initialHeightAfterVTOL_m = simulationData.initialHeightAfterVTOL_m;
initialEastAfterVTOL_m = simulationData.initialEastAfterVTOL_m;
initialNorthAfterVTOL_m = simulationData.initialNorthAfterVTOL_m;
initialUpAfterVTOL_m = simulationData.initialUpAfterVTOL_m;

%% -------------------- FIGURES --------------------
if getMissionRunOption(missionRunOptions, 'createFigures', true)
    createMissionFigure(missionCfg, planningData, simulationData, plotData);
    createResultsDashboard(missionCfg, planningData, simulationData);
end
if getMissionRunOption(missionRunOptions, 'printSummary', true)
    printMissionSummary(missionCfg, planningData, simulationData);
end

function value = getMissionRunOption(options, fieldName, defaultValue)
    value = defaultValue;
    if isstruct(options) && isfield(options, fieldName)
        value = options.(fieldName);
    end
end

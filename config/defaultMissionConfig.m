function missionCfg = defaultMissionConfig(layoutOptions)
%DEFAULTMISSIONCONFIG Mission defaults and reproducible SA farm example.
% layoutOptions.mode: 'realWorld' (default), 'interactive', 'saved', 'manual'.
% layoutOptions.fieldPath: GeoJSON/CSV path; '' opens the file chooser.
% layoutOptions.selectGroundStation: true opens interactive GS placement.
% layoutOptions.groundStationXY: optional fixed [East North] position in metres.
% layoutOptions.fields: required for manual mode.
% layoutOptions.savedLayoutPath: required for a custom saved layout.
if nargin < 1, layoutOptions = struct(); end
layoutInputMode = getOption(layoutOptions, 'mode', 'realWorld');
useInteractiveLayoutDesigner = strcmpi(layoutInputMode, 'interactive');
% Spray
sprayWidth = 6;
overlap    = 0;

% Payload / application
payloadLitres   = 47;
applicationRate = 15;   % [L/ha]
initialWeightKg = 145;

% Flight
flightHeight = 4;
uavAirSpeed  = 25.8;
maxRollAngle = deg2rad(30);

% Inter-pass turn model
turnClimbAltitude_m       = 15.0;
interPassClimbAngle_deg   = 5.0;
interPassDescentAngle_deg = 5.0;
preDescentLevelDistance_m = 0.0;
preSprayLevelDistance_m   = 75.0;
postSprayLevelDistance_m  = 20.0;

% Return-to-GS pull-up model
% The aircraft now climbs straight ahead to this altitude before starting
% the return turn back toward the ground station.
returnPullUpAltitude_m = 15.0;
returnPullUpClimbAngle_deg = interPassClimbAngle_deg;
interruptReturnRunOutDistance_m = 25.0;

% VTOL / GS transfer model
% The UAV climbs vertically to vtolAltitude_m at the ground station for
% VTOL energy modelling, then uses fieldTransferAltitude_m for first-field
% entry and inter-field transfers before descending into spray height.
vtolAltitude_m          = 20;
fieldTransferAltitude_m = 15.0;
vtolClimbRate_mps       = 2.5;
vtolHoverTime_s         = 5.0;
vtolHoverPowerDraw_kW   = 37.7;
vtolClimbPowerDraw_kW   = 49.3;

% Ground station
groundStationNorth_m = -200;
groundStationEast_m  = 1000;
groundStationSize    = 10;
missionGeodata = struct();


switch lower(layoutInputMode)
    case 'realworld'
        fieldPath = getOption(layoutOptions, 'fieldPath', ...
            fullfile(projectRoot(), 'examples', 'data', 'SAFarmboundaries.geojson'));
        selectGS = getOption(layoutOptions, 'selectGroundStation', false);
        [fields, fieldCount, fieldVisitOrder, groundStationEast_m, groundStationNorth_m, missionGeodata] = ...
            importRealWorldMissionLayout(groundStationEast_m, groundStationNorth_m, groundStationSize, fieldPath, selectGS);
    case 'saved'
        savedPath = getOption(layoutOptions, 'savedLayoutPath', ...
            fullfile(projectRoot(), 'examples', 'data', 'rectangularLayout.mat'));
        [fields, fieldCount, fieldVisitOrder, groundStationEast_m, groundStationNorth_m, missionGeodata] = loadSavedMissionLayout(savedPath);
    case 'interactive'
        [fields, fieldCount, fieldVisitOrder, groundStationEast_m, groundStationNorth_m] = ...
            designMissionLayoutInteractive(3, groundStationSize, [-200 1400], [-400 1000], ...
            groundStationEast_m, groundStationNorth_m, [90 70 1220 840], 220, 160, 0);
    case 'manual'
        assert(isfield(layoutOptions, 'fields') && ~isempty(layoutOptions.fields), ...
            'Manual mode requires layoutOptions.fields.');
        fields = layoutOptions.fields;
        fieldCount = numel(fields);
        fieldVisitOrder = 1:fieldCount;
    otherwise
        error('Unknown layout mode: %s', layoutInputMode);
end
if isfield(layoutOptions, 'groundStationXY')
    validateattributes(layoutOptions.groundStationXY, {'numeric'}, {'vector','numel',2,'finite'});
    groundStationEast_m = layoutOptions.groundStationXY(1);
    groundStationNorth_m = layoutOptions.groundStationXY(2);
end
%% -------------------- EFFICIENCY PLANNING --------------------
planningOptions = struct();
planningOptions.optimizeFieldVisitOrder = true;
planningOptions.optimizePolygonLaneAngle = true;
planningOptions.candidateLaneAngleStep_deg = 15;
planningOptions.deferShortEdgeLanes = true;
planningOptions.shortLaneLengthRatio = 0.65;
planningOptions.shortLaneAbsoluteThreshold_m = max(2*sprayWidth, 12);
planningOptions.entryDistanceWeight = 1.0;
planningOptions.turnConnectionWeight = 1.0;
planningOptions.shortLanePenalty_m = 4 * sprayWidth;
planningOptions.useOriginalFieldOrderAsTieBreak = true;

% Strip any legacy refactor-era planning flags that are no longer active.
planningOptions = sanitizePlanningOptions(planningOptions);

% Battery
batteryVoltage_V   = 88.8;
batteryCapacity_Ah = 66;
cruisePowerDraw_kW = 5.18;
cruiseClimbPowerDraw_kW = 12.9;

% RTB / turnaround logic
batteryReturnThreshold_pct = 20;
turnaroundTime_s           = 3*60;
batteryGuardCheckDt_s      = 30.0;

% Path sampling
dsLine = 1.0;
dsTurn = 1.0;

% Command export
rollCmdDt = 1.0;
rollCmdInterpMethod = 'linear';

%% -------------------- ECONOMICS --------------------
revenuePerHa_GBP = 20.0;
transportCost_GBP = 100.0;
workerRatePerHour_GBP = 20.0;
numberOfWorkers = 3;
batteryChargeCostPerReturn_GBP = 1.0;
maintenanceRatePerFlightHour_GBP = 20.0;

%% -------------------- VISUALISATION --------------------
displayDsSpray    = 5.0;
displayDsLaneTurn = 2.0;
displayDsTransit  = 3.0;

compressTurnaroundInPlayback = true;
turnaroundPlaybackTime_s     = 5.0;

playbackFPS = 20;
targetPlaybackDurationFull_s   = 100;
targetPlaybackDurationSortie_s = 60;

viewAzimuth_deg   = 35;
viewElevation_deg = 20;
verticalExaggeration = 6;

missionCfg = struct();
missionCfg.fields = fields;
missionCfg.fieldCount = fieldCount;
missionCfg.fieldVisitOrder = fieldVisitOrder;
missionCfg.useInteractiveLayoutDesigner = useInteractiveLayoutDesigner;
missionCfg.sprayWidth = sprayWidth;
missionCfg.overlap = overlap;
missionCfg.payloadLitres = payloadLitres;
missionCfg.applicationRate = applicationRate;
missionCfg.initialWeightKg = initialWeightKg;
missionCfg.flightHeight = flightHeight;
missionCfg.uavAirSpeed = uavAirSpeed;
missionCfg.maxRollAngle = maxRollAngle;
missionCfg.turnClimbAltitude_m = turnClimbAltitude_m;
missionCfg.interPassClimbAngle_deg = interPassClimbAngle_deg;
missionCfg.interPassDescentAngle_deg = interPassDescentAngle_deg;
missionCfg.preDescentLevelDistance_m = preDescentLevelDistance_m;
missionCfg.preSprayLevelDistance_m = preSprayLevelDistance_m;
missionCfg.postSprayLevelDistance_m = postSprayLevelDistance_m;
missionCfg.returnPullUpAltitude_m = returnPullUpAltitude_m;
missionCfg.returnPullUpClimbAngle_deg = returnPullUpClimbAngle_deg;
missionCfg.interruptReturnRunOutDistance_m = interruptReturnRunOutDistance_m;
missionCfg.vtolAltitude_m = vtolAltitude_m;
missionCfg.fieldTransferAltitude_m = fieldTransferAltitude_m;
missionCfg.vtolClimbRate_mps = vtolClimbRate_mps;
missionCfg.vtolHoverTime_s = vtolHoverTime_s;
missionCfg.vtolHoverPowerDraw_kW = vtolHoverPowerDraw_kW;
missionCfg.vtolClimbPowerDraw_kW = vtolClimbPowerDraw_kW;
missionCfg.groundStationNorth_m = groundStationNorth_m;
missionCfg.groundStationEast_m = groundStationEast_m;
missionCfg.groundStationSize = groundStationSize;
missionCfg.planningOptions = planningOptions;
missionCfg.batteryVoltage_V = batteryVoltage_V;
missionCfg.batteryCapacity_Ah = batteryCapacity_Ah;
missionCfg.cruisePowerDraw_kW = cruisePowerDraw_kW;
missionCfg.cruiseClimbPowerDraw_kW = cruiseClimbPowerDraw_kW;
missionCfg.batteryReturnThreshold_pct = batteryReturnThreshold_pct;
missionCfg.turnaroundTime_s = turnaroundTime_s;
missionCfg.batteryGuardCheckDt_s = batteryGuardCheckDt_s;
missionCfg.dsLine = dsLine;
missionCfg.dsTurn = dsTurn;
missionCfg.rollCmdDt = rollCmdDt;
missionCfg.rollCmdInterpMethod = rollCmdInterpMethod;
missionCfg.revenuePerHa_GBP = revenuePerHa_GBP;
missionCfg.transportCost_GBP = transportCost_GBP;
missionCfg.workerRatePerHour_GBP = workerRatePerHour_GBP;
missionCfg.numberOfWorkers = numberOfWorkers;
missionCfg.batteryChargeCostPerReturn_GBP = batteryChargeCostPerReturn_GBP;
missionCfg.maintenanceRatePerFlightHour_GBP = maintenanceRatePerFlightHour_GBP;
missionCfg.displayDsSpray = displayDsSpray;
missionCfg.displayDsLaneTurn = displayDsLaneTurn;
missionCfg.displayDsTransit = displayDsTransit;
missionCfg.compressTurnaroundInPlayback = compressTurnaroundInPlayback;
missionCfg.turnaroundPlaybackTime_s = turnaroundPlaybackTime_s;
missionCfg.playbackFPS = playbackFPS;
missionCfg.targetPlaybackDurationFull_s = targetPlaybackDurationFull_s;
missionCfg.targetPlaybackDurationSortie_s = targetPlaybackDurationSortie_s;
missionCfg.viewAzimuth_deg = viewAzimuth_deg;
missionCfg.viewElevation_deg = viewElevation_deg;
missionCfg.verticalExaggeration = verticalExaggeration;
missionCfg.geodata = missionGeodata;

validateMissionCfg(missionCfg);
end

function value = getOption(options, name, fallback)
value = fallback;
if isfield(options, name), value = options.(name); end
end

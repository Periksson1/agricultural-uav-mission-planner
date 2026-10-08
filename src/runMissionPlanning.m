function planningData = runMissionPlanning(missionCfg)
    validateMissionCfg(missionCfg);
    codes = missionCodes();

    fields = missionCfg.fields;
    fieldCount = missionCfg.fieldCount;
    fieldVisitOrder = missionCfg.fieldVisitOrder;
    sprayWidth = missionCfg.sprayWidth;
    overlap = missionCfg.overlap;
    applicationRate = missionCfg.applicationRate;
    flightHeight = missionCfg.flightHeight;
    uavAirSpeed = missionCfg.uavAirSpeed;
    maxRollAngle = missionCfg.maxRollAngle;
    turnClimbAltitude_m = missionCfg.turnClimbAltitude_m;
    interPassClimbAngle_deg = missionCfg.interPassClimbAngle_deg;
    interPassDescentAngle_deg = missionCfg.interPassDescentAngle_deg;
    preDescentLevelDistance_m = missionCfg.preDescentLevelDistance_m;
    preSprayLevelDistance_m = missionCfg.preSprayLevelDistance_m;
    postSprayLevelDistance_m = 20.0;
    if isfield(missionCfg, 'postSprayLevelDistance_m')
        postSprayLevelDistance_m = missionCfg.postSprayLevelDistance_m;
    end
    vtolAltitude_m = missionCfg.vtolAltitude_m;
    fieldTransferAltitude_m = missionCfg.fieldTransferAltitude_m;
    groundStationNorth_m = missionCfg.groundStationNorth_m;
    groundStationEast_m = missionCfg.groundStationEast_m;
    groundStationSize = missionCfg.groundStationSize;
    planningOptions = missionCfg.planningOptions;
    batteryVoltage_V = missionCfg.batteryVoltage_V;
    batteryCapacity_Ah = missionCfg.batteryCapacity_Ah;
    batteryReturnThreshold_pct = missionCfg.batteryReturnThreshold_pct;
    turnaroundTime_s = missionCfg.turnaroundTime_s;
    batteryGuardCheckDt_s = missionCfg.batteryGuardCheckDt_s;
    dsLine = missionCfg.dsLine;
    dsTurn = missionCfg.dsTurn;
    cruisePowerDraw_kW = missionCfg.cruisePowerDraw_kW;
    cruiseClimbPowerDraw_kW = missionCfg.cruiseClimbPowerDraw_kW;
    payloadLitres = missionCfg.payloadLitres;
    geodata = struct();
    if isfield(missionCfg, 'geodata') && isstruct(missionCfg.geodata)
        geodata = missionCfg.geodata;
    end
    terrainModel = getOptionalTerrainModel(geodata);

    %% -------------------- INPUT CHECKS --------------------
    assert(numel(fields) == fieldCount, 'fieldCount must match number of field definitions.');
    assert(numel(fieldVisitOrder) == fieldCount, 'fieldVisitOrder must have fieldCount entries.');
    assert(numel(unique(fieldVisitOrder)) == fieldCount && all(sort(fieldVisitOrder(:).') == 1:fieldCount), ...
        'fieldVisitOrder must be a permutation of 1:fieldCount.');

    assert(sprayWidth > 0, 'sprayWidth must be > 0.');
    assert(applicationRate > 0, 'applicationRate must be > 0.');
    assert(uavAirSpeed > 0, 'uavAirSpeed must be > 0.');
    assert(vtolAltitude_m >= flightHeight, 'vtolAltitude_m must be >= flightHeight.');
    assert(fieldTransferAltitude_m >= flightHeight, 'fieldTransferAltitude_m must be >= flightHeight.');
    assert(turnClimbAltitude_m >= flightHeight, 'turnClimbAltitude_m must be >= flightHeight.');
    assert(interPassClimbAngle_deg > 0 && interPassClimbAngle_deg < 89, 'interPassClimbAngle_deg invalid.');
    assert(interPassDescentAngle_deg > 0 && interPassDescentAngle_deg < 89, 'interPassDescentAngle_deg invalid.');
    assert(preDescentLevelDistance_m >= 0, 'preDescentLevelDistance_m must be >= 0.');
    assert(preSprayLevelDistance_m >= 0, 'preSprayLevelDistance_m must be >= 0.');
    assert(postSprayLevelDistance_m >= 0, 'postSprayLevelDistance_m must be >= 0.');
    assert(groundStationSize > 0, 'groundStationSize must be > 0.');
    assert(missionCfg.vtolClimbRate_mps > 0, 'vtolClimbRate_mps must be > 0.');
    assert(batteryVoltage_V > 0 && batteryCapacity_Ah > 0, 'Battery parameters must be > 0.');
    assert(batteryReturnThreshold_pct >= 0 && batteryReturnThreshold_pct < 100, 'batteryReturnThreshold_pct invalid.');
    assert(turnaroundTime_s >= 0, 'turnaroundTime_s must be >= 0.');
    assert(batteryGuardCheckDt_s > 0, 'batteryGuardCheckDt_s must be > 0.');
    assert(planningOptions.candidateLaneAngleStep_deg > 0 && planningOptions.candidateLaneAngleStep_deg <= 180, ...
        'planningOptions.candidateLaneAngleStep_deg must be in (0, 180].');

    laneSpacing = sprayWidth * (1 - overlap);
    assert(laneSpacing > 0, 'laneSpacing must be > 0.');

    processedFields = struct([]);
    for i = 1:fieldCount
        thisField = preprocessUserFieldDef(fields(i), sprayWidth);
        thisField.sourceFieldIndices = i;
        thisField.sourceFieldNames = {thisField.name};
        thisField.wasAutoCombined = false;
        thisField.memberBoundariesXY = {thisField.boundaryXY};
        thisField.displayBoundaryLoops = {thisField.boundaryXY};
        assert(thisField.area_ha > 0, 'Each field must have positive area.');
        assert(thisField.maxCrossTrackWidth_m >= sprayWidth, ...
            'Each field must be at least as wide as sprayWidth somewhere across the lanes.');
        if isempty(processedFields)
            processedFields = thisField;
        else
            processedFields(i,1) = thisField;
        end
    end
    fields = processedFields;

    originalFieldCount = fieldCount;
    fieldVisitOrderInput = fieldVisitOrder;
    combinedFieldCount = fieldCount;
    combinationInfo = struct('originalFieldCount', originalFieldCount, ...
        'combinedFieldCount', combinedFieldCount, 'didCombine', false, ...
        'groupMembers', arrayfun(@(k) k, (1:fieldCount).', 'UniformOutput', false));

    %% -------------------- DERIVED VALUES --------------------
    coverageWidthForPayload = laneSpacing;
    applicationRate_m2 = applicationRate / 10000;
    litresPerSprayMetre = applicationRate_m2 * coverageWidthForPayload;
    maxSprayDistancePerSortie = payloadLitres / litresPerSprayMetre;

    batteryEnergy_Wh  = batteryVoltage_V * batteryCapacity_Ah;
    batteryEnergy_kWh = batteryEnergy_Wh / 1000;
    cruisePowerW      = cruisePowerDraw_kW * 1000;
    cruiseClimbPowerW = cruiseClimbPowerDraw_kW * 1000;
    reserveBatteryWh  = batteryEnergy_Wh * batteryReturnThreshold_pct / 100;

    groundStationElevation_m = sampleTerrainElevation(terrainModel, [groundStationEast_m, groundStationNorth_m]);
    groundStationGroundXYZ = [groundStationEast_m, groundStationNorth_m, groundStationElevation_m];
    groundStationHighXYZ   = [groundStationEast_m, groundStationNorth_m, groundStationElevation_m + vtolAltitude_m];
    groundStationCentreXY  = [groundStationEast_m, groundStationNorth_m];

    groundStationHalfSize = groundStationSize/2;
    groundStationX = groundStationCentreXY(1) + groundStationHalfSize * [-1 1 1 -1];
    groundStationY = groundStationCentreXY(2) + groundStationHalfSize * [-1 -1 1 1];
    groundStationZ = groundStationElevation_m * [1 1 1 1];

    totalFieldArea_ha = 0;
    for i = 1:fieldCount
        totalFieldArea_ha = totalFieldArea_ha + fields(i).area_ha;
    end

    %% -------------------- UAV TURN OBJECT --------------------
    uavConn = uavDubinsConnection;
    uavConn.AirSpeed = uavAirSpeed;
    uavConn.MaxRollAngle = maxRollAngle;
    actualTurnRadius = uavConn.MinTurningRadius;

    if planningOptions.optimizeFieldVisitOrder
        fieldVisitOrder = optimizeFieldVisitOrderGreedy(fields, fieldVisitOrderInput, groundStationCentreXY, ...
            sprayWidth, laneSpacing, flightHeight, postSprayLevelDistance_m, planningOptions);
    end
    fieldVisitOrderWasOptimized = ~isequal(fieldVisitOrder(:).', fieldVisitOrderInput(:).');

    %% -------------------- BUILD BASE PATH ACROSS ALL FIELDS --------------------
    basePathXYZ = zeros(0,3);
    basePathRollAngle = zeros(0,1);
    basePathFlightPathAngle = zeros(0,1);
    baseSegmentTimes = zeros(0,1);
    baseSegmentModes = zeros(0,1);
    baseSegmentStages = zeros(0,1);
    baseSegmentPass = nan(0,1);
    baseSegmentPowerW = zeros(0,1);

    fieldInfo(fieldCount) = struct( ...
        'name','', 'centreXY',[0 0], 'boundaryXYZ',zeros(0,3), ...
        'firstCheckpointXYZ',[0 0 0], 'firstHeading',0, ...
        'lastEndXYZ',[0 0 0], 'lastHeading',0, ...
        'nLanes',0, 'laneTurnCount',0, 'area_ha',0, ...
        'sprayAxisText','', 'lanePassNumbers',zeros(0,1), 'lanePatchLoops',{{}}, ...
        'displayBoundaryLoops',{{}}, 'chosenLaneAngle_deg',NaN, 'deferredShortLaneCount',0, ...
        'entryStrategyText','', 'lanePlanScore_m',NaN, 'fieldPathLength_m',NaN, ...
        'laneMissionStructs', repmat(makeEmptyLaneMissionStruct(),0,1));

    fieldSummaryLines = cell(fieldCount,1);
    fieldFirstCheckpoints = zeros(fieldCount,3);
    laneMissionStructs = repmat(makeEmptyLaneMissionStruct(), 0, 1);

    anchorXY = groundStationCentreXY;
    globalPassOffset = 0;
    totalLaneCount = 0;
    laneTurnCount = 0;

    for visitIdx = 1:fieldCount
        fIdx = fieldVisitOrder(visitIdx);
        [fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, ...
            fieldSegPass, fieldSegPowerW, info] = ...
            buildPolygonFieldBasePath(fields(fIdx), anchorXY, globalPassOffset, ...
            sprayWidth, laneSpacing, flightHeight, turnClimbAltitude_m, ...
            interPassClimbAngle_deg, interPassDescentAngle_deg, ...
            preDescentLevelDistance_m, preSprayLevelDistance_m, postSprayLevelDistance_m, dsLine, dsTurn, ...
            uavAirSpeed, cruisePowerW, cruiseClimbPowerW, planningOptions, uavConn);

        if visitIdx == 1
            firstCheckpointXYZ = info.firstCheckpointXYZ;
        else
            prevFIdx = fieldVisitOrder(visitIdx-1);
            [p,r,g,st,sm,ss,sp,spw] = buildInterFieldTransferChunk(uavConn, ...
                fieldInfo(prevFIdx).lastEndXYZ, fieldInfo(prevFIdx).lastHeading, ...
                info.firstCheckpointXYZ, info.firstHeading, fieldTransferAltitude_m, ...
                interPassClimbAngle_deg, interPassDescentAngle_deg, preDescentLevelDistance_m, preSprayLevelDistance_m, ...
                dsLine, dsTurn, uavAirSpeed, cruisePowerW);
            [basePathXYZ, basePathRollAngle, basePathFlightPathAngle, ...
                baseSegmentTimes, baseSegmentModes, baseSegmentStages, ...
                baseSegmentPass, baseSegmentPowerW] = ...
                appendChunkData(basePathXYZ, basePathRollAngle, basePathFlightPathAngle, ...
                baseSegmentTimes, baseSegmentModes, baseSegmentStages, ...
                baseSegmentPass, baseSegmentPowerW, ...
                p,r,g,st,sm,ss,sp,spw);
        end

        [basePathXYZ, basePathRollAngle, basePathFlightPathAngle, ...
            baseSegmentTimes, baseSegmentModes, baseSegmentStages, ...
            baseSegmentPass, baseSegmentPowerW] = ...
            appendChunkData(basePathXYZ, basePathRollAngle, basePathFlightPathAngle, ...
            baseSegmentTimes, baseSegmentModes, baseSegmentStages, ...
            baseSegmentPass, baseSegmentPowerW, ...
            fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, ...
            fieldSegPass, fieldSegPowerW);

        fieldInfo(fIdx) = info;
        fieldFirstCheckpoints(fIdx,:) = info.firstCheckpointXYZ;
        if isfield(info, 'laneMissionStructs') && ~isempty(info.laneMissionStructs)
            for iLaneStruct = 1:numel(info.laneMissionStructs)
                info.laneMissionStructs(iLaneStruct).fieldIndex = fIdx;
                info.laneMissionStructs(iLaneStruct).visitIndex = visitIdx;
            end
            laneMissionStructs = [laneMissionStructs; info.laneMissionStructs(:)]; %#ok<AGROW>
        end
        totalLaneCount = totalLaneCount + info.nLanes;
        laneTurnCount = laneTurnCount + info.laneTurnCount;
        globalPassOffset = globalPassOffset + info.nLanes;

        memberText = '';
        if numel(fields(fIdx).sourceFieldIndices) > 1
            memberText = sprintf(' | Members %s', mat2str(fields(fIdx).sourceFieldIndices));
        end
        fieldSummaryLines{fIdx} = sprintf('%s | %s | Area %.3f ha | Lanes %d | Lane yaw %.1f deg | Deferred short edge lanes %d%s', ...
            fields(fIdx).name, fields(fIdx).summaryText, info.area_ha, info.nLanes, ...
            info.chosenLaneAngle_deg, info.deferredShortLaneCount, memberText);

        anchorXY = info.lastEndXYZ(1:2);
    end

    if ~isempty(terrainModel)
        basePathXYZ = addTerrainToRelativeAltitudePath(basePathXYZ, terrainModel);
        [baseSegmentTimes, basePathFlightPathAngle] = recomputePathTimingAndGamma(basePathXYZ, uavAirSpeed);
        [fieldInfo, fieldFirstCheckpoints, laneMissionStructs] = ...
            addTerrainToStoredMissionGeometry(fieldInfo, fieldFirstCheckpoints, laneMissionStructs, terrainModel);
        firstCheckpointXYZ = fieldInfo(fieldVisitOrder(1)).firstCheckpointXYZ;
    end

    groundStationToFirstCheckpointDistance = norm(firstCheckpointXYZ(1:2) - groundStationCentreXY);

    basePathTime = [0; cumsum(baseSegmentTimes)];
    baseSegmentLengths = vecnorm(diff(basePathXYZ,1,1),2,2);
    baseSegmentIsSpray = (baseSegmentModes == codes.mode.spray);
    [passStartBaseTime, passEndBaseTime] = computePassTimeBounds(baseSegmentStages, baseSegmentPass, basePathTime, totalLaneCount);

    plannedSprayDistance = sum(baseSegmentLengths(baseSegmentIsSpray));
    plannedSprayArea_ha  = plannedSprayDistance * coverageWidthForPayload / 10000;

    baseCumSprayDistance = zeros(numel(basePathTime),1);
    baseCumSprayDistance(2:end) = cumsum(baseSegmentLengths .* baseSegmentIsSpray);
    FsSpray = griddedInterpolant(basePathTime, baseCumSprayDistance, 'linear');

    baseCumEnergyWh = zeros(numel(basePathTime),1);
    baseCumEnergyWh(2:end) = cumsum(baseSegmentPowerW .* baseSegmentTimes / 3600);
    FBaseEnergyWh = griddedInterpolant(basePathTime, baseCumEnergyWh, 'linear');

    FbX = griddedInterpolant(basePathTime, basePathXYZ(:,1), 'linear');
    FbY = griddedInterpolant(basePathTime, basePathXYZ(:,2), 'linear');
    FbZ = griddedInterpolant(basePathTime, basePathXYZ(:,3), 'linear');
    FbRoll = griddedInterpolant(basePathTime, basePathRollAngle, 'linear');
    FbGamma = griddedInterpolant(basePathTime, basePathFlightPathAngle, 'linear');

    for iLaneStruct = 1:numel(laneMissionStructs)
        laneMissionStructs(iLaneStruct).laneSequenceNumber = iLaneStruct;
        if iLaneStruct < numel(laneMissionStructs)
            laneMissionStructs(iLaneStruct).hasFollowingLane = true;
            laneMissionStructs(iLaneStruct).nextLanePassNumber = laneMissionStructs(iLaneStruct+1).passNumber;
            laneMissionStructs(iLaneStruct).nextLaneFieldIndex = laneMissionStructs(iLaneStruct+1).fieldIndex;
            laneMissionStructs(iLaneStruct).nextLaneStartXYZ = laneMissionStructs(iLaneStruct+1).startXYZ;
        else
            laneMissionStructs(iLaneStruct).hasFollowingLane = false;
            laneMissionStructs(iLaneStruct).nextLanePassNumber = NaN;
            laneMissionStructs(iLaneStruct).nextLaneFieldIndex = NaN;
            laneMissionStructs(iLaneStruct).nextLaneStartXYZ = [NaN NaN NaN];
        end
    end

    planningData = struct();
    planningData.fields = fields;
    planningData.fieldCount = fieldCount;
    planningData.fieldVisitOrder = fieldVisitOrder;
    planningData.fieldVisitOrderInput = fieldVisitOrderInput;
    planningData.fieldVisitOrderWasOptimized = fieldVisitOrderWasOptimized;
    planningData.originalFieldCount = originalFieldCount;
    planningData.combinedFieldCount = combinedFieldCount;
    planningData.combinationInfo = combinationInfo;
    planningData.laneSpacing = laneSpacing;
    planningData.coverageWidthForPayload = coverageWidthForPayload;
    planningData.applicationRate_m2 = applicationRate_m2;
    planningData.litresPerSprayMetre = litresPerSprayMetre;
    planningData.maxSprayDistancePerSortie = maxSprayDistancePerSortie;
    planningData.postSprayLevelDistance_m = postSprayLevelDistance_m;
    planningData.batteryEnergy_Wh = batteryEnergy_Wh;
    planningData.batteryEnergy_kWh = batteryEnergy_kWh;
    planningData.cruisePowerW = cruisePowerW;
    planningData.cruiseClimbPowerW = cruiseClimbPowerW;
    planningData.reserveBatteryWh = reserveBatteryWh;
    planningData.groundStationGroundXYZ = groundStationGroundXYZ;
    planningData.groundStationHighXYZ = groundStationHighXYZ;
    planningData.fieldTransferAltitude_m = fieldTransferAltitude_m;
    planningData.groundStationCentreXY = groundStationCentreXY;
    planningData.groundStationX = groundStationX;
    planningData.groundStationY = groundStationY;
    planningData.groundStationZ = groundStationZ;
    planningData.totalFieldArea_ha = totalFieldArea_ha;
    planningData.uavConn = uavConn;
    planningData.actualTurnRadius = actualTurnRadius;
    planningData.basePathXYZ = basePathXYZ;
    planningData.basePathRollAngle = basePathRollAngle;
    planningData.basePathFlightPathAngle = basePathFlightPathAngle;
    planningData.baseSegmentTimes = baseSegmentTimes;
    planningData.baseSegmentModes = baseSegmentModes;
    planningData.baseSegmentStages = baseSegmentStages;
    planningData.baseSegmentPass = baseSegmentPass;
    planningData.baseSegmentPowerW = baseSegmentPowerW;
    planningData.basePathTime = basePathTime;
    planningData.baseSegmentLengths = baseSegmentLengths;
    planningData.passStartBaseTime = passStartBaseTime;
    planningData.passEndBaseTime = passEndBaseTime;
    planningData.plannedSprayDistance = plannedSprayDistance;
    planningData.plannedSprayArea_ha = plannedSprayArea_ha;
    planningData.FsSpray = FsSpray;
    planningData.baseCumEnergyWh = baseCumEnergyWh;
    planningData.FBaseEnergyWh = FBaseEnergyWh;
    planningData.FbX = FbX;
    planningData.FbY = FbY;
    planningData.FbZ = FbZ;
    planningData.FbRoll = FbRoll;
    planningData.FbGamma = FbGamma;
    planningData.fieldInfo = fieldInfo;
    planningData.laneMissionStructs = laneMissionStructs;
    planningData.fieldSummaryLines = fieldSummaryLines;
    planningData.fieldFirstCheckpoints = fieldFirstCheckpoints;
    planningData.totalLaneCount = totalLaneCount;
    planningData.laneTurnCount = laneTurnCount;
    planningData.groundStationToFirstCheckpointDistance = groundStationToFirstCheckpointDistance;
    planningData.firstCheckpointXYZ = firstCheckpointXYZ;
    planningData.geodata = geodata;
    validatePlanningData(planningData);
end

function terrainModel = getOptionalTerrainModel(geodata)
    terrainModel = [];
    if ~isstruct(geodata) || ~isfield(geodata, 'terrainModel') || ~isstruct(geodata.terrainModel)
        return
    end
    if isfield(geodata.terrainModel, 'pointsXY') && isfield(geodata.terrainModel, 'elevationM') && ...
            ~isempty(geodata.terrainModel.pointsXY) && ~isempty(geodata.terrainModel.elevationM)
        terrainModel = geodata.terrainModel;
    end
end

function elevation_m = sampleTerrainElevation(terrainModel, xy)
    if isempty(terrainModel)
        elevation_m = zeros(size(xy,1), 1);
        return
    end

    interpolant = scatteredInterpolant(terrainModel.pointsXY(:,1), terrainModel.pointsXY(:,2), ...
        terrainModel.elevationM(:), 'natural', 'nearest');
    elevation_m = interpolant(xy(:,1), xy(:,2));
end

function xyzOut = addTerrainToRelativeAltitudePath(xyzIn, terrainModel)
    xyzOut = xyzIn;
    if isempty(xyzOut)
        return
    end

    terrainZ = sampleTerrainElevation(terrainModel, xyzOut(:,1:2));
    xyzOut(:,3) = terrainZ(:) + xyzIn(:,3);
end

function [segTimes, gamma] = recomputePathTimingAndGamma(pathXYZ, uavAirSpeed)
    if size(pathXYZ,1) < 2
        segTimes = zeros(0,1);
        gamma = zeros(size(pathXYZ,1),1);
        return
    end

    dXYZ = diff(pathXYZ, 1, 1);
    segLengths = vecnorm(dXYZ, 2, 2);
    segTimes = segLengths / uavAirSpeed;
    segGamma = atan2(dXYZ(:,3), max(vecnorm(dXYZ(:,1:2), 2, 2), eps));
    gamma = [segGamma(1); segGamma];
end

function [fieldInfo, fieldFirstCheckpoints, laneMissionStructs] = ...
    addTerrainToStoredMissionGeometry(fieldInfo, fieldFirstCheckpoints, laneMissionStructs, terrainModel)

    for iField = 1:numel(fieldInfo)
        fieldInfo(iField).boundaryXYZ = addTerrainToGroundLoop(fieldInfo(iField).boundaryXYZ, terrainModel);
        fieldInfo(iField).firstCheckpointXYZ = addTerrainToSingleRelativePoint(fieldInfo(iField).firstCheckpointXYZ, terrainModel);
        fieldInfo(iField).lastEndXYZ = addTerrainToSingleRelativePoint(fieldInfo(iField).lastEndXYZ, terrainModel);
        for iLane = 1:numel(fieldInfo(iField).laneMissionStructs)
            fieldInfo(iField).laneMissionStructs(iLane) = ...
                addTerrainToLaneMissionStruct(fieldInfo(iField).laneMissionStructs(iLane), terrainModel);
        end
        if iField <= size(fieldFirstCheckpoints,1)
            fieldFirstCheckpoints(iField,:) = fieldInfo(iField).firstCheckpointXYZ;
        end
    end

    for iLane = 1:numel(laneMissionStructs)
        laneMissionStructs(iLane) = addTerrainToLaneMissionStruct(laneMissionStructs(iLane), terrainModel);
    end
end

function xyzOut = addTerrainToGroundLoop(xyzIn, terrainModel)
    xyzOut = xyzIn;
    if isempty(xyzOut)
        return
    end
    xyzOut(:,3) = sampleTerrainElevation(terrainModel, xyzOut(:,1:2));
end

function xyzOut = addTerrainToSingleRelativePoint(xyzIn, terrainModel)
    xyzOut = xyzIn;
    if isempty(xyzOut) || any(~isfinite(xyzOut(1:2)))
        return
    end
    xyzOut(3) = xyzIn(3) + sampleTerrainElevation(terrainModel, xyzIn(1:2));
end

function laneStruct = addTerrainToLaneMissionStruct(laneStruct, terrainModel)
    laneStruct.startXYZ = addTerrainToSingleRelativePoint(laneStruct.startXYZ, terrainModel);
    laneStruct.endXYZ = addTerrainToSingleRelativePoint(laneStruct.endXYZ, terrainModel);
    laneStruct.runInStartXYZ = addTerrainToSingleRelativePoint(laneStruct.runInStartXYZ, terrainModel);
    laneStruct.runOutEndXYZ = addTerrainToSingleRelativePoint(laneStruct.runOutEndXYZ, terrainModel);
    laneStruct.nextLaneStartXYZ = addTerrainToSingleRelativePoint(laneStruct.nextLaneStartXYZ, terrainModel);
end

function laneStruct = makeEmptyLaneMissionStruct()
    laneStruct = struct( ...
        'fieldIndex', NaN, ...
        'fieldName', '', ...
        'visitIndex', NaN, ...
        'localLaneIndex', NaN, ...
        'laneSequenceNumber', NaN, ...
        'passNumber', NaN, ...
        'direction', NaN, ...
        'headingRad', NaN, ...
        'headingDeg', NaN, ...
        'startXYZ', [NaN NaN NaN], ...
        'endXYZ', [NaN NaN NaN], ...
        'runInStartXYZ', [NaN NaN NaN], ...
        'runOutEndXYZ', [NaN NaN NaN], ...
        'sprayLength_m', NaN, ...
        'pathLength_m', NaN, ...
        'yLocal_m', NaN, ...
        'intervalsLocal', zeros(0,2), ...
        'intervalsOrderedLocal', zeros(0,2), ...
        'footprintLoopsXY', {{}}, ...
        'preDescentLevelDistance_m', NaN, ...
        'preSprayLevelDistance_m', NaN, ...
        'postSprayLevelDistance_m', NaN, ...
        'hasFollowingLane', false, ...
        'nextLanePassNumber', NaN, ...
        'nextLaneFieldIndex', NaN, ...
        'nextLaneStartXYZ', [NaN NaN NaN]);
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

function fieldDef = preprocessUserFieldDef(fieldDef, sprayWidth)
    tol = 1e-9;

    if ~isfield(fieldDef, 'name') || isempty(fieldDef.name)
        fieldDef.name = 'Field';
    end

    if ~isfield(fieldDef, 'type') || isempty(fieldDef.type)
        if isfield(fieldDef, 'boundaryXY') && ~isempty(fieldDef.boundaryXY)
            fieldDef.type = 'polygon';
        else
            fieldDef.type = 'rectangle';
        end
    end

    switch lower(fieldDef.type)
        case 'rectangle'
            assert(isfield(fieldDef, 'centreEast_m') && isfield(fieldDef, 'centreNorth_m'), ...
                'Rectangle field must define centreEast_m and centreNorth_m.');
            assert(isfield(fieldDef, 'length_m') && isfield(fieldDef, 'width_m') && isfield(fieldDef, 'rotation_deg'), ...
                'Rectangle field must define length_m, width_m, and rotation_deg.');

            L = fieldDef.length_m;
            B = fieldDef.width_m;
            assert(L > 0 && B > 0, 'Rectangle field dimensions must be > 0.');

            if L >= B
                xSpan = L;
                ySpan = B;
                psi = deg2rad(fieldDef.rotation_deg);
            else
                xSpan = B;
                ySpan = L;
                psi = deg2rad(fieldDef.rotation_deg + 90);
            end

            boundaryLocalXY = [ ...
                -xSpan/2, -ySpan/2;
                 xSpan/2, -ySpan/2;
                 xSpan/2,  ySpan/2;
                -xSpan/2,  ySpan/2];

            originXY = [fieldDef.centreEast_m, fieldDef.centreNorth_m];
            boundaryXY = localPointsToGlobal(boundaryLocalXY, originXY, psi);

            fieldDef.boundaryXY = boundaryXY;
            fieldDef.boundaryLocalXY = boundaryLocalXY;
            fieldDef.localFrameOriginXY = originXY;
            fieldDef.localFrameYaw_rad = psi;
            fieldDef.area_ha = abs(polyarea(boundaryXY(:,1), boundaryXY(:,2))) / 10000;
            fieldDef.maxCrossTrackWidth_m = ySpan;
            fieldDef.summaryText = sprintf('Rectangle %.0f x %.0f m, rot %.1f deg', ...
                L, B, fieldDef.rotation_deg);

        case 'polygon'
            assert(isfield(fieldDef, 'boundaryXY'), 'Polygon field must define boundaryXY.');
            boundaryXY = fieldDef.boundaryXY;
            assert(size(boundaryXY,2) == 2 && size(boundaryXY,1) >= 3, ...
                'boundaryXY must be Nx2 with at least 3 vertices.');

            if norm(boundaryXY(1,:) - boundaryXY(end,:)) <= tol
                boundaryXY = boundaryXY(1:end-1,:);
            end
            assert(size(boundaryXY,1) >= 3, 'Polygon field must have at least 3 unique vertices.');

            areaSigned = polyarea(boundaryXY(:,1), boundaryXY(:,2));
            assert(abs(areaSigned) > tol, 'Polygon field area must be > 0.');

            originXY = mean(boundaryXY, 1);
            demeaned = boundaryXY - originXY;

            if size(demeaned,1) >= 2
                C = cov(demeaned, 1);
            else
                C = eye(2);
            end

            [V,D] = eig(C);
            [~, idxMax] = max(diag(D));
            majorVec = V(:, idxMax);

            psi = atan2(majorVec(2), majorVec(1));
            boundaryLocalXY = globalPointsToLocal(boundaryXY, originXY, psi);

            xSpan = max(boundaryLocalXY(:,1)) - min(boundaryLocalXY(:,1));
            ySpan = max(boundaryLocalXY(:,2)) - min(boundaryLocalXY(:,2));

            if ySpan > xSpan
                psi = psi + pi/2;
                boundaryLocalXY = globalPointsToLocal(boundaryXY, originXY, psi);
                ySpan = max(boundaryLocalXY(:,2)) - min(boundaryLocalXY(:,2));
            end

            fieldDef.boundaryXY = boundaryXY;
            fieldDef.boundaryLocalXY = boundaryLocalXY;
            fieldDef.localFrameOriginXY = originXY;
            fieldDef.localFrameYaw_rad = psi;
            fieldDef.area_ha = abs(polyarea(boundaryXY(:,1), boundaryXY(:,2))) / 10000;
            fieldDef.maxCrossTrackWidth_m = ySpan;
            fieldDef.summaryText = sprintf('Polygon %d vertices', size(boundaryXY,1));

        otherwise
            error('Unsupported field type "%s". Use rectangle or polygon.', fieldDef.type);
    end

    assert(fieldDef.maxCrossTrackWidth_m >= sprayWidth - tol, ...
        'Field cross-track width must be at least sprayWidth.');
end

function globalXY = localPointToGlobalXY(localUV, originXY, psi)
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    globalXY = (originXY(:) + R * localUV(:)).';
end

function globalXYZ = localXYZToGlobalXYZ(localXYZ, originXY, psi)
    globalXY = localPointToGlobalXY(localXYZ(1:2), originXY, psi);
    globalXYZ = [globalXY, localXYZ(3)];
end

function globalXY = localPointsToGlobal(localXY, originXY, psi)
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    globalXY = (R * localXY.').';
    globalXY(:,1) = globalXY(:,1) + originXY(1);
    globalXY(:,2) = globalXY(:,2) + originXY(2);
end

function localXY = globalPointsToLocal(globalXY, originXY, psi)
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    shifted = globalXY - originXY;
    localXY = (R.' * shifted.').';
end

function globalHeading = localHeadingToGlobal(localHeading, psi)
    localDir = [sin(localHeading); cos(localHeading)];
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    globalDir = R * localDir;
    globalHeading = atan2(globalDir(1), globalDir(2));
end

function intervals = collectLaneIntervalsInLoops(boundaryLocalLoops, yLane)
    intervals = zeros(0,2);
    for iLoop = 1:numel(boundaryLocalLoops)
        loopIntervals = collectLaneIntervalsInSinglePolygon(boundaryLocalLoops{iLoop}, yLane);
        if ~isempty(loopIntervals)
            intervals = [intervals; loopIntervals]; %#ok<AGROW>
        end
    end

    if isempty(intervals)
        return
    end

    intervals = sortrows(intervals, 1);
    merged = intervals(1,:);
    tol = 1e-9;
    for i = 2:size(intervals,1)
        if intervals(i,1) <= merged(end,2) + tol
            merged(end,2) = max(merged(end,2), intervals(i,2));
        else
            merged(end+1,:) = intervals(i,:); %#ok<AGROW>
        end
    end
    intervals = merged;
end

function intervals = collectLaneIntervalsInSinglePolygon(boundaryLocalXY, yLane)
    tol = 1e-10;
    xv = boundaryLocalXY(:,1);
    yv = boundaryLocalXY(:,2);
    n = size(boundaryLocalXY,1);

    xInts = zeros(0,1);

    for i = 1:n
        j = i + 1;
        if j > n
            j = 1;
        end

        x1 = xv(i); y1 = yv(i);
        x2 = xv(j); y2 = yv(j);

        if abs(y2 - y1) <= tol
            continue
        end

        crosses = ((y1 <= yLane) && (yLane < y2)) || ((y2 <= yLane) && (yLane < y1));
        if crosses
            xCross = x1 + (yLane - y1) * (x2 - x1) / (y2 - y1);
            xInts(end+1,1) = xCross; %#ok<AGROW>
        end
    end

    if numel(xInts) < 2
        intervals = zeros(0,2);
        return
    end

    xInts = sort(xInts(:));
    if mod(numel(xInts),2) ~= 0
        xInts = xInts(1:end-1);
    end

    intervals = zeros(0,2);
    for k = 1:2:numel(xInts)
        xa = xInts(k);
        xb = xInts(k+1);
        if xb <= xa + tol
            continue
        end

        xMid = 0.5 * (xa + xb);
        [in,on] = inpolygon(xMid, yLane, xv, yv);
        if in || on
            intervals(end+1,:) = [xa, xb]; %#ok<AGROW>
        end
    end
end

function plan = planEfficientFieldCoverage(fieldDef, anchorXY, sprayWidth, laneSpacing, flightHeight, postSprayLevelDistance_m, planningOptions)
    boundaryGlobalXY = fieldDef.boundaryXY;
    originXY = fieldDef.localFrameOriginXY;
    boundaryGlobalLoops = getFieldBoundaryLoops(fieldDef);
    psiCandidates = buildCandidateLaneAngleSet(fieldDef, planningOptions);

    bestScore = inf;
    bestPsi = NaN;
    bestBoundaryLocalLoops = {};
    bestLanePlan = struct([]);
    bestDeferredShortLaneCount = 0;
    bestApproxFieldPathLength_m = NaN;

    for iPsi = 1:numel(psiCandidates)
        psi = psiCandidates(iPsi);
        boundaryLocalLoops = cell(size(boundaryGlobalLoops));
        for iLoop = 1:numel(boundaryGlobalLoops)
            boundaryLocalLoops{iLoop} = globalPointsToLocal(boundaryGlobalLoops{iLoop}, originXY, psi);
        end

        laneData = buildLaneDataForOrientation(boundaryLocalLoops, sprayWidth, laneSpacing, planningOptions);
        if isempty(laneData)
            continue
        end

        orderCandidates = buildLaneSequenceCandidates(laneData, planningOptions);
        for iOrder = 1:numel(orderCandidates)
            seq = orderCandidates(iOrder).sequence;
            deferredShortLaneCount = orderCandidates(iOrder).deferredShortLaneCount;

            for startDirection = [+1, -1]
                lanePlan = buildLanePlanFromSequence(laneData, seq, startDirection);
                [score, approxFieldPathLength_m] = computeApproxLanePlanScore( ...
                    lanePlan, originXY, psi, anchorXY, postSprayLevelDistance_m, planningOptions);

                if score < bestScore - 1e-9
                    bestScore = score;
                    bestPsi = psi;
                    bestBoundaryLocalLoops = boundaryLocalLoops;
                    bestLanePlan = lanePlan;
                    bestDeferredShortLaneCount = deferredShortLaneCount;
                    bestApproxFieldPathLength_m = approxFieldPathLength_m;
                end
            end
        end
    end

    assert(isfinite(bestScore) && ~isempty(bestLanePlan), 'Field "%s" produced no valid spray lanes.', fieldDef.name);

    firstInterval = bestLanePlan(1).intervalsOrdered(1,:);
    lastInterval = bestLanePlan(end).intervalsOrdered(end,:);
    if bestLanePlan(1).direction > 0
        firstStartX = firstInterval(1);
    else
        firstStartX = firstInterval(2);
    end
    if bestLanePlan(end).direction > 0
        lastEndX = lastInterval(2);
    else
        lastEndX = lastInterval(1);
    end
    lastExitX = lastEndX + bestLanePlan(end).direction * postSprayLevelDistance_m;

    firstCheckpointXYZ = localXYZToGlobalXYZ([firstStartX, bestLanePlan(1).y, flightHeight], originXY, bestPsi);
    lastEndXYZ = localXYZToGlobalXYZ([lastEndX, bestLanePlan(end).y, flightHeight], originXY, bestPsi);
    lastExitXYZ = localXYZToGlobalXYZ([lastExitX, bestLanePlan(end).y, flightHeight], originXY, bestPsi);
    firstHeading = localHeadingToGlobal(bestLanePlan(1).headingLocal, bestPsi);
    lastHeading = localHeadingToGlobal(bestLanePlan(end).headingLocal, bestPsi);

    plan = struct();
    plan.boundaryGlobalXY = boundaryGlobalXY;
    plan.boundaryLocalLoops = bestBoundaryLocalLoops;
    plan.originXY = originXY;
    plan.psi = bestPsi;
    plan.lanePlan = bestLanePlan;
    plan.firstCheckpointXYZ = firstCheckpointXYZ;
    plan.firstHeading = firstHeading;
    plan.lastEndXYZ = lastEndXYZ;
    plan.lastExitXYZ = lastExitXYZ;
    plan.lastHeading = lastHeading;
    plan.deferredShortLaneCount = bestDeferredShortLaneCount;
    plan.approxFieldPathLength_m = bestApproxFieldPathLength_m;
    plan.totalScore_m = bestScore;
    plan.chosenLaneAngle_deg = rad2deg(bestPsi);
end

function boundaryLoops = getFieldBoundaryLoops(fieldDef)
    if isfield(fieldDef, 'memberBoundariesXY') && ~isempty(fieldDef.memberBoundariesXY)
        boundaryLoops = fieldDef.memberBoundariesXY;
    else
        boundaryLoops = {fieldDef.boundaryXY};
    end
end

function psiCandidates = buildCandidateLaneAngleSet(fieldDef, planningOptions)
    if strcmpi(fieldDef.type, 'rectangle') || ~planningOptions.optimizePolygonLaneAngle
        psiCandidates = wrapAnglesToPiPeriod(fieldDef.localFrameYaw_rad);
        return
    end

    gridPsi = deg2rad(0:planningOptions.candidateLaneAngleStep_deg:(180 - planningOptions.candidateLaneAngleStep_deg));
    psiCandidates = wrapAnglesToPiPeriod([fieldDef.localFrameYaw_rad; gridPsi(:)]);
end

function psiWrappedUnique = wrapAnglesToPiPeriod(psi)
    psi = mod(psi(:), pi);
    psiRounded = round(psi * 1e10) / 1e10;
    [~, ia] = unique(psiRounded, 'stable');
    psiWrappedUnique = psi(ia);
end

function laneData = buildLaneDataForOrientation(boundaryLocalLoops, sprayWidth, laneSpacing, planningOptions)
    allLocal = cell2mat(boundaryLocalLoops(:));
    yMin = min(allLocal(:,2));
    yMax = max(allLocal(:,2));

    if (yMax - yMin) <= sprayWidth + 1e-9
        laneCentresCandidate = 0.5 * (yMin + yMax);
    else
        laneCentresCandidate = buildExactSampleVector(yMin + sprayWidth/2, yMax - sprayWidth/2, laneSpacing);
    end

    laneData = struct('y', {}, 'xLeft', {}, 'xRight', {}, 'length', {}, 'pathLength', {}, 'intervals', {}, 'isShort', {}, 'originalIndex', {});
    for i = 1:numel(laneCentresCandidate)
        yLane = laneCentresCandidate(i);
        intervals = collectLaneIntervalsInLoops(boundaryLocalLoops, yLane);
        if isempty(intervals)
            continue
        end

        lengths = intervals(:,2) - intervals(:,1);
        sprayLength = sum(lengths);
        if sprayLength <= max(1e-6, 0.5*sprayWidth)
            continue
        end

        xLeft = intervals(1,1);
        xRight = intervals(end,2);
        laneData(end+1).y = yLane; %#ok<AGROW>
        laneData(end).xLeft = xLeft;
        laneData(end).xRight = xRight;
        laneData(end).length = sprayLength;
        laneData(end).pathLength = xRight - xLeft;
        laneData(end).intervals = intervals;
        laneData(end).isShort = false;
        laneData(end).originalIndex = numel(laneData);
    end

    if isempty(laneData)
        return
    end

    lengths = [laneData.length];
    shortLaneThreshold_m = max(planningOptions.shortLaneAbsoluteThreshold_m, planningOptions.shortLaneLengthRatio * median(lengths));
    shortMask = lengths <= shortLaneThreshold_m + 1e-9;

    if all(shortMask)
        shortMask(:) = false;
    end

    for i = 1:numel(laneData)
        laneData(i).isShort = shortMask(i);
    end
end

function orderCandidates = buildLaneSequenceCandidates(laneData, planningOptions)
    n = numel(laneData);
    orderCandidates = struct('sequence', {}, 'deferredShortLaneCount', {});

    baseSequences = {1:n, n:-1:1};
    for i = 1:numel(baseSequences)
        orderCandidates(end+1).sequence = baseSequences{i}; %#ok<AGROW>
        orderCandidates(end).deferredShortLaneCount = 0;
    end

    if ~planningOptions.deferShortEdgeLanes || n < 4
        return
    end

    lowCount = 0;
    while lowCount < n && laneData(lowCount+1).isShort
        lowCount = lowCount + 1;
    end

    highCount = 0;
    while highCount < (n - lowCount) && laneData(n-highCount).isShort
        highCount = highCount + 1;
    end

    deferredShortLaneCount = lowCount + highCount;
    if deferredShortLaneCount == 0 || (n - deferredShortLaneCount) < 2
        return
    end

    coreIdx = (lowCount+1):(n-highCount);
    lowIdx = 1:lowCount;
    highIdx = (n-highCount+1):n;

    candidateSeqs = {
        [coreIdx, fliplr(lowIdx), highIdx], ...
        [coreIdx, highIdx, fliplr(lowIdx)], ...
        [fliplr(coreIdx), lowIdx, fliplr(highIdx)], ...
        [fliplr(coreIdx), fliplr(highIdx), lowIdx]};

    for i = 1:numel(candidateSeqs)
        seq = candidateSeqs{i};
        if numel(unique(seq)) ~= n
            continue
        end
        if ~hasSequenceAlready(orderCandidates, seq)
            orderCandidates(end+1).sequence = seq; %#ok<AGROW>
            orderCandidates(end).deferredShortLaneCount = deferredShortLaneCount;
        end
    end
end

function tf = hasSequenceAlready(orderCandidates, seq)
    tf = false;
    for i = 1:numel(orderCandidates)
        if isequal(orderCandidates(i).sequence, seq)
            tf = true;
            return
        end
    end
end

function lanePlan = buildLanePlanFromSequence(laneData, seq, startDirection)
    n = numel(seq);
    lanePlan = repmat(struct( ...
        'y', NaN, 'xLeft', NaN, 'xRight', NaN, 'length', NaN, 'pathLength', NaN, 'intervals', zeros(0,2), 'intervalsOrdered', zeros(0,2), 'isShort', false, ...
        'startX', NaN, 'endX', NaN, 'headingLocal', NaN, 'direction', NaN, 'sourceIndex', NaN), n, 1);

    for i = 1:n
        src = laneData(seq(i));
        direction = startDirection * (-1)^(i-1);
        lanePlan(i).y = src.y;
        lanePlan(i).xLeft = src.xLeft;
        lanePlan(i).xRight = src.xRight;
        lanePlan(i).length = src.length;
        lanePlan(i).pathLength = src.pathLength;
        lanePlan(i).intervals = src.intervals;
        lanePlan(i).isShort = src.isShort;
        lanePlan(i).direction = direction;
        lanePlan(i).sourceIndex = src.originalIndex;

        if direction > 0
            lanePlan(i).startX = src.xLeft;
            lanePlan(i).endX = src.xRight;
            lanePlan(i).headingLocal = pi/2;
            lanePlan(i).intervalsOrdered = src.intervals;
        else
            lanePlan(i).startX = src.xRight;
            lanePlan(i).endX = src.xLeft;
            lanePlan(i).headingLocal = 3*pi/2;
            lanePlan(i).intervalsOrdered = flipud(src.intervals);
        end
    end
end

function [score, approxFieldPathLength_m] = computeApproxLanePlanScore( ...
    lanePlan, originXY, psi, anchorXY, postSprayLevelDistance_m, planningOptions)
    score = 0;
    approxFieldPathLength_m = 0;

    firstStartXY = localPointToGlobalXY([lanePlan(1).startX, lanePlan(1).y], originXY, psi);
    entryDistance = norm(firstStartXY - anchorXY(:).');
    score = score + planningOptions.entryDistanceWeight * entryDistance;

    pathLength = sum([lanePlan.pathLength]) + numel(lanePlan) * postSprayLevelDistance_m;
    score = score + pathLength;
    approxFieldPathLength_m = approxFieldPathLength_m + pathLength;

    for i = 1:(numel(lanePlan)-1)
        runOutEndX = lanePlan(i).endX + lanePlan(i).direction * postSprayLevelDistance_m;
        endXY = localPointToGlobalXY([runOutEndX, lanePlan(i).y], originXY, psi);
        nextStartXY = localPointToGlobalXY([lanePlan(i+1).startX, lanePlan(i+1).y], originXY, psi);
        connectionDistance = norm(nextStartXY - endXY);
        score = score + planningOptions.turnConnectionWeight * connectionDistance;
        approxFieldPathLength_m = approxFieldPathLength_m + connectionDistance;
    end

    shortLanesBeforeEnd = 0;
    lastLongIdx = find(~[lanePlan.isShort], 1, 'last');
    if ~isempty(lastLongIdx)
        shortLanesBeforeEnd = sum([lanePlan(1:lastLongIdx).isShort]);
    end
    score = score + planningOptions.shortLanePenalty_m * shortLanesBeforeEnd;
end

function fieldVisitOrder = optimizeFieldVisitOrderGreedy(fields, fieldVisitOrderInitial, groundStationCentreXY, ...
    sprayWidth, laneSpacing, flightHeight, postSprayLevelDistance_m, planningOptions)

    remaining = fieldVisitOrderInitial(:).';
    fieldVisitOrder = zeros(size(remaining));
    anchorXY = groundStationCentreXY;

    for iStep = 1:numel(remaining)
        bestScore = inf;
        bestPos = 1;
        bestEndXY = anchorXY;

        for iCand = 1:numel(remaining)
            fIdx = remaining(iCand);
            plan = planEfficientFieldCoverage(fields(fIdx), anchorXY, sprayWidth, laneSpacing, ...
                flightHeight, postSprayLevelDistance_m, planningOptions);
            score = plan.totalScore_m;

            if score < bestScore - 1e-9
                bestScore = score;
                bestPos = iCand;
                bestEndXY = plan.lastExitXYZ(1:2);
            elseif abs(score - bestScore) <= 1e-9 && planningOptions.useOriginalFieldOrderAsTieBreak
                bestPos = min(bestPos, iCand);
            end
        end

        fieldVisitOrder(iStep) = remaining(bestPos);
        anchorXY = bestEndXY;
        remaining(bestPos) = [];
    end
end

function [fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, ...
    fieldSegPass, fieldSegPowerW, info] = ...
    buildPolygonFieldBasePath(fieldDef, anchorXY, globalPassOffset, ...
    sprayWidth, laneSpacing, flightHeight, turnClimbAltitude_m, ...
    climbAngle_deg, descentAngle_deg, preDescentLevelDistance_m, preSprayLevelDistance_m, postSprayLevelDistance_m, ...
    dsLine, dsTurn, uavAirSpeed, powerW, climbPowerW, planningOptions, uavConn)

    codes = missionCodes();
    plan = planEfficientFieldCoverage(fieldDef, anchorXY, sprayWidth, laneSpacing, ...
        flightHeight, postSprayLevelDistance_m, planningOptions);
    boundaryGlobalXY = plan.boundaryGlobalXY;
    originXY = plan.originXY;
    psi = plan.psi;
    lanePlan = plan.lanePlan;
    displayBoundaryLoops = getFieldBoundaryLoops(fieldDef);

    nLanes = numel(lanePlan);
    lanePassNumbers = zeros(nLanes,1);
    lanePatchLoops = cell(nLanes,1);
    laneMissionStructs = repmat(makeEmptyLaneMissionStruct(), nLanes, 1);

    fieldPts = zeros(0,3);
    fieldRoll = zeros(0,1);
    fieldGamma = zeros(0,1);
    fieldSegTimes = zeros(0,1);
    fieldSegModes = zeros(0,1);
    fieldSegStages = zeros(0,1);
    fieldSegPass = nan(0,1);
    fieldSegPowerW = zeros(0,1);

    for iLane = 1:nLanes
        passNumber = globalPassOffset + iLane;
        lanePassNumbers(iLane) = passNumber;
        lanePatchLoops{iLane} = buildLaneFootprintLoopsForIntervals(originXY, psi, ...
            lanePlan(iLane).intervals, lanePlan(iLane).y, sprayWidth);

        intervalsOrdered = lanePlan(iLane).intervalsOrdered;
        headingLocal = lanePlan(iLane).headingLocal;
        heading = localHeadingToGlobal(headingLocal, psi);

        if lanePlan(iLane).direction > 0
            laneStartLocal = [intervalsOrdered(1,1), lanePlan(iLane).y, flightHeight];
            laneEndLocal   = [intervalsOrdered(end,2), lanePlan(iLane).y, flightHeight];
        else
            laneStartLocal = [intervalsOrdered(1,2), lanePlan(iLane).y, flightHeight];
            laneEndLocal   = [intervalsOrdered(end,1), lanePlan(iLane).y, flightHeight];
        end
        laneStartXYZ = localXYZToGlobalXYZ(laneStartLocal, originXY, psi);
        laneEndXYZ = localXYZToGlobalXYZ(laneEndLocal, originXY, psi);
        laneRunInStartXYZ = offsetPointAlongHeading(laneStartXYZ, heading, -preSprayLevelDistance_m);
        laneRunInStartXYZ(3) = flightHeight;
        laneRunOutEndXYZ = offsetPointAlongHeading(laneEndXYZ, heading, postSprayLevelDistance_m);
        laneRunOutEndXYZ(3) = flightHeight;

        laneMissionStructs(iLane).fieldName = fieldDef.name;
        laneMissionStructs(iLane).localLaneIndex = iLane;
        laneMissionStructs(iLane).passNumber = passNumber;
        laneMissionStructs(iLane).direction = lanePlan(iLane).direction;
        laneMissionStructs(iLane).headingRad = heading;
        laneMissionStructs(iLane).headingDeg = rad2deg(heading);
        laneMissionStructs(iLane).startXYZ = laneStartXYZ;
        laneMissionStructs(iLane).endXYZ = laneEndXYZ;
        laneMissionStructs(iLane).runInStartXYZ = laneRunInStartXYZ;
        laneMissionStructs(iLane).runOutEndXYZ = laneRunOutEndXYZ;
        laneMissionStructs(iLane).sprayLength_m = lanePlan(iLane).length;
        laneMissionStructs(iLane).pathLength_m = lanePlan(iLane).pathLength + postSprayLevelDistance_m;
        laneMissionStructs(iLane).yLocal_m = lanePlan(iLane).y;
        laneMissionStructs(iLane).intervalsLocal = lanePlan(iLane).intervals;
        laneMissionStructs(iLane).intervalsOrderedLocal = lanePlan(iLane).intervalsOrdered;
        laneMissionStructs(iLane).footprintLoopsXY = lanePatchLoops{iLane};
        laneMissionStructs(iLane).preDescentLevelDistance_m = preDescentLevelDistance_m;
        laneMissionStructs(iLane).preSprayLevelDistance_m = preSprayLevelDistance_m;
        laneMissionStructs(iLane).postSprayLevelDistance_m = postSprayLevelDistance_m;
        laneMissionStructs(iLane).hasFollowingLane = false;
        laneMissionStructs(iLane).nextLanePassNumber = NaN;
        laneMissionStructs(iLane).nextLaneFieldIndex = NaN;
        laneMissionStructs(iLane).nextLaneStartXYZ = [NaN NaN NaN];
        laneMissionStructs(iLane).laneSequenceNumber = NaN;

        for iSeg = 1:size(intervalsOrdered,1)
            segInterval = intervalsOrdered(iSeg,:);
            if lanePlan(iLane).direction > 0
                segStartLocal = [segInterval(1), lanePlan(iLane).y, flightHeight];
                segEndLocal   = [segInterval(2), lanePlan(iLane).y, flightHeight];
            else
                segStartLocal = [segInterval(2), lanePlan(iLane).y, flightHeight];
                segEndLocal   = [segInterval(1), lanePlan(iLane).y, flightHeight];
            end

            segStartXYZ = localXYZToGlobalXYZ(segStartLocal, originXY, psi);
            segEndXYZ   = localXYZToGlobalXYZ(segEndLocal, originXY, psi);

            [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk( ...
                segStartXYZ, segEndXYZ, dsLine, uavAirSpeed, ...
                codes.mode.spray, codes.stage.sprayLeg, passNumber, powerW);

            [fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW] = ...
                appendChunkData(fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW, ...
                p,r,g,st,sm,ss,sp,spw);

            if iSeg < size(intervalsOrdered,1)
                nextInterval = intervalsOrdered(iSeg+1,:);
                if lanePlan(iLane).direction > 0
                    gapStartLocal = [segInterval(2), lanePlan(iLane).y, flightHeight];
                    gapEndLocal   = [nextInterval(1), lanePlan(iLane).y, flightHeight];
                else
                    gapStartLocal = [segInterval(1), lanePlan(iLane).y, flightHeight];
                    gapEndLocal   = [nextInterval(2), lanePlan(iLane).y, flightHeight];
                end

                gapStartXYZ = localXYZToGlobalXYZ(gapStartLocal, originXY, psi);
                gapEndXYZ   = localXYZToGlobalXYZ(gapEndLocal, originXY, psi);

                [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk( ...
                    gapStartXYZ, gapEndXYZ, dsLine, uavAirSpeed, ...
                    codes.mode.transfer, codes.stage.transferRejoin, passNumber, powerW);

                [fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW] = ...
                    appendChunkData(fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW, ...
                    p,r,g,st,sm,ss,sp,spw);
            end
        end

        [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk( ...
            laneEndXYZ, laneRunOutEndXYZ, dsLine, uavAirSpeed, ...
            codes.mode.interPassTurn, codes.stage.interPassRunOut, passNumber, powerW);

        [fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW] = ...
            appendChunkData(fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW, ...
            p,r,g,st,sm,ss,sp,spw);

        if iLane < nLanes
            nextHeadingLocal = lanePlan(iLane+1).headingLocal;
            nextHeading = localHeadingToGlobal(nextHeadingLocal, psi);
            nextIntervals = lanePlan(iLane+1).intervalsOrdered;

            if lanePlan(iLane+1).direction > 0
                nextStartLocal = [nextIntervals(1,1), lanePlan(iLane+1).y, flightHeight];
            else
                nextStartLocal = [nextIntervals(1,2), lanePlan(iLane+1).y, flightHeight];
            end

            currentEndXYZ = laneRunOutEndXYZ;
            nextStartXYZ = localXYZToGlobalXYZ(nextStartLocal, originXY, psi);

            [p,r,g,st,sm,ss,sp,spw] = buildInterPassTurnChunk(uavConn, ...
                currentEndXYZ, heading, nextStartXYZ, nextHeading, flightHeight, turnClimbAltitude_m, ...
                climbAngle_deg, descentAngle_deg, preDescentLevelDistance_m, preSprayLevelDistance_m, ...
                dsLine, dsTurn, uavAirSpeed, passNumber, powerW, climbPowerW);

            [fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW] = ...
                appendChunkData(fieldPts, fieldRoll, fieldGamma, fieldSegTimes, fieldSegModes, fieldSegStages, fieldSegPass, fieldSegPowerW, ...
                p,r,g,st,sm,ss,sp,spw);
        end
    end

    info.name = fieldDef.name;
    info.centreXY = mean(boundaryGlobalXY, 1);
    info.boundaryXYZ = [boundaryGlobalXY zeros(size(boundaryGlobalXY,1),1)];
    info.displayBoundaryLoops = displayBoundaryLoops;
    info.firstCheckpointXYZ = plan.firstCheckpointXYZ;
    info.firstHeading = plan.firstHeading;
    info.lastEndXYZ = laneMissionStructs(end).runOutEndXYZ;
    info.lastHeading = plan.lastHeading;
    info.nLanes = nLanes;
    info.laneTurnCount = max(nLanes - 1, 0);
    info.area_ha = fieldDef.area_ha;
    info.sprayAxisText = sprintf('Lane yaw %.1f deg', plan.chosenLaneAngle_deg);
    info.lanePassNumbers = lanePassNumbers;
    info.lanePatchLoops = lanePatchLoops;
    info.chosenLaneAngle_deg = plan.chosenLaneAngle_deg;
    info.deferredShortLaneCount = plan.deferredShortLaneCount;
    info.entryStrategyText = sprintf('Start dir %s', ternary(lanePlan(1).direction > 0, 'left-to-right', 'right-to-left'));
    info.lanePlanScore_m = plan.totalScore_m;
    info.fieldPathLength_m = plan.approxFieldPathLength_m;
    info.laneMissionStructs = laneMissionStructs;
end

function laneLoops = buildLaneFootprintLoopsForIntervals(originXY, psi, intervals, yLane, sprayWidth)
    tolLen = 1e-9;
    tolVertex = 1e-9;
    laneLoops = {};

    if isempty(intervals)
        return
    end

    yLo = yLane - sprayWidth/2;
    yHi = yLane + sprayWidth/2;

    for iInterval = 1:size(intervals,1)
        xLo = intervals(iInterval,1);
        xHi = intervals(iInterval,2);
        if ~isfinite(xLo) || ~isfinite(xHi) || (xHi - xLo) <= tolLen
            continue
        end

        stripLocalXY = [ ...
            xLo, yLo; ...
            xHi, yLo; ...
            xHi, yHi; ...
            xLo, yHi];
        stripGlobalXY = localPointsToGlobal(stripLocalXY, originXY, psi);
        stripGlobalXY = removeDuplicatePolygonVertices(stripGlobalXY, tolVertex);

        if size(stripGlobalXY,1) >= 3 && abs(polyarea(stripGlobalXY(:,1), stripGlobalXY(:,2))) > tolLen
            laneLoops{end+1,1} = stripGlobalXY; %#ok<AGROW>
        end
    end
end

function polygonXY = removeDuplicatePolygonVertices(polygonXY, tol)
    if isempty(polygonXY)
        return
    end

    keep = true(size(polygonXY,1),1);
    for i = 2:size(polygonXY,1)
        if norm(polygonXY(i,:) - polygonXY(i-1,:)) <= tol
            keep(i) = false;
        end
    end
    polygonXY = polygonXY(keep,:);

    if size(polygonXY,1) >= 2 && norm(polygonXY(1,:) - polygonXY(end,:)) <= tol
        polygonXY = polygonXY(1:end-1,:);
    end
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildInterPassTurnChunk(uavConn, currentPassEndXYZ, currentHeading, nextPassStartXYZ, nextHeading, ...
    flightHeight, turnClimbAltitude_m, climbAngle_deg, descentAngle_deg, ...
    preDescentLevelDistance_m, preSprayLevelDistance_m, dsLine, dsTurn, uavAirSpeed, passNumber, powerW, climbPowerW)

    codes = missionCodes();
    altDelta = max(turnClimbAltitude_m - flightHeight, 0);
    climbDistance = ternary(altDelta > 0, altDelta / tan(deg2rad(climbAngle_deg)), 0);
    descentDistance = ternary(altDelta > 0, altDelta / tan(deg2rad(descentAngle_deg)), 0);

    climbEndXYZ = offsetPointAlongHeading(currentPassEndXYZ, currentHeading, climbDistance);
    climbEndXYZ(3) = turnClimbAltitude_m;

    runInStartXYZ = offsetPointAlongHeading(nextPassStartXYZ, nextHeading, -preSprayLevelDistance_m);
    runInStartXYZ(3) = flightHeight;

    descentStartXYZ = offsetPointAlongHeading(runInStartXYZ, nextHeading, -descentDistance);
    descentStartXYZ(3) = turnClimbAltitude_m;

    preDescentStartXYZ = offsetPointAlongHeading(descentStartXYZ, nextHeading, -preDescentLevelDistance_m);
    preDescentStartXYZ(3) = turnClimbAltitude_m;

    pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
    segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
    segPass = nan(0,1); segPowerW = zeros(0,1);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(currentPassEndXYZ, climbEndXYZ, dsLine, uavAirSpeed, ...
        codes.mode.interPassTurn, codes.stage.interPassClimbOut, passNumber, climbPowerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildDubinsChunk(uavConn, climbEndXYZ, currentHeading, preDescentStartXYZ, nextHeading, dsTurn, ...
        codes.mode.interPassTurn, codes.stage.interPassElevatedTurn, uavAirSpeed, passNumber, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(preDescentStartXYZ, descentStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.interPassTurn, codes.stage.interPassElevatedTurn, passNumber, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(descentStartXYZ, runInStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.interPassTurn, codes.stage.interPassDescent, passNumber, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(runInStartXYZ, nextPassStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.interPassTurn, codes.stage.interPassRunIn, passNumber, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildInterFieldTransferChunk(uavConn, currentFieldEndXYZ, currentHeading, nextFieldStartXYZ, nextFieldHeading, ...
    transferAltitude_m, climbAngle_deg, descentAngle_deg, preDescentLevelDistance_m, preSprayLevelDistance_m, ...
    dsLine, dsTurn, uavAirSpeed, powerW)

    codes = missionCodes();
    flightHeight = currentFieldEndXYZ(3);
    altDelta = max(transferAltitude_m - flightHeight, 0);
    climbDistance = ternary(altDelta > 0, altDelta / tan(deg2rad(climbAngle_deg)), 0);
    descentDistance = ternary(altDelta > 0, altDelta / tan(deg2rad(descentAngle_deg)), 0);

    climbEndXYZ = offsetPointAlongHeading(currentFieldEndXYZ, currentHeading, climbDistance);
    climbEndXYZ(3) = transferAltitude_m;

    runInStartXYZ = offsetPointAlongHeading(nextFieldStartXYZ, nextFieldHeading, -preSprayLevelDistance_m);
    runInStartXYZ(3) = nextFieldStartXYZ(3);

    descentStartXYZ = offsetPointAlongHeading(runInStartXYZ, nextFieldHeading, -descentDistance);
    descentStartXYZ(3) = transferAltitude_m;

    preDescentStartXYZ = offsetPointAlongHeading(descentStartXYZ, nextFieldHeading, -preDescentLevelDistance_m);
    preDescentStartXYZ(3) = transferAltitude_m;

    pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
    segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
    segPass = nan(0,1); segPowerW = zeros(0,1);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(currentFieldEndXYZ, climbEndXYZ, dsLine, uavAirSpeed, ...
        codes.mode.transfer, codes.stage.transferCruise, NaN, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
        appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildDubinsChunk(uavConn, ...
        climbEndXYZ, currentHeading, preDescentStartXYZ, nextFieldHeading, ...
        dsTurn, codes.mode.transfer, codes.stage.transferCruise, uavAirSpeed, NaN, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
        appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk( ...
        preDescentStartXYZ, descentStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.transfer, codes.stage.transferCruise, NaN, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
        appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk( ...
        descentStartXYZ, runInStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.transfer, codes.stage.transferRejoin, NaN, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
        appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk( ...
        runInStartXYZ, nextFieldStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.transfer, codes.stage.transferRejoin, NaN, powerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
        appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);
end

function [passStartBaseTime, passEndBaseTime] = computePassTimeBounds(baseSegmentStages, baseSegmentPass, basePathTime, totalLaneCount)
    codes = missionCodes();
    passStartBaseTime = nan(totalLaneCount,1);
    passEndBaseTime = nan(totalLaneCount,1);
    for passNum = 1:totalLaneCount
        idxPass = find(baseSegmentStages == codes.stage.sprayLeg & baseSegmentPass == passNum);
        if ~isempty(idxPass)
            passStartBaseTime(passNum) = basePathTime(idxPass(1));
            passEndBaseTime(passNum) = basePathTime(idxPass(end)+1);
        end
    end
end

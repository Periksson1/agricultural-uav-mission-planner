function simulationData = runMissionSimulation(missionCfg, planningData)
    validateMissionCfg(missionCfg);
    validatePlanningData(planningData);
    codes = missionCodes();

    payloadLitres = missionCfg.payloadLitres;
    initialWeightKg = missionCfg.initialWeightKg;
    applicationRate = missionCfg.applicationRate;
    uavAirSpeed = missionCfg.uavAirSpeed;
    interPassDescentAngle_deg = missionCfg.interPassDescentAngle_deg;
    preDescentLevelDistance_m = missionCfg.preDescentLevelDistance_m;
    preSprayLevelDistance_m = missionCfg.preSprayLevelDistance_m;
    interruptReturnRunOutDistance_m = getOptionalMissionScalar(missionCfg, 'interruptReturnRunOutDistance_m', 25.0);
    returnPullUpAltitude_m = missionCfg.returnPullUpAltitude_m;
    returnPullUpClimbAngle_deg = missionCfg.returnPullUpClimbAngle_deg;
    vtolClimbRate_mps = missionCfg.vtolClimbRate_mps;
    vtolHoverTime_s = missionCfg.vtolHoverTime_s;
    vtolHoverPowerDraw_kW = missionCfg.vtolHoverPowerDraw_kW;
    vtolClimbPowerDraw_kW = missionCfg.vtolClimbPowerDraw_kW;
    batteryGuardCheckDt_s = missionCfg.batteryGuardCheckDt_s;
    turnaroundTime_s = missionCfg.turnaroundTime_s;
    payloadTurnaroundTime_s = getOptionalMissionScalar(missionCfg, 'payloadTurnaroundTime_s', turnaroundTime_s);
    batteryTurnaroundTime_s = getOptionalMissionScalar(missionCfg, 'batteryTurnaroundTime_s', turnaroundTime_s);
    dsLine = missionCfg.dsLine;
    dsTurn = missionCfg.dsTurn;
    rollCmdDt = missionCfg.rollCmdDt;
    rollCmdInterpMethod = missionCfg.rollCmdInterpMethod;
    revenuePerHa_GBP = missionCfg.revenuePerHa_GBP;
    transportCost_GBP = missionCfg.transportCost_GBP;
    workerRatePerHour_GBP = missionCfg.workerRatePerHour_GBP;
    numberOfWorkers = missionCfg.numberOfWorkers;
    batteryChargeCostPerReturn_GBP = missionCfg.batteryChargeCostPerReturn_GBP;
    maintenanceRatePerFlightHour_GBP = missionCfg.maintenanceRatePerFlightHour_GBP;
    displayDsSpray = missionCfg.displayDsSpray;
    displayDsLaneTurn = missionCfg.displayDsLaneTurn;
    displayDsTransit = missionCfg.displayDsTransit;
    compressTurnaroundInPlayback = missionCfg.compressTurnaroundInPlayback;
    turnaroundPlaybackTime_s = missionCfg.turnaroundPlaybackTime_s;
    playbackFPS = missionCfg.playbackFPS;
    targetPlaybackDurationFull_s = missionCfg.targetPlaybackDurationFull_s;
    targetPlaybackDurationSortie_s = missionCfg.targetPlaybackDurationSortie_s;

    uavConn = planningData.uavConn;
    basePathTime = planningData.basePathTime;
    basePathXYZ = planningData.basePathXYZ;
    baseSegmentModes = planningData.baseSegmentModes;
    baseSegmentStages = planningData.baseSegmentStages;
    baseSegmentPass = planningData.baseSegmentPass;
    baseSegmentPowerW = planningData.baseSegmentPowerW;
    totalLaneCount = planningData.totalLaneCount;
    totalFieldArea_ha = planningData.totalFieldArea_ha;
    laneTurnCount = planningData.laneTurnCount;
    fieldCount = planningData.fieldCount;
    fieldVisitOrder = planningData.fieldVisitOrder;
    fieldInfo = planningData.fieldInfo;
    fieldSummaryLines = planningData.fieldSummaryLines;
    fieldFirstCheckpoints = planningData.fieldFirstCheckpoints;
    litresPerSprayMetre = planningData.litresPerSprayMetre;
    maxSprayDistancePerSortie = planningData.maxSprayDistancePerSortie;
    batteryEnergy_Wh = planningData.batteryEnergy_Wh;
    cruisePowerW = planningData.cruisePowerW;
    reserveBatteryWh = planningData.reserveBatteryWh;
    groundStationGroundXYZ = planningData.groundStationGroundXYZ;
    groundStationHighXYZ = planningData.groundStationHighXYZ;
    fieldTransferAltitudeAbs_m = groundStationGroundXYZ(3) + planningData.fieldTransferAltitude_m;
    returnCruiseAltitude_m = groundStationHighXYZ(3);
    returnPullUpAltitudeAbs_m = groundStationGroundXYZ(3) + returnPullUpAltitude_m;
    terrainModel = getOptionalTerrainModelForSimulation(planningData);
    passStartBaseTime = planningData.passStartBaseTime;
    passEndBaseTime = planningData.passEndBaseTime;
    plannedSprayArea_ha = planningData.plannedSprayArea_ha;
    FsSpray = planningData.FsSpray;
    FBaseEnergyWh = planningData.FBaseEnergyWh;
    FbX = planningData.FbX;
    FbY = planningData.FbY;
    FbZ = planningData.FbZ;
    FbRoll = planningData.FbRoll;
    FbGamma = planningData.FbGamma;
    groundStationToFirstCheckpointDistance = planningData.groundStationToFirstCheckpointDistance;

    vtolHoverPowerW = vtolHoverPowerDraw_kW * 1000;
    vtolClimbPowerW = vtolClimbPowerDraw_kW * 1000;

    %% -------------------- MULTI-SORTIE MISSION --------------------
    initialSortieCapacity = max(8, min(max(totalLaneCount + 1, 1), 64));
    initialChunkCapacity = max(32, 4 * initialSortieCapacity);

    missionChunks = makeEmptyMissionChunkStore(initialChunkCapacity);
    chunkCount = 0;

    payloadInterruptPoints = nan(initialSortieCapacity,3);
    batteryInterruptPoints = nan(initialSortieCapacity,3);
    payloadInterruptSorties = nan(initialSortieCapacity,1);
    batteryInterruptSorties = nan(initialSortieCapacity,1);
    payloadInterruptCount = 0;
    batteryInterruptCount = 0;

    sortieTemplate = struct( ...
        'sortieNumber',NaN, 'startMissionT',NaN, 'resumeTrigger','', 'interruptReason','', ...
        'interruptT',NaN, 'arrivalGST',NaN, 'readyAgainT',NaN, ...
        'completeT',NaN, 'resumeBaseTime',NaN, 'stopBaseTime',NaN, 'nextResumeBaseTime',NaN);
    sortieLog = repmat(sortieTemplate, initialSortieCapacity, 1);

    resumeBaseTime = 0;
    missionTimeCursor = 0;
    sortieNum = 0;
    tol = 1e-9;

    while resumeBaseTime < basePathTime(end) - tol
        sortieNum = sortieNum + 1;
        sortieLog = ensureSortieLogCapacity(sortieLog, sortieNum, sortieTemplate);
        sortieStartMissionTime = missionTimeCursor;

        [resumePos, ~, ~, resumeHeading, ~, ~] = getPathStateAtTime( ...
            resumeBaseTime, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, baseSegmentModes, baseSegmentPass);

        if sortieNum == 1
            resumeTrigger = 'initial';
        else
            resumeTrigger = sortieLog(sortieNum-1).interruptReason;
        end

        resumeRunInStartXYZ = offsetPointAlongHeading(resumePos, resumeHeading, -preSprayLevelDistance_m);
        resumeRunInStartXYZ(3) = sampleTerrainElevationForSimulation(terrainModel, resumeRunInStartXYZ(1:2)) + missionCfg.flightHeight;

        [chunkPts, chunkRoll, chunkGamma, chunkSegTimes, chunkSegModes, ...
            chunkSegStages, chunkSegPass, chunkSegPowerW] = ...
            buildOutboundChunk(uavConn, groundStationGroundXYZ, groundStationHighXYZ, ...
            resumeRunInStartXYZ, resumePos, resumeHeading, fieldTransferAltitudeAbs_m, ...
            interPassDescentAngle_deg, preDescentLevelDistance_m, vtolClimbRate_mps, vtolHoverTime_s, ...
            uavAirSpeed, dsLine, dsTurn, cruisePowerW, vtolHoverPowerW, vtolClimbPowerW);

        outboundTime = sum(chunkSegTimes);
        outboundEnergyWh = sum(chunkSegPowerW .* chunkSegTimes / 3600);

        [missionChunks, chunkCount] = storeChunkCells(missionChunks, chunkCount, ...
            chunkPts, chunkRoll, chunkGamma, chunkSegTimes, chunkSegModes, ...
            chunkSegStages, chunkSegPass, chunkSegPowerW, sortieNum);

        missionTimeCursor = missionTimeCursor + outboundTime;

        batteryAtResumeWh = batteryEnergy_Wh - outboundEnergyWh;
        requiredReturnEnergyAtResumeWh = computeSimpleRequiredReturnEnergy( ...
            uavConn, resumePos, resumeHeading, groundStationGroundXYZ, ...
            interruptReturnRunOutDistance_m, returnPullUpAltitudeAbs_m, returnPullUpClimbAngle_deg, returnCruiseAltitude_m, ...
            vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
            cruisePowerW, vtolHoverPowerW, vtolClimbPowerW);

        if batteryAtResumeWh <= reserveBatteryWh + requiredReturnEnergyAtResumeWh - tol
            error(['After reaching the resume point, the UAV would no longer be able to return to the ground station, ' ...
                   'land, and still retain the required battery reserve. Increase battery capacity, reduce reserve, ' ...
                   'move the ground station closer, or reduce power draw.']);
        end

        [stopBaseTime, stopReason, nextResumeBaseTime] = advanceAlongBaseUntilTrigger( ...
            resumeBaseTime, basePathTime, FsSpray, maxSprayDistancePerSortie, ...
            batteryAtResumeWh, reserveBatteryWh, cruisePowerW, batteryGuardCheckDt_s, ...
            uavConn, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
            baseSegmentModes, baseSegmentStages, baseSegmentPass, FBaseEnergyWh, ...
            passStartBaseTime, passEndBaseTime, ...
            groundStationGroundXYZ, interruptReturnRunOutDistance_m, returnPullUpAltitudeAbs_m, returnPullUpClimbAngle_deg, ...
            returnCruiseAltitude_m, vtolClimbRate_mps, uavAirSpeed, dsLine, dsTurn, ...
            vtolHoverTime_s, vtolHoverPowerW, vtolClimbPowerW);

        [slicePts, sliceRoll, sliceGamma, sliceSegTimes, sliceSegModes, ...
            sliceSegStages, sliceSegPass, sliceSegPowerW] = ...
            extractPathSlice(resumeBaseTime, stopBaseTime, ...
            basePathTime, FbX, FbY, FbZ, FbRoll, FbGamma, ...
            baseSegmentModes, baseSegmentStages, baseSegmentPass, baseSegmentPowerW);

        [missionChunks, chunkCount] = storeChunkCells(missionChunks, chunkCount, ...
            slicePts, sliceRoll, sliceGamma, sliceSegTimes, sliceSegModes, ...
            sliceSegStages, sliceSegPass, sliceSegPowerW, sortieNum);

        missionTimeCursor = missionTimeCursor + sum(sliceSegTimes);

        stateQueryTime = stopBaseTime;
        if strcmp(stopReason, 'battery') && nextResumeBaseTime > stopBaseTime + tol
            stateQueryTime = max(basePathTime(1), stopBaseTime - 1e-6);
        end

        [stopPos, ~, ~, stopHeading, ~, ~] = getPathStateAtTime( ...
            stateQueryTime, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, baseSegmentModes, baseSegmentPass);
        if stateQueryTime < stopBaseTime - 1e-12
            stopPos = [FbX(stopBaseTime), FbY(stopBaseTime), FbZ(stopBaseTime)];
        end

        if strcmp(stopReason, 'complete')
            [rtbPts, rtbRoll, rtbGamma, rtbSegTimes, rtbSegModes, ...
                rtbSegStages, rtbSegPass, rtbSegPowerW] = ...
                buildReturnChunk(uavConn, stopPos, stopHeading, groundStationGroundXYZ, ...
                0, returnPullUpAltitudeAbs_m, returnPullUpClimbAngle_deg, returnCruiseAltitude_m, ...
                vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
                cruisePowerW, vtolHoverPowerW, vtolClimbPowerW);

            [missionChunks, chunkCount] = storeChunkCells(missionChunks, chunkCount, ...
                rtbPts, rtbRoll, rtbGamma, rtbSegTimes, rtbSegModes, ...
                rtbSegStages, rtbSegPass, rtbSegPowerW, sortieNum);

            missionTimeCursor = missionTimeCursor + sum(rtbSegTimes);
            arrivalGSTime = missionTimeCursor;

            sortieLog(sortieNum).sortieNumber    = sortieNum;
            sortieLog(sortieNum).startMissionT   = sortieStartMissionTime;
            sortieLog(sortieNum).resumeTrigger   = resumeTrigger;
            sortieLog(sortieNum).interruptReason = 'complete';
            sortieLog(sortieNum).arrivalGST      = arrivalGSTime;
            sortieLog(sortieNum).completeT       = missionTimeCursor;
            sortieLog(sortieNum).resumeBaseTime  = resumeBaseTime;
            sortieLog(sortieNum).stopBaseTime    = stopBaseTime;
            sortieLog(sortieNum).nextResumeBaseTime = NaN;
            break
        end

        interruptMissionTime = missionTimeCursor;

        if strcmp(stopReason, 'payload')
            payloadInterruptCount = payloadInterruptCount + 1;
            payloadInterruptPoints = ensurePointCapacity(payloadInterruptPoints, payloadInterruptCount);
            payloadInterruptSorties = ensurePointCapacity(payloadInterruptSorties, payloadInterruptCount);
            payloadInterruptPoints(payloadInterruptCount,:) = stopPos;
            payloadInterruptSorties(payloadInterruptCount) = sortieNum;
        elseif strcmp(stopReason, 'battery')
            batteryInterruptCount = batteryInterruptCount + 1;
            batteryInterruptPoints = ensurePointCapacity(batteryInterruptPoints, batteryInterruptCount);
            batteryInterruptSorties = ensurePointCapacity(batteryInterruptSorties, batteryInterruptCount);
            batteryInterruptPoints(batteryInterruptCount,:) = stopPos;
            batteryInterruptSorties(batteryInterruptCount) = sortieNum;
        end

        [rtbPts, rtbRoll, rtbGamma, rtbSegTimes, rtbSegModes, ...
            rtbSegStages, rtbSegPass, rtbSegPowerW] = ...
            buildReturnChunk(uavConn, stopPos, stopHeading, groundStationGroundXYZ, ...
            interruptReturnRunOutDistance_m, returnPullUpAltitudeAbs_m, returnPullUpClimbAngle_deg, returnCruiseAltitude_m, ...
            vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
            cruisePowerW, vtolHoverPowerW, vtolClimbPowerW);

        [missionChunks, chunkCount] = storeChunkCells(missionChunks, chunkCount, ...
            rtbPts, rtbRoll, rtbGamma, rtbSegTimes, rtbSegModes, ...
            rtbSegStages, rtbSegPass, rtbSegPowerW, sortieNum);

        missionTimeCursor = missionTimeCursor + sum(rtbSegTimes);
        arrivalGSTime = missionTimeCursor;

        selectedTurnaroundTime_s = selectTurnaroundTime(stopReason, ...
            turnaroundTime_s, payloadTurnaroundTime_s, batteryTurnaroundTime_s);

        [waitPts, waitRoll, waitGamma, waitSegTimes, waitSegModes, ...
            waitSegStages, waitSegPass, waitSegPowerW] = ...
            buildPauseChunk(groundStationGroundXYZ, selectedTurnaroundTime_s, ...
            codes.mode.turnaround, codes.stage.turnaroundPause, 0, NaN);

        [missionChunks, chunkCount] = storeChunkCells(missionChunks, chunkCount, ...
            waitPts, waitRoll, waitGamma, waitSegTimes, waitSegModes, ...
            waitSegStages, waitSegPass, waitSegPowerW, sortieNum);

        missionTimeCursor = missionTimeCursor + sum(waitSegTimes);
        readyAgainTime = missionTimeCursor;

        sortieLog(sortieNum).sortieNumber    = sortieNum;
        sortieLog(sortieNum).startMissionT   = sortieStartMissionTime;
        sortieLog(sortieNum).resumeTrigger   = resumeTrigger;
        sortieLog(sortieNum).interruptReason = stopReason;
        sortieLog(sortieNum).interruptT      = interruptMissionTime;
        sortieLog(sortieNum).arrivalGST      = arrivalGSTime;
        sortieLog(sortieNum).readyAgainT     = readyAgainTime;
        sortieLog(sortieNum).resumeBaseTime  = resumeBaseTime;
        sortieLog(sortieNum).stopBaseTime    = stopBaseTime;
        sortieLog(sortieNum).nextResumeBaseTime = nextResumeBaseTime;

        resumeBaseTime = nextResumeBaseTime;
    end

    nSorties = sortieNum;
    sortieLog = sortieLog(1:nSorties);
    payloadInterruptPoints = payloadInterruptPoints(1:payloadInterruptCount,:);
    batteryInterruptPoints = batteryInterruptPoints(1:batteryInterruptCount,:);
    payloadInterruptSorties = payloadInterruptSorties(1:payloadInterruptCount,:);
    batteryInterruptSorties = batteryInterruptSorties(1:batteryInterruptCount,:);

    %% -------------------- FINAL MISSION ARRAYS --------------------
    [pathXYZ, pathRollAngle, pathFlightPathAngle, ...
        segmentTimes, segmentModes, segmentStages, segmentPass, ...
        segmentSortie, segmentPowerW] = ...
        finalizeMissionChunks(missionChunks, chunkCount);

    pathTime = [0; cumsum(segmentTimes)];
    nSeg = numel(segmentTimes);
    segmentLengths = vecnorm(diff(pathXYZ,1,1),2,2);

    %% -------------------- MISSION METRICS --------------------
    maskSpray       = segmentStages == codes.stage.sprayLeg;
    maskInterPass   = ismember(segmentStages, codes.stageGroups.interPass);
    maskVTOL        = ismember(segmentStages, codes.stageGroups.vtol);
    maskTransfer    = ismember(segmentStages, codes.stageGroups.transfer);
    maskTurnaround  = segmentStages == codes.stage.turnaroundPause;

    sprayDistance       = sum(segmentLengths(maskSpray));
    laneTurnDistance    = sum(segmentLengths(maskInterPass));
    vtolDistance        = sum(segmentLengths(maskVTOL));
    transferDistance    = sum(segmentLengths(maskTransfer));
    totalDistance       = sum(segmentLengths);

    sprayTime           = sum(segmentTimes(maskSpray));
    laneTurnTime        = sum(segmentTimes(maskInterPass));
    vtolTimeTotal       = sum(segmentTimes(maskVTOL));
    transferTimeTotal   = sum(segmentTimes(maskTransfer));
    turnaroundTimeTotal = sum(segmentTimes(maskTurnaround));
    flightTimeTotal     = sum(segmentTimes(~maskTurnaround));
    totalTime           = pathTime(end);

    segmentEnergy_Wh = segmentPowerW .* segmentTimes / 3600;
    sprayEnergy_Wh      = sum(segmentEnergy_Wh(maskSpray));
    laneTurnEnergy_Wh   = sum(segmentEnergy_Wh(maskInterPass));
    vtolEnergy_Wh       = sum(segmentEnergy_Wh(maskVTOL));
    transferEnergy_Wh   = sum(segmentEnergy_Wh(maskTransfer));
    missionEnergyDemand_Wh  = sum(segmentEnergy_Wh);
    missionEnergyDemand_kWh = missionEnergyDemand_Wh / 1000;

    %% -------------------- ONBOARD PAYLOAD / BATTERY STATES --------------------
    payloadRemaining_L  = zeros(size(pathTime));
    batteryRemaining_Wh = zeros(size(pathTime));
    batteryPct          = zeros(size(pathTime));
    totalLitresDispensed = zeros(size(pathTime));
    sprayedArea_ha      = zeros(size(pathTime));
    droneWeightKg       = zeros(size(pathTime));

    payloadRemaining_L(1)  = payloadLitres;
    batteryRemaining_Wh(1) = batteryEnergy_Wh;
    batteryPct(1)          = 100;
    droneWeightKg(1)       = initialWeightKg;

    for i = 1:nSeg
        dt_i    = segmentTimes(i);
        dist_i  = segmentLengths(i);
        mode_i  = segmentModes(i);
        power_i = segmentPowerW(i);

        payloadEnd   = payloadRemaining_L(i);
        dispensedEnd = totalLitresDispensed(i);

        switch mode_i
            case codes.mode.spray
                litresThisSeg = dist_i * litresPerSprayMetre;
                payloadEnd = max(payloadRemaining_L(i) - litresThisSeg, 0);
                dispensedEnd = totalLitresDispensed(i) + litresThisSeg;
                batteryEnd = max(batteryRemaining_Wh(i) - power_i * dt_i / 3600, 0);
            case codes.mode.turnaround
                payloadEnd = payloadLitres;
                batteryEnd = batteryEnergy_Wh;
            otherwise
                batteryEnd = max(batteryRemaining_Wh(i) - power_i * dt_i / 3600, 0);
        end

        payloadRemaining_L(i+1)  = payloadEnd;
        batteryRemaining_Wh(i+1) = batteryEnd;
        batteryPct(i+1)          = 100 * batteryEnd / max(batteryEnergy_Wh, eps);
        totalLitresDispensed(i+1)= dispensedEnd;
        sprayedArea_ha(i+1)      = dispensedEnd / applicationRate;
        droneWeightKg(i+1)       = initialWeightKg - (payloadLitres - payloadEnd);
    end

    finalBatteryPct = batteryPct(end);
    finalWeightKg = droneWeightKg(end);
    totalPayloadUsed_L = totalLitresDispensed(end);
    areaNotSprayed_ha = max(totalFieldArea_ha - sprayedArea_ha(end), 0);

    passCompletionMissionTime = nan(totalLaneCount,1);
    spraySegIdx = find(maskSpray);
    if ~isempty(spraySegIdx)
        sprayPassNums = segmentPass(spraySegIdx);
        validPassMask = isfinite(sprayPassNums) & sprayPassNums >= 1 & sprayPassNums <= totalLaneCount;
        spraySegIdx = spraySegIdx(validPassMask);
        sprayPassNums = sprayPassNums(validPassMask);

        if ~isempty(spraySegIdx)
            [~, lastPerPassIdx] = unique(sprayPassNums, 'last');
            passCompletionMissionTime(sprayPassNums(lastPerPassIdx)) = pathTime(spraySegIdx(lastPerPassIdx) + 1);
        end
    end

    if sprayTime > 0
        sprayRate_ha_per_hr = sprayedArea_ha(end) / (sprayTime / 3600);
    else
        sprayRate_ha_per_hr = 0;
    end

    if totalTime > 0
        overallFieldCapacity_ha_per_hr = sprayedArea_ha(end) / (totalTime / 3600);
        sprayTimeFraction_pct = 100 * sprayTime / totalTime;
        nonProductiveTime_min = (totalTime - sprayTime) / 60;
        nonProductiveTime_pct = 100 * (totalTime - sprayTime) / totalTime;
        turnaroundShare_pct = 100 * turnaroundTimeTotal / totalTime;
        transitShare_pct = 100 * (transferTimeTotal + vtolTimeTotal) / totalTime;
        turnShare_pct = 100 * laneTurnTime / totalTime;
    else
        overallFieldCapacity_ha_per_hr = 0;
        sprayTimeFraction_pct = 0;
        nonProductiveTime_min = 0;
        nonProductiveTime_pct = 0;
        turnaroundShare_pct = 0;
        transitShare_pct = 0;
        turnShare_pct = 0;
    end

    if nSorties > 0
        averageAreaPerSortie_ha = sprayedArea_ha(end) / nSorties;
    else
        averageAreaPerSortie_ha = 0;
    end

    payloadTriggerCount = sum(strcmp({sortieLog.interruptReason}, 'payload'));
    batteryTriggerCount = sum(strcmp({sortieLog.interruptReason}, 'battery'));
    completeSortieCount = sum(strcmp({sortieLog.interruptReason}, 'complete'));
    returnCount = payloadTriggerCount + batteryTriggerCount + completeSortieCount;

    if sprayedArea_ha(end) > 0
        energyPerHa_kWh = missionEnergyDemand_kWh / sprayedArea_ha(end);
        turnsPerHa = laneTurnCount / sprayedArea_ha(end);
    else
        energyPerHa_kWh = NaN;
        turnsPerHa = NaN;
    end

    %% -------------------- ECONOMICS --------------------
    sprayLabourBillableHours = ceil((sprayTime / 3600) * 2) / 2;
    missionLabourBillableHours = ceil((totalTime / 3600) * 2) / 2;
    labourCost_GBP = workerRatePerHour_GBP * numberOfWorkers * missionLabourBillableHours;
    batteryChargeCount = returnCount;
    batteryChargingCost_GBP = batteryChargeCostPerReturn_GBP * batteryChargeCount;
    maintenanceFlightHours = flightTimeTotal / 3600;
    maintenanceCost_GBP = maintenanceRatePerFlightHour_GBP * maintenanceFlightHours;

    totalRevenue_GBP = revenuePerHa_GBP * sprayedArea_ha(end);
    totalCost_GBP = transportCost_GBP + labourCost_GBP + batteryChargingCost_GBP + maintenanceCost_GBP;
    netProfit_GBP = totalRevenue_GBP - totalCost_GBP;

    %% -------------------- BATTERY RTB STATS --------------------
    rtbTriggerMask = ~strcmp({sortieLog.interruptReason}, 'complete');
    rtbTriggerTimes = [sortieLog(rtbTriggerMask).interruptT]';
    rtbArrivalTimes = [sortieLog(rtbTriggerMask).arrivalGST]';
    rtbTriggerTypes = {sortieLog(rtbTriggerMask).interruptReason}';

    if ~isempty(rtbTriggerTimes)
        batteryPctAtRTBTrigger = interp1SafeNonUnique(pathTime, batteryPct, rtbTriggerTimes, 'linear');
        payloadAtRTBTrigger_L = interp1SafeNonUnique(pathTime, payloadRemaining_L, rtbTriggerTimes, 'linear');
        avgBatteryPctAtRTBTrigger = mean(batteryPctAtRTBTrigger, 'omitnan');
        minBatteryPctAtRTBTrigger = min(batteryPctAtRTBTrigger);
    else
        batteryPctAtRTBTrigger = [];
        payloadAtRTBTrigger_L = [];
        rtbTriggerTypes = cell(0,1);
        avgBatteryPctAtRTBTrigger = NaN;
        minBatteryPctAtRTBTrigger = NaN;
    end

    if ~isempty(rtbArrivalTimes)
        batteryPctAtGSArrival = interp1SafeNonUnique(pathTime, batteryPct, rtbArrivalTimes, 'linear');
        avgBatteryPctAtGSArrival = mean(batteryPctAtGSArrival, 'omitnan');
    else
        batteryPctAtGSArrival = [];
        avgBatteryPctAtGSArrival = NaN;
    end

    pathHeadingAngle = computeHeadingFromPath(pathXYZ);
    pathHeadingAngleUnwrapped = unwrap(pathHeadingAngle);

    %% -------------------- COMMAND EXPORT --------------------
    sortieCommandExports = repmat(makeEmptySortieCommandExport(), nSorties, 1);
    for iSortie = 1:nSorties
        sortieCommandExports(iSortie) = buildCruiseOnlySortieCommandExport( ...
            iSortie, pathXYZ, pathRollAngle, pathFlightPathAngle, pathTime, ...
            segmentStages, segmentSortie, rollCmdDt, rollCmdInterpMethod);
    end

    rollCmdTimeBySortie = {sortieCommandExports.rollCmdTime}.';
    rollCmdRadBySortie = {sortieCommandExports.rollCmdRad}.';
    rollCmdDegBySortie = {sortieCommandExports.rollCmdDeg}.';
    rollCmdMatrixBySortie = {sortieCommandExports.rollCmdMatrix}.';
    rollCmdTSBySortie = {sortieCommandExports.rollCmdTS}.';

    flightPathCmdTimeBySortie = {sortieCommandExports.flightPathCmdTime}.';
    flightPathCmdRadBySortie = {sortieCommandExports.flightPathCmdRad}.';
    flightPathCmdDegBySortie = {sortieCommandExports.flightPathCmdDeg}.';
    flightPathCmdMatrixBySortie = {sortieCommandExports.flightPathCmdMatrix}.';
    flightPathCmdTSBySortie = {sortieCommandExports.flightPathCmdTS}.';

    headingCmdTimeBySortie = {sortieCommandExports.headingCmdTime}.';
    headingCmdRadBySortie = {sortieCommandExports.headingCmdRad}.';
    headingCmdDegBySortie = {sortieCommandExports.headingCmdDeg}.';
    headingCmdMatrixBySortie = {sortieCommandExports.headingCmdMatrix}.';
    headingCmdTSBySortie = {sortieCommandExports.headingCmdTS}.';

    altitudeCmdTimeBySortie = {sortieCommandExports.altitudeCmdTime}.';
    altitudeCmdMBySortie = {sortieCommandExports.altitudeCmdM}.';
    altitudeCmdMatrixBySortie = {sortieCommandExports.altitudeCmdMatrix}.';
    altitudeCmdTSBySortie = {sortieCommandExports.altitudeCmdTS}.';

    upCmdTimeBySortie = altitudeCmdTimeBySortie;
    upCmdMBySortie = altitudeCmdMBySortie;
    upCmdMatrixBySortie = altitudeCmdMatrixBySortie;
    upCmdTSBySortie = altitudeCmdTSBySortie;

    initialHeadingAfterVTOLRadBySortie = [sortieCommandExports.initialHeadingAfterVTOLRad].';
    initialHeadingAfterVTOLDegBySortie = [sortieCommandExports.initialHeadingAfterVTOLDeg].';
    initialHeightAfterVTOL_mBySortie = [sortieCommandExports.initialHeightAfterVTOL_m].';
    initialEastAfterVTOL_mBySortie = [sortieCommandExports.initialEastAfterVTOL_m].';
    initialNorthAfterVTOL_mBySortie = [sortieCommandExports.initialNorthAfterVTOL_m].';
    initialUpAfterVTOL_mBySortie = [sortieCommandExports.initialUpAfterVTOL_m].';
    initialCruiseStartMissionTimeBySortie = [sortieCommandExports.startMissionTime].';
    if nSorties >= 1
        initialCruiseStartXYZBySortie = vertcat(sortieCommandExports.startPositionXYZ);
    else
        initialCruiseStartXYZBySortie = zeros(0,3);
    end

    if isempty(initialCruiseStartXYZBySortie)
        InitialPosBySortie = zeros(0,3);
    else
        InitialPosBySortie = [ ...
            initialCruiseStartXYZBySortie(:,2), ...
            initialCruiseStartXYZBySortie(:,1), ...
            -abs(initialCruiseStartXYZBySortie(:,3))];
    end
    InitialHeadingRadBySortie = initialHeadingAfterVTOLRadBySortie;
    InitialHeadingDegBySortie = initialHeadingAfterVTOLDegBySortie;

    sortieInitialConditionExports = repmat(struct( ...
        'sortieNumber', NaN, ...
        'InitialPosNED', [NaN NaN NaN], ...
        'InitialHeadingRad', NaN, ...
        'InitialHeadingDeg', NaN, ...
        'InitialCruiseStartXYZ', [NaN NaN NaN], ...
        'InitialCruiseStartMissionTime', NaN), nSorties, 1);
    for iSortie = 1:nSorties
        sortieInitialConditionExports(iSortie).sortieNumber = iSortie;
        sortieInitialConditionExports(iSortie).InitialPosNED = InitialPosBySortie(iSortie,:);
        sortieInitialConditionExports(iSortie).InitialHeadingRad = InitialHeadingRadBySortie(iSortie);
        sortieInitialConditionExports(iSortie).InitialHeadingDeg = InitialHeadingDegBySortie(iSortie);
        sortieInitialConditionExports(iSortie).InitialCruiseStartXYZ = initialCruiseStartXYZBySortie(iSortie,:);
        sortieInitialConditionExports(iSortie).InitialCruiseStartMissionTime = initialCruiseStartMissionTimeBySortie(iSortie);
    end

    commandExportBundle = struct();
    commandExportBundle.sortieCommandExports = sortieCommandExports;
    commandExportBundle.rollCmdTimeBySortie = rollCmdTimeBySortie;
    commandExportBundle.rollCmdRadBySortie = rollCmdRadBySortie;
    commandExportBundle.rollCmdDegBySortie = rollCmdDegBySortie;
    commandExportBundle.rollCmdMatrixBySortie = rollCmdMatrixBySortie;
    commandExportBundle.rollCmdTSBySortie = rollCmdTSBySortie;
    commandExportBundle.flightPathCmdTimeBySortie = flightPathCmdTimeBySortie;
    commandExportBundle.flightPathCmdRadBySortie = flightPathCmdRadBySortie;
    commandExportBundle.flightPathCmdDegBySortie = flightPathCmdDegBySortie;
    commandExportBundle.flightPathCmdMatrixBySortie = flightPathCmdMatrixBySortie;
    commandExportBundle.flightPathCmdTSBySortie = flightPathCmdTSBySortie;
    commandExportBundle.headingCmdTimeBySortie = headingCmdTimeBySortie;
    commandExportBundle.headingCmdRadBySortie = headingCmdRadBySortie;
    commandExportBundle.headingCmdDegBySortie = headingCmdDegBySortie;
    commandExportBundle.headingCmdMatrixBySortie = headingCmdMatrixBySortie;
    commandExportBundle.headingCmdTSBySortie = headingCmdTSBySortie;
    commandExportBundle.altitudeCmdTimeBySortie = altitudeCmdTimeBySortie;
    commandExportBundle.altitudeCmdMBySortie = altitudeCmdMBySortie;
    commandExportBundle.altitudeCmdMatrixBySortie = altitudeCmdMatrixBySortie;
    commandExportBundle.altitudeCmdTSBySortie = altitudeCmdTSBySortie;
    commandExportBundle.InitialPosBySortie = InitialPosBySortie;
    commandExportBundle.InitialHeadingRadBySortie = InitialHeadingRadBySortie;
    commandExportBundle.InitialHeadingDegBySortie = InitialHeadingDegBySortie;
    commandExportBundle.sortieInitialConditionExports = sortieInitialConditionExports;

    missionExport = struct();
    missionExport.formatVersion = 'lane_struct_v1';
    missionExport.coordinateSystem = struct('position', 'NED', 'heading', 'rad', 'altitudeCommand', 'Up');
    missionExport.sortieCount = nSorties;
    missionExport.sorties = repmat(struct( ...
        'sortieNumber', NaN, ...
        'InitialPosNED', [NaN NaN NaN], ...
        'InitialHeadingRad', NaN, ...
        'InitialHeadingDeg', NaN, ...
        'InitialCruiseStartXYZ', [NaN NaN NaN], ...
        'InitialCruiseStartMissionTime', NaN, ...
        'Time', zeros(0,1), ...
        'RollCmdRad', zeros(0,1), ...
        'FlightPathCmdRad', zeros(0,1), ...
        'HeadingCmdRad', zeros(0,1), ...
        'AltitudeCmdM', zeros(0,1), ...
        'RollCmdMatrix', zeros(0,2), ...
        'FlightPathCmdMatrix', zeros(0,2), ...
        'HeadingCmdMatrix', zeros(0,2), ...
        'AltitudeCmdMatrix', zeros(0,2), ...
        'Timeseries', struct('Roll', [], 'FlightPath', [], 'Heading', [], 'Altitude', [])), nSorties, 1);
    for iSortie = 1:nSorties
        missionExport.sorties(iSortie).sortieNumber = iSortie;
        missionExport.sorties(iSortie).InitialPosNED = InitialPosBySortie(iSortie,:);
        missionExport.sorties(iSortie).InitialHeadingRad = InitialHeadingRadBySortie(iSortie);
        missionExport.sorties(iSortie).InitialHeadingDeg = InitialHeadingDegBySortie(iSortie);
        missionExport.sorties(iSortie).InitialCruiseStartXYZ = initialCruiseStartXYZBySortie(iSortie,:);
        missionExport.sorties(iSortie).InitialCruiseStartMissionTime = initialCruiseStartMissionTimeBySortie(iSortie);
        missionExport.sorties(iSortie).Time = headingCmdTimeBySortie{iSortie};
        missionExport.sorties(iSortie).RollCmdRad = rollCmdRadBySortie{iSortie};
        missionExport.sorties(iSortie).FlightPathCmdRad = flightPathCmdRadBySortie{iSortie};
        missionExport.sorties(iSortie).HeadingCmdRad = headingCmdRadBySortie{iSortie};
        missionExport.sorties(iSortie).AltitudeCmdM = altitudeCmdMBySortie{iSortie};
        missionExport.sorties(iSortie).RollCmdMatrix = rollCmdMatrixBySortie{iSortie};
        missionExport.sorties(iSortie).FlightPathCmdMatrix = flightPathCmdMatrixBySortie{iSortie};
        missionExport.sorties(iSortie).HeadingCmdMatrix = headingCmdMatrixBySortie{iSortie};
        missionExport.sorties(iSortie).AltitudeCmdMatrix = altitudeCmdMatrixBySortie{iSortie};
        missionExport.sorties(iSortie).Timeseries.Roll = rollCmdTSBySortie{iSortie};
        missionExport.sorties(iSortie).Timeseries.FlightPath = flightPathCmdTSBySortie{iSortie};
        missionExport.sorties(iSortie).Timeseries.Heading = headingCmdTSBySortie{iSortie};
        missionExport.sorties(iSortie).Timeseries.Altitude = altitudeCmdTSBySortie{iSortie};
    end
    commandExportBundle.missionExport = missionExport;
    commandExportBundle.laneMissionStructs = planningData.laneMissionStructs;

    if nSorties >= 1
        rollCmdTime = rollCmdTimeBySortie{1};
        rollCmdRad = rollCmdRadBySortie{1};
        rollCmdDeg = rollCmdDegBySortie{1};
        rollCmdMatrix = rollCmdMatrixBySortie{1};
        rollCmdTS = rollCmdTSBySortie{1};

        flightPathCmdTime = flightPathCmdTimeBySortie{1};
        flightPathCmdRad = flightPathCmdRadBySortie{1};
        flightPathCmdDeg = flightPathCmdDegBySortie{1};
        flightPathCmdMatrix = flightPathCmdMatrixBySortie{1};
        flightPathCmdTS = flightPathCmdTSBySortie{1};

        headingCmdTime = headingCmdTimeBySortie{1};
        headingCmdRad = headingCmdRadBySortie{1};
        headingCmdDeg = headingCmdDegBySortie{1};
        headingCmdMatrix = headingCmdMatrixBySortie{1};
        headingCmdTS = headingCmdTSBySortie{1};

        altitudeCmdTime = altitudeCmdTimeBySortie{1};
        altitudeCmdM = altitudeCmdMBySortie{1};
        altitudeCmdMatrix = altitudeCmdMatrixBySortie{1};
        altitudeCmdTS = altitudeCmdTSBySortie{1};

        upCmdTime = altitudeCmdTime;
        upCmdM = altitudeCmdM;
        upCmdMatrix = altitudeCmdMatrix;
        upCmdTS = altitudeCmdTS;

        initialHeadingAfterVTOLRad = initialHeadingAfterVTOLRadBySortie(1);
        initialHeadingAfterVTOLDeg = initialHeadingAfterVTOLDegBySortie(1);
        initialHeightAfterVTOL_m = initialHeightAfterVTOL_mBySortie(1);
        initialEastAfterVTOL_m = initialEastAfterVTOL_mBySortie(1);
        initialNorthAfterVTOL_m = initialNorthAfterVTOL_mBySortie(1);
        initialUpAfterVTOL_m = initialUpAfterVTOL_mBySortie(1);
    else
        rollCmdTime = zeros(0,1); rollCmdRad = zeros(0,1); rollCmdDeg = zeros(0,1);
        rollCmdMatrix = zeros(0,2); rollCmdTS = timeseries([], [], 'Name', 'rollCmdRad');
        flightPathCmdTime = zeros(0,1); flightPathCmdRad = zeros(0,1); flightPathCmdDeg = zeros(0,1);
        flightPathCmdMatrix = zeros(0,2); flightPathCmdTS = timeseries([], [], 'Name', 'flightPathCmdRad');
        headingCmdTime = zeros(0,1); headingCmdRad = zeros(0,1); headingCmdDeg = zeros(0,1);
        headingCmdMatrix = zeros(0,2); headingCmdTS = timeseries([], [], 'Name', 'headingCmdRad');
        altitudeCmdTime = zeros(0,1); altitudeCmdM = zeros(0,1); altitudeCmdMatrix = zeros(0,2);
        altitudeCmdTS = timeseries([], [], 'Name', 'altitudeCmd_m');
        upCmdTime = altitudeCmdTime; upCmdM = altitudeCmdM; upCmdMatrix = altitudeCmdMatrix; upCmdTS = altitudeCmdTS;
        initialHeadingAfterVTOLRad = NaN; initialHeadingAfterVTOLDeg = NaN; initialHeightAfterVTOL_m = NaN;
        initialEastAfterVTOL_m = NaN; initialNorthAfterVTOL_m = NaN; initialUpAfterVTOL_m = NaN;
    end

    simulationData = struct();
    simulationData.pathXYZ = pathXYZ;
    simulationData.pathRollAngle = pathRollAngle;
    simulationData.pathFlightPathAngle = pathFlightPathAngle;
    simulationData.pathTime = pathTime;
    simulationData.segmentTimes = segmentTimes;
    simulationData.segmentModes = segmentModes;
    simulationData.segmentStages = segmentStages;
    simulationData.segmentPass = segmentPass;
    simulationData.segmentSortie = segmentSortie;
    simulationData.segmentPowerW = segmentPowerW;
    simulationData.segmentLengths = segmentLengths;
    simulationData.payloadRemaining_L = payloadRemaining_L;
    simulationData.batteryRemaining_Wh = batteryRemaining_Wh;
    simulationData.batteryPct = batteryPct;
    simulationData.totalLitresDispensed = totalLitresDispensed;
    simulationData.sprayedArea_ha = sprayedArea_ha;
    simulationData.droneWeightKg = droneWeightKg;
    simulationData.finalBatteryPct = finalBatteryPct;
    simulationData.finalWeightKg = finalWeightKg;
    simulationData.totalPayloadUsed_L = totalPayloadUsed_L;
    simulationData.areaNotSprayed_ha = areaNotSprayed_ha;
    simulationData.passCompletionMissionTime = passCompletionMissionTime;
    simulationData.sprayDistance = sprayDistance;
    simulationData.laneTurnDistance = laneTurnDistance;
    simulationData.vtolDistance = vtolDistance;
    simulationData.transferDistance = transferDistance;
    simulationData.totalDistance = totalDistance;
    simulationData.sprayTime = sprayTime;
    simulationData.laneTurnTime = laneTurnTime;
    simulationData.vtolTimeTotal = vtolTimeTotal;
    simulationData.transferTimeTotal = transferTimeTotal;
    simulationData.turnaroundTimeTotal = turnaroundTimeTotal;
    simulationData.turnaroundTime_s = turnaroundTime_s;
    simulationData.payloadTurnaroundTime_s = payloadTurnaroundTime_s;
    simulationData.batteryTurnaroundTime_s = batteryTurnaroundTime_s;
    simulationData.flightTimeTotal = flightTimeTotal;
    simulationData.totalTime = totalTime;
    simulationData.sprayEnergy_Wh = sprayEnergy_Wh;
    simulationData.laneTurnEnergy_Wh = laneTurnEnergy_Wh;
    simulationData.vtolEnergy_Wh = vtolEnergy_Wh;
    simulationData.transferEnergy_Wh = transferEnergy_Wh;
    simulationData.missionEnergyDemand_Wh = missionEnergyDemand_Wh;
    simulationData.missionEnergyDemand_kWh = missionEnergyDemand_kWh;
    simulationData.sprayRate_ha_per_hr = sprayRate_ha_per_hr;
    simulationData.overallFieldCapacity_ha_per_hr = overallFieldCapacity_ha_per_hr;
    simulationData.sprayTimeFraction_pct = sprayTimeFraction_pct;
    simulationData.nonProductiveTime_min = nonProductiveTime_min;
    simulationData.nonProductiveTime_pct = nonProductiveTime_pct;
    simulationData.turnaroundShare_pct = turnaroundShare_pct;
    simulationData.transitShare_pct = transitShare_pct;
    simulationData.turnShare_pct = turnShare_pct;
    simulationData.averageAreaPerSortie_ha = averageAreaPerSortie_ha;
    simulationData.payloadTriggerCount = payloadTriggerCount;
    simulationData.batteryTriggerCount = batteryTriggerCount;
    simulationData.completeSortieCount = completeSortieCount;
    simulationData.returnCount = returnCount;
    simulationData.energyPerHa_kWh = energyPerHa_kWh;
    simulationData.turnsPerHa = turnsPerHa;
    simulationData.sprayLabourBillableHours = sprayLabourBillableHours;
    simulationData.missionLabourBillableHours = missionLabourBillableHours;
    simulationData.labourCost_GBP = labourCost_GBP;
    simulationData.batteryChargeCount = batteryChargeCount;
    simulationData.batteryChargingCost_GBP = batteryChargingCost_GBP;
    simulationData.maintenanceFlightHours = maintenanceFlightHours;
    simulationData.maintenanceCost_GBP = maintenanceCost_GBP;
    simulationData.totalRevenue_GBP = totalRevenue_GBP;
    simulationData.totalCost_GBP = totalCost_GBP;
    simulationData.netProfit_GBP = netProfit_GBP;
    simulationData.rtbTriggerTimes = rtbTriggerTimes;
    simulationData.rtbArrivalTimes = rtbArrivalTimes;
    simulationData.rtbTriggerTypes = rtbTriggerTypes;
    simulationData.batteryPctAtRTBTrigger = batteryPctAtRTBTrigger;
    simulationData.payloadAtRTBTrigger_L = payloadAtRTBTrigger_L;
    simulationData.avgBatteryPctAtRTBTrigger = avgBatteryPctAtRTBTrigger;
    simulationData.minBatteryPctAtRTBTrigger = minBatteryPctAtRTBTrigger;
    simulationData.batteryPctAtGSArrival = batteryPctAtGSArrival;
    simulationData.avgBatteryPctAtGSArrival = avgBatteryPctAtGSArrival;
    simulationData.pathHeadingAngle = pathHeadingAngle;
    simulationData.pathHeadingAngleUnwrapped = pathHeadingAngleUnwrapped;
    simulationData.payloadInterruptPoints = payloadInterruptPoints;
    simulationData.batteryInterruptPoints = batteryInterruptPoints;
    simulationData.payloadInterruptSorties = payloadInterruptSorties;
    simulationData.batteryInterruptSorties = batteryInterruptSorties;
    simulationData.sortieLog = sortieLog;
    simulationData.nSorties = nSorties;
    simulationData.sortieCommandExports = sortieCommandExports;
    simulationData.rollCmdTimeBySortie = rollCmdTimeBySortie;
    simulationData.rollCmdRadBySortie = rollCmdRadBySortie;
    simulationData.rollCmdDegBySortie = rollCmdDegBySortie;
    simulationData.rollCmdMatrixBySortie = rollCmdMatrixBySortie;
    simulationData.rollCmdTSBySortie = rollCmdTSBySortie;
    simulationData.flightPathCmdTimeBySortie = flightPathCmdTimeBySortie;
    simulationData.flightPathCmdRadBySortie = flightPathCmdRadBySortie;
    simulationData.flightPathCmdDegBySortie = flightPathCmdDegBySortie;
    simulationData.flightPathCmdMatrixBySortie = flightPathCmdMatrixBySortie;
    simulationData.flightPathCmdTSBySortie = flightPathCmdTSBySortie;
    simulationData.headingCmdTimeBySortie = headingCmdTimeBySortie;
    simulationData.headingCmdRadBySortie = headingCmdRadBySortie;
    simulationData.headingCmdDegBySortie = headingCmdDegBySortie;
    simulationData.headingCmdMatrixBySortie = headingCmdMatrixBySortie;
    simulationData.headingCmdTSBySortie = headingCmdTSBySortie;
    simulationData.altitudeCmdTimeBySortie = altitudeCmdTimeBySortie;
    simulationData.altitudeCmdMBySortie = altitudeCmdMBySortie;
    simulationData.altitudeCmdMatrixBySortie = altitudeCmdMatrixBySortie;
    simulationData.altitudeCmdTSBySortie = altitudeCmdTSBySortie;
    simulationData.upCmdTimeBySortie = upCmdTimeBySortie;
    simulationData.upCmdMBySortie = upCmdMBySortie;
    simulationData.upCmdMatrixBySortie = upCmdMatrixBySortie;
    simulationData.upCmdTSBySortie = upCmdTSBySortie;
    simulationData.initialHeadingAfterVTOLRadBySortie = initialHeadingAfterVTOLRadBySortie;
    simulationData.initialHeadingAfterVTOLDegBySortie = initialHeadingAfterVTOLDegBySortie;
    simulationData.initialHeightAfterVTOL_mBySortie = initialHeightAfterVTOL_mBySortie;
    simulationData.initialEastAfterVTOL_mBySortie = initialEastAfterVTOL_mBySortie;
    simulationData.initialNorthAfterVTOL_mBySortie = initialNorthAfterVTOL_mBySortie;
    simulationData.initialUpAfterVTOL_mBySortie = initialUpAfterVTOL_mBySortie;
    simulationData.initialCruiseStartMissionTimeBySortie = initialCruiseStartMissionTimeBySortie;
    simulationData.initialCruiseStartXYZBySortie = initialCruiseStartXYZBySortie;
    simulationData.InitialPosBySortie = InitialPosBySortie;
    simulationData.InitialHeadingRadBySortie = InitialHeadingRadBySortie;
    simulationData.InitialHeadingDegBySortie = InitialHeadingDegBySortie;
    simulationData.sortieInitialConditionExports = sortieInitialConditionExports;
    simulationData.commandExportBundle = commandExportBundle;
    simulationData.missionExport = missionExport;
    simulationData.rollCmdTime = rollCmdTime;
    simulationData.rollCmdRad = rollCmdRad;
    simulationData.rollCmdDeg = rollCmdDeg;
    simulationData.rollCmdMatrix = rollCmdMatrix;
    simulationData.rollCmdTS = rollCmdTS;
    simulationData.flightPathCmdTime = flightPathCmdTime;
    simulationData.flightPathCmdRad = flightPathCmdRad;
    simulationData.flightPathCmdDeg = flightPathCmdDeg;
    simulationData.flightPathCmdMatrix = flightPathCmdMatrix;
    simulationData.flightPathCmdTS = flightPathCmdTS;
    simulationData.headingCmdTime = headingCmdTime;
    simulationData.headingCmdRad = headingCmdRad;
    simulationData.headingCmdDeg = headingCmdDeg;
    simulationData.headingCmdMatrix = headingCmdMatrix;
    simulationData.headingCmdTS = headingCmdTS;
    simulationData.altitudeCmdTime = altitudeCmdTime;
    simulationData.altitudeCmdM = altitudeCmdM;
    simulationData.altitudeCmdMatrix = altitudeCmdMatrix;
    simulationData.altitudeCmdTS = altitudeCmdTS;
    simulationData.upCmdTime = upCmdTime;
    simulationData.upCmdM = upCmdM;
    simulationData.upCmdMatrix = upCmdMatrix;
    simulationData.upCmdTS = upCmdTS;
    simulationData.initialHeadingAfterVTOLRad = initialHeadingAfterVTOLRad;
    simulationData.initialHeadingAfterVTOLDeg = initialHeadingAfterVTOLDeg;
    simulationData.initialHeightAfterVTOL_m = initialHeightAfterVTOL_m;
    simulationData.initialEastAfterVTOL_m = initialEastAfterVTOL_m;
    simulationData.initialNorthAfterVTOL_m = initialNorthAfterVTOL_m;
    simulationData.initialUpAfterVTOL_m = initialUpAfterVTOL_m;

    %% -------------------- DISPLAY PATHS --------------------
    [baseDisplayXYZ, ~, ~, ~, ~, ~, ~, ~] = buildDisplayMission( ...
        planningData.basePathXYZ, planningData.basePathTime, planningData.basePathRollAngle, planningData.basePathFlightPathAngle, ...
        planningData.baseSegmentModes, planningData.baseSegmentStages, planningData.baseSegmentPass, ones(size(planningData.baseSegmentModes)), ...
        displayDsSpray, displayDsLaneTurn, displayDsTransit);

    [displayPathXYZ, displayPathTimeReal, displayPathRoll, displayPathGamma, ...
        displaySegmentModes, displaySegmentStages, displaySegmentPass, displaySegmentSortie] = ...
        buildDisplayMission(pathXYZ, pathTime, pathRollAngle, pathFlightPathAngle, ...
        segmentModes, segmentStages, segmentPass, segmentSortie, ...
        displayDsSpray, displayDsLaneTurn, displayDsTransit);

    %% -------------------- INTERPOLANTS --------------------
    Fx = makeSafeInterpolant(pathTime, pathXYZ(:,1), 'linear');
    Fy = makeSafeInterpolant(pathTime, pathXYZ(:,2), 'linear');
    Fz = makeSafeInterpolant(pathTime, pathXYZ(:,3), 'linear');
    FRoll = makeSafeInterpolant(pathTime, pathRollAngle, 'linear');
    FGamma = makeSafeInterpolant(pathTime, pathFlightPathAngle, 'linear');
    FHeading = makeSafeInterpolant(pathTime, pathHeadingAngleUnwrapped, 'linear');
    FBatteryPct = makeSafeInterpolant(pathTime, batteryPct, 'linear');
    FPayloadRem = makeSafeInterpolant(pathTime, payloadRemaining_L, 'linear');
    FTotalLitres = makeSafeInterpolant(pathTime, totalLitresDispensed, 'linear');
    FAreaSprayed = makeSafeInterpolant(pathTime, sprayedArea_ha, 'linear');
    FWeightKg = makeSafeInterpolant(pathTime, droneWeightKg, 'linear');

    stateInterp.Fx = Fx;
    stateInterp.Fy = Fy;
    stateInterp.Fz = Fz;
    stateInterp.FRoll = FRoll;
    stateInterp.FGamma = FGamma;
    stateInterp.FHeading = FHeading;
    stateInterp.FBatteryPct = FBatteryPct;
    stateInterp.FPayloadRem = FPayloadRem;
    stateInterp.FTotalLitres = FTotalLitres;
    stateInterp.FAreaSprayed = FAreaSprayed;
    stateInterp.FWeightKg = FWeightKg;
    stateInterp.totalMissionTime = totalTime;

    %% -------------------- SORTIE LOG LINES --------------------
    sortieLines = cell(nSorties,1);
    for i = 1:nSorties
        if strcmp(sortieLog(i).interruptReason, 'complete')
            sortieLines{i} = sprintf('Sortie %02d | Start %.1f s | GS %.1f s | Mission complete %.1f s', ...
                sortieLog(i).sortieNumber, sortieLog(i).startMissionT, sortieLog(i).arrivalGST, sortieLog(i).completeT);
        else
            sortieLines{i} = sprintf('Sortie %02d | %s at %.1f s | GS %.1f s | Ready %.1f s', ...
                sortieLog(i).sortieNumber, upper(sortieLog(i).interruptReason), sortieLog(i).interruptT, ...
                sortieLog(i).arrivalGST, sortieLog(i).readyAgainT);
        end
    end

    %% -------------------- PLAYBACK DATA --------------------
    playbackModes = cell(nSorties + 1,1);
    playbackModeNames = cell(nSorties + 1,1);

    playbackModeNames{1} = 'Full mission';
    playbackModes{1} = buildPlaybackPackSimple( ...
        'Full mission', ...
        displayPathXYZ, displayPathTimeReal, displaySegmentModes, displaySegmentStages, ...
        displaySegmentPass, displaySegmentSortie, compressTurnaroundInPlayback, ...
        turnaroundPlaybackTime_s, targetPlaybackDurationFull_s, playbackFPS, stateInterp);

    for s = 1:nSorties
        [sliceXYZ, sliceTimeReal, sliceSegModes, sliceSegStages, sliceSegPass, sliceSegSortie] = ...
            extractDisplaySliceForSortie(displayPathXYZ, displayPathTimeReal, ...
            displaySegmentModes, displaySegmentStages, displaySegmentPass, displaySegmentSortie, s);

        playbackModeNames{s+1} = sprintf('Sortie %d only', s);
        playbackModes{s+1} = buildPlaybackPackSimple( ...
            sprintf('Sortie %d only', s), ...
            sliceXYZ, sliceTimeReal, sliceSegModes, sliceSegStages, ...
            sliceSegPass, sliceSegSortie, compressTurnaroundInPlayback, ...
            turnaroundPlaybackTime_s, targetPlaybackDurationSortie_s, playbackFPS, stateInterp);
    end

    simulationData.baseDisplayXYZ = baseDisplayXYZ;
    simulationData.displayPathXYZ = displayPathXYZ;
    simulationData.displayPathTimeReal = displayPathTimeReal;
    simulationData.displayPathRoll = displayPathRoll;
    simulationData.displayPathGamma = displayPathGamma;
    simulationData.displaySegmentModes = displaySegmentModes;
    simulationData.displaySegmentStages = displaySegmentStages;
    simulationData.displaySegmentPass = displaySegmentPass;
    simulationData.displaySegmentSortie = displaySegmentSortie;
    simulationData.stateInterp = stateInterp;
    simulationData.sortieLines = sortieLines;
    simulationData.playbackModes = playbackModes;
    simulationData.playbackModeNames = playbackModeNames;
    simulationData.groundStationToFirstCheckpointDistance = groundStationToFirstCheckpointDistance;
    simulationData.plannedSprayArea_ha = plannedSprayArea_ha;
    simulationData.fieldSummaryLines = fieldSummaryLines;
    simulationData.fieldFirstCheckpoints = fieldFirstCheckpoints;
    simulationData.fieldInfo = fieldInfo;
    simulationData.fieldVisitOrder = fieldVisitOrder;
    simulationData.fieldCount = fieldCount;
    validateSimulationData(simulationData);
end

function exportStruct = makeEmptySortieCommandExport()
    exportStruct = struct( ...
        'sortieNumber', NaN, ...
        'startMissionTime', NaN, ...
        'endMissionTime', NaN, ...
        'startPositionXYZ', [NaN NaN NaN], ...
        'endPositionXYZ', [NaN NaN NaN], ...
        'initialHeadingAfterVTOLRad', NaN, ...
        'initialHeadingAfterVTOLDeg', NaN, ...
        'initialHeightAfterVTOL_m', NaN, ...
        'initialEastAfterVTOL_m', NaN, ...
        'initialNorthAfterVTOL_m', NaN, ...
        'initialUpAfterVTOL_m', NaN, ...
        'rollCmdTime', zeros(0,1), 'rollCmdRad', zeros(0,1), 'rollCmdDeg', zeros(0,1), ...
        'rollCmdMatrix', zeros(0,2), 'rollCmdTS', timeseries([], [], 'Name', 'rollCmdRad'), ...
        'flightPathCmdTime', zeros(0,1), 'flightPathCmdRad', zeros(0,1), 'flightPathCmdDeg', zeros(0,1), ...
        'flightPathCmdMatrix', zeros(0,2), 'flightPathCmdTS', timeseries([], [], 'Name', 'flightPathCmdRad'), ...
        'headingCmdTime', zeros(0,1), 'headingCmdRad', zeros(0,1), 'headingCmdDeg', zeros(0,1), ...
        'headingCmdMatrix', zeros(0,2), 'headingCmdTS', timeseries([], [], 'Name', 'headingCmdRad'), ...
        'altitudeCmdTime', zeros(0,1), 'altitudeCmdM', zeros(0,1), ...
        'altitudeCmdMatrix', zeros(0,2), 'altitudeCmdTS', timeseries([], [], 'Name', 'altitudeCmd_m'));
end

function exportStruct = buildCruiseOnlySortieCommandExport(sortieNumber, pathXYZ, pathRollAngle, pathFlightPathAngle, pathTime, ...
    segmentStages, segmentSortie, rollCmdDt, rollCmdInterpMethod)

    codes = missionCodes();
    exportStruct = makeEmptySortieCommandExport();
    exportStruct.sortieNumber = sortieNumber;

    cruiseOnlyStageMask = ~ismember(segmentStages, codes.stageGroups.cruiseOnlyExcluded);
    idxSeg = find(segmentSortie == sortieNumber & cruiseOnlyStageMask);
    if isempty(idxSeg)
        return
    end

    firstSeg = idxSeg(1);
    lastSeg = idxSeg(end);

    sortieXYZ = pathXYZ(firstSeg:lastSeg+1,:);
    sortieRoll = pathRollAngle(firstSeg:lastSeg+1);
    sortieGamma = pathFlightPathAngle(firstSeg:lastSeg+1);
    sortieTime = pathTime(firstSeg:lastSeg+1);
    sortieTime = sortieTime - sortieTime(1);

    if numel(sortieTime) < 2 || sortieTime(end) <= 0
        return
    end

    sortieHeading = computeHeadingFromPath(sortieXYZ);
    sortieHeadingUnwrapped = unwrap(sortieHeading);

    cmdTime = buildExactSampleVector(0, sortieTime(end), rollCmdDt);

    exportStruct.startMissionTime = pathTime(firstSeg);
    exportStruct.endMissionTime = pathTime(lastSeg+1);
    exportStruct.startPositionXYZ = sortieXYZ(1,:);
    exportStruct.endPositionXYZ = sortieXYZ(end,:);
    exportStruct.initialHeadingAfterVTOLRad = wrapToPiLocal(sortieHeadingUnwrapped(1));
    exportStruct.initialHeadingAfterVTOLDeg = rad2deg(exportStruct.initialHeadingAfterVTOLRad);
    exportStruct.initialHeightAfterVTOL_m = sortieXYZ(1,3);
    exportStruct.initialEastAfterVTOL_m = sortieXYZ(1,1);
    exportStruct.initialNorthAfterVTOL_m = sortieXYZ(1,2);
    exportStruct.initialUpAfterVTOL_m = sortieXYZ(1,3);

    exportStruct.rollCmdTime = cmdTime;
    exportStruct.rollCmdRad = interp1SafeNonUnique(sortieTime, sortieRoll, cmdTime, rollCmdInterpMethod);
    exportStruct.rollCmdDeg = rad2deg(exportStruct.rollCmdRad);
    exportStruct.rollCmdMatrix = [exportStruct.rollCmdTime(:), exportStruct.rollCmdRad(:)];
    exportStruct.rollCmdTS = timeseries(exportStruct.rollCmdRad(:), exportStruct.rollCmdTime(:), 'Name', sprintf('rollCmdRad_sortie%02d', sortieNumber));

    exportStruct.flightPathCmdTime = cmdTime;
    exportStruct.flightPathCmdRad = interp1SafeNonUnique(sortieTime, sortieGamma, cmdTime, rollCmdInterpMethod);
    exportStruct.flightPathCmdDeg = rad2deg(exportStruct.flightPathCmdRad);
    exportStruct.flightPathCmdMatrix = [exportStruct.flightPathCmdTime(:), exportStruct.flightPathCmdRad(:)];
    exportStruct.flightPathCmdTS = timeseries(exportStruct.flightPathCmdRad(:), exportStruct.flightPathCmdTime(:), 'Name', sprintf('flightPathCmdRad_sortie%02d', sortieNumber));

    exportStruct.headingCmdTime = cmdTime;
    headingRadUnwrapped = interp1SafeNonUnique(sortieTime, sortieHeadingUnwrapped, cmdTime, rollCmdInterpMethod);
    exportStruct.headingCmdRad = wrapToPiLocal(headingRadUnwrapped);
    exportStruct.headingCmdDeg = rad2deg(exportStruct.headingCmdRad);
    exportStruct.headingCmdMatrix = [exportStruct.headingCmdTime(:), exportStruct.headingCmdRad(:)];
    exportStruct.headingCmdTS = timeseries(exportStruct.headingCmdRad(:), exportStruct.headingCmdTime(:), 'Name', sprintf('headingCmdRad_sortie%02d', sortieNumber));

    exportStruct.altitudeCmdTime = cmdTime;
    exportStruct.altitudeCmdM = interp1SafeNonUnique(sortieTime, sortieXYZ(:,3), cmdTime, rollCmdInterpMethod);
    exportStruct.altitudeCmdMatrix = [exportStruct.altitudeCmdTime(:), exportStruct.altitudeCmdM(:)];
    exportStruct.altitudeCmdTS = timeseries(exportStruct.altitudeCmdM(:), exportStruct.altitudeCmdTime(:), 'Name', sprintf('altitudeCmd_m_sortie%02d', sortieNumber));
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

function vq = interp1SafeNonUnique(x, v, xq, method)
    [xUnique, vUnique] = collapseInterpSamples(x, v);

    if isempty(xUnique)
        vq = nan(size(xq));
        return
    end

    if numel(xUnique) == 1
        if isvector(vUnique)
            vq = repmat(vUnique(1), size(xq));
        else
            vq = repmat(vUnique(1,:), numel(xq), 1);
        end
        return
    end

    vq = interp1(xUnique, vUnique, xq, method);
end

function F = makeSafeInterpolant(x, v, method)
    [xUnique, vUnique] = collapseInterpSamples(x, v);

    if isempty(xUnique)
        F = @(xq) nan(size(xq));
        return
    end

    if numel(xUnique) == 1
        if isvector(vUnique)
            constantValue = vUnique(1);
            F = @(xq) repmat(constantValue, size(xq));
        else
            constantValue = vUnique(1,:);
            F = @(xq) repmat(constantValue, numel(xq), 1);
        end
        return
    end

    G = griddedInterpolant(xUnique, vUnique, method);
    F = @(xq) G(xq);
end

function [xUnique, vUnique] = collapseInterpSamples(x, v)
    x = x(:);
    if size(v,1) ~= numel(x)
        error('collapseInterpSamples:SizeMismatch', ...
            'Sample values must have the same number of rows as sample times.');
    end

    if isempty(x)
        xUnique = x;
        vUnique = v;
        return
    end

    keepMask = [diff(x) > 0; true];
    xUnique = x(keepMask);
    vUnique = v(keepMask,:);
end

function [pos, roll, gamma, heading, passNum, modeCode] = ...
    getPathStateAtTime(t, pathTime, pathXYZ, Fx, Fy, Fz, FRoll, FGamma, segmentModes, segmentPass)

    nSeg = numel(pathTime) - 1;
    t = max(pathTime(1), min(t, pathTime(end)));

    pos = [Fx(t), Fy(t), Fz(t)];
    roll = FRoll(t);
    gamma = FGamma(t);

    if t >= pathTime(end) - 1e-12
        segIdx = nSeg;
    else
        segIdx = findSegmentIndex(t, pathTime);
    end

    if segIdx < size(pathXYZ,1)
        dP = pathXYZ(segIdx+1,:) - pathXYZ(segIdx,:);
    else
        dP = pathXYZ(end,:) - pathXYZ(end-1,:);
    end
    if norm(dP(1:2)) < 1e-12
        dP = [1 0 0];
    end
    heading = atan2(dP(1), dP(2));
    passNum = segmentPass(segIdx);
    modeCode = segmentModes(segIdx);
end

function idx = findSegmentIndex(t, pathTime)
    nSeg = numel(pathTime) - 1;
    if nSeg <= 0
        idx = 1;
    elseif t >= pathTime(end)
        idx = nSeg;
    else
        idx = max(1, min(findLastTimeIndexLE(t, pathTime), nSeg));
    end
end

function idx = findLastTimeIndexLE(t, pathTime)
    nTime = numel(pathTime);
    if nTime == 0 || t < pathTime(1)
        idx = 0;
        return
    end
    if t >= pathTime(end)
        idx = nTime;
        return
    end

    lo = 1;
    hi = nTime;
    while lo < hi
        mid = floor((lo + hi + 1) / 2);
        if pathTime(mid) <= t
            lo = mid;
        else
            hi = mid - 1;
        end
    end
    idx = lo;
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildVerticalChunk(startXYZ, endXYZ, verticalRate_mps, modeCode, stageCode, powerW, passNumber)

    if norm(endXYZ - startXYZ) <= 1e-12
        pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
        segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
        segPass = nan(0,1); segPowerW = zeros(0,1);
        return
    end

    dz = endXYZ(3) - startXYZ(3);
    pts = [startXYZ; endXYZ];
    roll = [0; 0];
    gamma = [sign(dz)*pi/2; sign(dz)*pi/2];
    segTimes = abs(dz) / verticalRate_mps;
    segModes = modeCode;
    segStages = stageCode;
    segPass = passNumber;
    segPowerW = powerW;
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildPauseChunk(positionXYZ, pauseTime_s, modeCode, stageCode, powerW, passNumber)
    pts = [positionXYZ; positionXYZ];
    roll = [0; 0];
    gamma = [0; 0];
    segTimes = pauseTime_s;
    segModes = modeCode;
    segStages = stageCode;
    segPass = passNumber;
    segPowerW = powerW;
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildOutboundChunk(uavConn, gsGroundXYZ, gsHighXYZ, runInStartXYZ, resumePos, resumeHeading, transferAltitude_m, ...
    descentAngle_deg, preDescentLevelDistance_m, vtolClimbRate_mps, vtolHoverTime_s, ...
    uavAirSpeed, dsLine, dsTurn, cruisePowerW, vtolHoverPowerW, vtolClimbPowerW)

    codes = missionCodes();
    pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
    segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
    segPass = nan(0,1); segPowerW = zeros(0,1);

    transferStartXYZ = gsHighXYZ;
    transferStartXYZ(3) = transferAltitude_m;

    [p,r,g,st,sm,ss,sp,spw] = buildVerticalChunk(gsGroundXYZ, gsHighXYZ, vtolClimbRate_mps, ...
        codes.mode.transfer, codes.stage.vtolOutboundClimb, vtolClimbPowerW, NaN);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildPauseChunk(gsHighXYZ, vtolHoverTime_s, ...
        codes.mode.transfer, codes.stage.vtolOutboundHover, vtolHoverPowerW, NaN);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildVerticalChunk(gsHighXYZ, transferStartXYZ, vtolClimbRate_mps, ...
        codes.mode.transfer, codes.stage.vtolOutboundClimb, cruisePowerW, NaN);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    altDelta = max(transferAltitude_m - runInStartXYZ(3), 0);
    descentDistance = ternary(altDelta > 0, altDelta / tan(deg2rad(descentAngle_deg)), 0);
    descentStartXYZ = offsetPointAlongHeading(runInStartXYZ, resumeHeading, -descentDistance);
    descentStartXYZ(3) = transferAltitude_m;

    preDescentStartXYZ = offsetPointAlongHeading(descentStartXYZ, resumeHeading, -preDescentLevelDistance_m);
    preDescentStartXYZ(3) = transferAltitude_m;

    gsStartHeading = atan2(preDescentStartXYZ(1) - transferStartXYZ(1), preDescentStartXYZ(2) - transferStartXYZ(2));
    if norm(preDescentStartXYZ(1:2) - transferStartXYZ(1:2)) <= 1e-9
        gsStartHeading = resumeHeading;
    end

    [p,r,g,st,sm,ss,sp,spw] = buildDubinsChunk(uavConn, transferStartXYZ, gsStartHeading, preDescentStartXYZ, resumeHeading, ...
        dsTurn, codes.mode.transfer, codes.stage.transferCruise, uavAirSpeed, NaN, cruisePowerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(preDescentStartXYZ, descentStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.transfer, codes.stage.transferCruise, NaN, cruisePowerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(descentStartXYZ, runInStartXYZ, dsLine, uavAirSpeed, ...
        codes.mode.transfer, codes.stage.transferRejoin, NaN, cruisePowerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(runInStartXYZ, resumePos, dsLine, uavAirSpeed, ...
        codes.mode.transfer, codes.stage.transferRejoin, NaN, cruisePowerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildReturnChunk(uavConn, stopPos, stopHeading, gsGroundXYZ, returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, ...
    vtolAltitude_m, vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
    cruisePowerW, vtolHoverPowerW, ~)

    codes = missionCodes();
    gsHighXYZ = [gsGroundXYZ(1), gsGroundXYZ(2), vtolAltitude_m];

    pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
    segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
    segPass = nan(0,1); segPowerW = zeros(0,1);

    returnStartXYZ = stopPos;
    if returnRunOutDistance_m > 1e-12
        returnStartXYZ = offsetPointAlongHeading(stopPos, stopHeading, returnRunOutDistance_m);
        returnStartXYZ(3) = stopPos(3);

        [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(stopPos, returnStartXYZ, dsLine, uavAirSpeed, ...
            codes.mode.returnToBase, codes.stage.returnRunOut, NaN, cruisePowerW);
        [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);
    end

    pullUpAltitude_m = max(returnPullUpAltitude_m, returnStartXYZ(3));
    climbDistance = 0;
    if pullUpAltitude_m > returnStartXYZ(3) + 1e-12
        climbDistance = (pullUpAltitude_m - returnStartXYZ(3)) / tan(deg2rad(returnPullUpClimbAngle_deg));
    end
    climbEndXYZ = offsetPointAlongHeading(returnStartXYZ, stopHeading, climbDistance);
    climbEndXYZ(3) = pullUpAltitude_m;

    if norm(climbEndXYZ - returnStartXYZ) > 1e-12
        [p,r,g,st,sm,ss,sp,spw] = buildStraightChunk(returnStartXYZ, climbEndXYZ, dsLine, uavAirSpeed, ...
            codes.mode.returnToBase, codes.stage.returnPullUp, NaN, cruisePowerW);
        [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);
    else
        climbEndXYZ = returnStartXYZ;
    end

    goalHeadingAtGS = atan2(gsHighXYZ(1) - climbEndXYZ(1), gsHighXYZ(2) - climbEndXYZ(2));
    if norm(gsHighXYZ(1:2) - climbEndXYZ(1:2)) <= 1e-9
        goalHeadingAtGS = stopHeading;
    end

    [p,r,g,st,sm,ss,sp,spw] = buildDubinsChunk(uavConn, climbEndXYZ, stopHeading, gsHighXYZ, goalHeadingAtGS, ...
        dsTurn, codes.mode.returnToBase, codes.stage.returnCruise, uavAirSpeed, NaN, cruisePowerW);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildPauseChunk(gsHighXYZ, vtolHoverTime_s, ...
        codes.mode.returnToBase, codes.stage.preLandingHover, vtolHoverPowerW, NaN);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);

    [p,r,g,st,sm,ss,sp,spw] = buildVerticalChunk(gsHighXYZ, gsGroundXYZ, vtolClimbRate_mps, ...
        codes.mode.returnToBase, codes.stage.vtolLandingVertical, cruisePowerW, NaN);
    [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = appendChunkData(pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, p,r,g,st,sm,ss,sp,spw);
end

function returnEnergyWh = computeSimpleRequiredReturnEnergy( ...
    uavConn, currentPosXYZ, currentHeading, gsGroundXYZ, returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, ...
    vtolAltitude_m, vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
    cruisePowerW, vtolHoverPowerW, vtolClimbPowerW)

    [~, ~, ~, segTimes, ~, ~, ~, segPowerW] = buildReturnChunk( ...
        uavConn, currentPosXYZ, currentHeading, gsGroundXYZ, returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, ...
        vtolAltitude_m, vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
        cruisePowerW, vtolHoverPowerW, vtolClimbPowerW);
    returnEnergyWh = sum(segPowerW .* segTimes / 3600);
end

function marginWh = computeBatteryGuardMarginAtBaseTime( ...
    t, resumeBaseTime, batteryAtResumeWh, reserveBatteryWh, cruisePowerW, ...
    uavConn, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
    baseSegmentModes, baseSegmentPass, FBaseEnergyWh, ...
    gsGroundXYZ, returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, vtolAltitude_m, ...
    vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, vtolHoverPowerW, vtolClimbPowerW)

    [pos, ~, ~, heading, ~, ~] = getPathStateAtTime( ...
        t, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, baseSegmentModes, baseSegmentPass);

    baseEnergyUsedWh = getBaseEnergyIntervalWh(resumeBaseTime, t, basePathTime, FBaseEnergyWh);
    batteryRemainingWh = batteryAtResumeWh - baseEnergyUsedWh;
    requiredReturnEnergyWh = computeSimpleRequiredReturnEnergy( ...
        uavConn, pos, heading, gsGroundXYZ, returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, ...
        vtolAltitude_m, vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
        cruisePowerW, vtolHoverPowerW, vtolClimbPowerW);

    marginWh = batteryRemainingWh - reserveBatteryWh - requiredReturnEnergyWh;
end

function energyWh = getBaseEnergyIntervalWh(tStart, tEnd, basePathTime, FBaseEnergyWh)
    if tEnd <= tStart
        energyWh = 0;
        return
    end

    tStart = max(basePathTime(1), min(tStart, basePathTime(end)));
    tEnd = max(basePathTime(1), min(tEnd, basePathTime(end)));
    if tEnd <= tStart
        energyWh = 0;
        return
    end

    energyWh = FBaseEnergyWh(tEnd) - FBaseEnergyWh(tStart);
end

function tCross = findPayloadTriggerTime(tLo, tHi, sprayAtResume, sprayAllowance, FsSpray)
    for k = 1:30
        tMid = 0.5 * (tLo + tHi);
        if FsSpray(tMid) - sprayAtResume >= sprayAllowance
            tHi = tMid;
        else
            tLo = tMid;
        end
    end
    tCross = tHi;
end

function tCross = findBatteryTriggerTime( ...
    tLo, tHi, resumeBaseTime, batteryAtResumeWh, reserveBatteryWh, cruisePowerW, ...
    uavConn, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
    baseSegmentModes, baseSegmentPass, FBaseEnergyWh, gsGroundXYZ, ...
    returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, vtolAltitude_m, vtolClimbRate_mps, ...
    vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, vtolHoverPowerW, vtolClimbPowerW)

    for k = 1:28
        tMid = 0.5 * (tLo + tHi);
        marginMid = computeBatteryGuardMarginAtBaseTime( ...
            tMid, resumeBaseTime, batteryAtResumeWh, reserveBatteryWh, cruisePowerW, ...
            uavConn, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
            baseSegmentModes, baseSegmentPass, FBaseEnergyWh, gsGroundXYZ, ...
            returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, vtolAltitude_m, ...
            vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
            vtolHoverPowerW, vtolClimbPowerW);

        if marginMid <= 0
            tHi = tMid;
        else
            tLo = tMid;
        end
    end
    tCross = tHi;
end

function [decisionTimes, decisionPasses] = findPassEndDecisionEventsInWindow( ...
    tLo, tHi, resumeBaseTime, passEndBaseTime)

    tol = 1e-10;
    nPass = numel(passEndBaseTime);
    decisionTimes = zeros(0,1);
    decisionPasses = zeros(0,1);

    for passNum = 1:nPass
        decisionTime = passEndBaseTime(passNum);
        if ~isfinite(decisionTime)
            continue
        end
        if decisionTime < resumeBaseTime - tol
            continue
        end
        if decisionTime >= tLo - tol && decisionTime <= tHi + tol
            decisionTimes(end+1,1) = decisionTime; %#ok<AGROW>
            decisionPasses(end+1,1) = passNum; %#ok<AGROW>
        end
    end

    [decisionTimes, idxSort] = sort(decisionTimes);
    decisionPasses = decisionPasses(idxSort);
end

function [stopBaseTime, stopReason, nextResumeBaseTime] = advanceAlongBaseUntilTrigger( ...
    resumeBaseTime, basePathTime, FsSpray, sprayAllowance, ...
    batteryAtResumeWh, reserveBatteryWh, cruisePowerW, batteryGuardCheckDt_s, ...
    uavConn, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
    baseSegmentModes, baseSegmentStages, baseSegmentPass, FBaseEnergyWh, ...
    passStartBaseTime, passEndBaseTime, ...
    gsGroundXYZ, returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, vtolAltitude_m, ...
    vtolClimbRate_mps, uavAirSpeed, dsLine, dsTurn, vtolHoverTime_s, vtolHoverPowerW, vtolClimbPowerW)
    totalBaseTime = basePathTime(end);
    tNow = resumeBaseTime;
    sprayAtResume = FsSpray(resumeBaseTime);
    nextResumeBaseTime = resumeBaseTime;

    marginAtResume = computeBatteryGuardMarginAtBaseTime( ...
        resumeBaseTime, resumeBaseTime, batteryAtResumeWh, reserveBatteryWh, cruisePowerW, ...
        uavConn, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
        baseSegmentModes, baseSegmentPass, FBaseEnergyWh, gsGroundXYZ, ...
        returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, vtolAltitude_m, ...
        vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
        vtolHoverPowerW, vtolClimbPowerW);

    if marginAtResume <= 0
        stopBaseTime = resumeBaseTime;
        stopReason = 'battery';
        nextResumeBaseTime = resumeBaseTime;
        return
    end

    while tNow < totalBaseTime - 1e-10
        tNext = min(tNow + batteryGuardCheckDt_s, totalBaseTime);

        payloadCandidate = inf;
        batteryCandidate = inf;
        preTurnBatteryCandidate = inf;
        preTurnNextResumeBaseTime = NaN;

        if FsSpray(tNext) - sprayAtResume >= sprayAllowance - 1e-10
            payloadCandidate = findPayloadTriggerTime(tNow, tNext, sprayAtResume, sprayAllowance, FsSpray);
        end

        marginAtNext = computeBatteryGuardMarginAtBaseTime( ...
            tNext, resumeBaseTime, batteryAtResumeWh, reserveBatteryWh, cruisePowerW, ...
            uavConn, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
            baseSegmentModes, baseSegmentPass, FBaseEnergyWh, gsGroundXYZ, ...
            returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, vtolAltitude_m, ...
            vtolClimbRate_mps, vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, ...
            vtolHoverPowerW, vtolClimbPowerW);

        if marginAtNext <= 0
            candidateTime = findBatteryTriggerTime( ...
                tNow, tNext, resumeBaseTime, batteryAtResumeWh, reserveBatteryWh, cruisePowerW, ...
                uavConn, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, ...
                baseSegmentModes, baseSegmentPass, FBaseEnergyWh, gsGroundXYZ, ...
                returnRunOutDistance_m, returnPullUpAltitude_m, returnPullUpClimbAngle_deg, vtolAltitude_m, vtolClimbRate_mps, ...
                vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, vtolHoverPowerW, vtolClimbPowerW);

            stageQueryTime = max(basePathTime(1), candidateTime - 1e-9);
            idxCandidate = findSegmentIndex(stageQueryTime, basePathTime);
            if baseSegmentStages(idxCandidate) == missionCodes().stage.sprayLeg
                batteryCandidate = candidateTime;
            end
        end

        [decisionTimes, decisionPasses] = findPassEndDecisionEventsInWindow( ...
            tNow, tNext, resumeBaseTime, passEndBaseTime);

        for iDecision = 1:numel(decisionTimes)
            currentPass = decisionPasses(iDecision);
            nextPass = currentPass + 1;
            if nextPass > numel(passStartBaseTime)
                continue
            end
            if ~(isfinite(passStartBaseTime(nextPass)) && isfinite(passEndBaseTime(nextPass)))
                continue
            end

            decisionTime = decisionTimes(iDecision);
            if passEndBaseTime(nextPass) <= decisionTime + 1e-10
                continue
            end

            energyToDecisionWh = getBaseEnergyIntervalWh(resumeBaseTime, decisionTime, basePathTime, FBaseEnergyWh);
            batteryRemainingAtDecisionWh = batteryAtResumeWh - energyToDecisionWh;
            energyToNextLaneEndWh = getBaseEnergyIntervalWh(decisionTime, passEndBaseTime(nextPass), basePathTime, FBaseEnergyWh);

            nextLaneStateTime = max(passStartBaseTime(nextPass), passEndBaseTime(nextPass) - 1e-6);
            [~, ~, ~, nextLaneEndHeading, ~, ~] = getPathStateAtTime( ...
                nextLaneStateTime, basePathTime, basePathXYZ, FbX, FbY, FbZ, FbRoll, FbGamma, baseSegmentModes, baseSegmentPass);
            nextLaneEndPos = [FbX(passEndBaseTime(nextPass)), FbY(passEndBaseTime(nextPass)), FbZ(passEndBaseTime(nextPass))];

            requiredReturnEnergyWh = computeSimpleRequiredReturnEnergy( ...
                uavConn, nextLaneEndPos, nextLaneEndHeading, gsGroundXYZ, returnRunOutDistance_m, returnPullUpAltitude_m, ...
                returnPullUpClimbAngle_deg, vtolAltitude_m, vtolClimbRate_mps, ...
                vtolHoverTime_s, uavAirSpeed, dsLine, dsTurn, cruisePowerW, vtolHoverPowerW, vtolClimbPowerW);

            if batteryRemainingAtDecisionWh <= energyToNextLaneEndWh + reserveBatteryWh + requiredReturnEnergyWh + 1e-10
                preTurnBatteryCandidate = decisionTime;
                preTurnNextResumeBaseTime = passStartBaseTime(nextPass);
                break
            end
        end

        candidateTimes = [payloadCandidate, preTurnBatteryCandidate, batteryCandidate];
        finiteMask = isfinite(candidateTimes);
        if any(finiteMask)
            [stopBaseTime, idxLocal] = min(candidateTimes(finiteMask));
            finiteIdx = find(finiteMask);
            idxCase = finiteIdx(idxLocal);
            switch idxCase
                case 1
                    stopReason = 'payload';
                    nextResumeBaseTime = stopBaseTime;
                case 2
                    stopReason = 'battery';
                    nextResumeBaseTime = preTurnNextResumeBaseTime;
                case 3
                    stopReason = 'battery';
                    nextResumeBaseTime = stopBaseTime;
            end
            return
        elseif tNext >= totalBaseTime - 1e-10
            stopBaseTime = totalBaseTime;
            stopReason = 'complete';
            nextResumeBaseTime = totalBaseTime;
            return
        else
            tNow = tNext;
        end
    end

    stopBaseTime = totalBaseTime;
    stopReason = 'complete';
    nextResumeBaseTime = totalBaseTime;
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    extractPathSlice(t0, t1, pathTime, Fx, Fy, Fz, FRoll, FGamma, ...
    segmentModes, segmentStages, segmentPass, segmentPowerW)

    if t1 <= t0 + 1e-12
        pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
        segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
        segPass = nan(0,1); segPowerW = zeros(0,1);
        return
    end

    firstInternalIdx = findLastTimeIndexLE(t0 + 1e-12, pathTime) + 1;
    lastInternalIdx = findLastTimeIndexLE(t1 - 1e-12, pathTime);

    if firstInternalIdx <= lastInternalIdx
        internalTimes = pathTime(firstInternalIdx:lastInternalIdx);
    else
        internalTimes = zeros(0,1);
    end
    sliceTimes = [t0; internalTimes; t1];

    pts = [Fx(sliceTimes), Fy(sliceTimes), Fz(sliceTimes)];
    roll = FRoll(sliceTimes);
    gamma = FGamma(sliceTimes);
    segTimes = diff(sliceTimes);

    nSliceSeg = numel(segTimes);
    segModes = zeros(nSliceSeg,1);
    segStages = zeros(nSliceSeg,1);
    segPass = nan(nSliceSeg,1);
    segPowerW = zeros(nSliceSeg,1);

    for i = 1:nSliceSeg
        tMid = 0.5 * (sliceTimes(i) + sliceTimes(i+1));
        idx = findSegmentIndex(tMid, pathTime);
        segModes(i) = segmentModes(idx);
        segStages(i)= segmentStages(idx);
        segPass(i)  = segmentPass(idx);
        segPowerW(i)= segmentPowerW(idx);
    end
end

function [missionChunks, chunkCount] = storeChunkCells( ...
    missionChunks, chunkCount, ...
    pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW, sortieNum)

    if isempty(segTimes)
        return
    end
    chunkCount = chunkCount + 1;
    missionChunks = ensureMissionChunkCapacity(missionChunks, chunkCount);
    missionChunks.pts{chunkCount}       = pts;
    missionChunks.roll{chunkCount}      = roll;
    missionChunks.gamma{chunkCount}     = gamma;
    missionChunks.segTimes{chunkCount}  = segTimes(:);
    missionChunks.segModes{chunkCount}  = segModes(:);
    missionChunks.segStages{chunkCount} = segStages(:);
    missionChunks.segPass{chunkCount}   = segPass(:);
    missionChunks.segPowerW{chunkCount} = segPowerW(:);
    missionChunks.segSortie{chunkCount} = repmat(sortieNum, numel(segTimes), 1);
end

function missionChunks = makeEmptyMissionChunkStore(capacity)
    missionChunks.pts       = cell(capacity,1);
    missionChunks.roll      = cell(capacity,1);
    missionChunks.gamma     = cell(capacity,1);
    missionChunks.segTimes  = cell(capacity,1);
    missionChunks.segModes  = cell(capacity,1);
    missionChunks.segStages = cell(capacity,1);
    missionChunks.segPass   = cell(capacity,1);
    missionChunks.segPowerW = cell(capacity,1);
    missionChunks.segSortie = cell(capacity,1);
end

function missionChunks = ensureMissionChunkCapacity(missionChunks, requiredCount)
    currentCapacity = numel(missionChunks.pts);
    if requiredCount <= currentCapacity
        return
    end

    newCapacity = max(requiredCount, max(16, 2 * currentCapacity));
    fields = fieldnames(missionChunks);
    for iField = 1:numel(fields)
        fieldName = fields{iField};
        missionChunks.(fieldName)(currentCapacity+1:newCapacity,1) = cell(newCapacity - currentCapacity, 1);
    end
end

function sortieLog = ensureSortieLogCapacity(sortieLog, requiredCount, sortieTemplate)
    currentCapacity = numel(sortieLog);
    if requiredCount <= currentCapacity
        return
    end

    newCapacity = max(requiredCount, max(16, 2 * currentCapacity));
    sortieLog(currentCapacity+1:newCapacity,1) = repmat(sortieTemplate, newCapacity - currentCapacity, 1);
end

function points = ensurePointCapacity(points, requiredCount)
    currentCapacity = size(points,1);
    if requiredCount <= currentCapacity
        return
    end

    newCapacity = max(requiredCount, max(16, 2 * currentCapacity));
    points(currentCapacity+1:newCapacity,:) = nan(newCapacity - currentCapacity, size(points,2));
end

function [pathXYZ, pathRoll, pathGamma, segTimes, segModes, segStages, segPass, segSortie, segPowerW] = ...
    finalizeMissionChunks(missionChunks, chunkCount)

    if chunkCount == 0
        pathXYZ = zeros(0,3); pathRoll = zeros(0,1); pathGamma = zeros(0,1);
        segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
        segPass = nan(0,1); segSortie = zeros(0,1); segPowerW = zeros(0,1);
        return
    end

    totalPts = 0; totalSeg = 0;
    for i = 1:chunkCount
        pts_i = missionChunks.pts{i};
        seg_i = missionChunks.segTimes{i};
        totalSeg = totalSeg + numel(seg_i);
        if i == 1
            totalPts = totalPts + size(pts_i,1);
        else
            totalPts = totalPts + size(pts_i,1) - 1;
        end
    end

    pathXYZ = zeros(totalPts,3); pathRoll = zeros(totalPts,1); pathGamma = zeros(totalPts,1);
    segTimes = zeros(totalSeg,1); segModes = zeros(totalSeg,1); segStages = zeros(totalSeg,1);
    segPass = nan(totalSeg,1); segSortie = zeros(totalSeg,1); segPowerW = zeros(totalSeg,1);

    pCur = 1; sCur = 1;
    for i = 1:chunkCount
        pts_i = missionChunks.pts{i};
        roll_i = missionChunks.roll{i};
        gamma_i = missionChunks.gamma{i};
        segT_i = missionChunks.segTimes{i};
        segM_i = missionChunks.segModes{i};
        segSg_i = missionChunks.segStages{i};
        segP_i = missionChunks.segPass{i};
        segSo_i = missionChunks.segSortie{i};
        segPw_i = missionChunks.segPowerW{i};

        if i > 1
            pts_i = pts_i(2:end,:);
            roll_i = roll_i(2:end);
            gamma_i = gamma_i(2:end);
        end

        pIdx = pCur:(pCur + size(pts_i,1) - 1);
        sIdx = sCur:(sCur + numel(segT_i) - 1);

        pathXYZ(pIdx,:) = pts_i;
        pathRoll(pIdx) = roll_i;
        pathGamma(pIdx)= gamma_i;

        segTimes(sIdx) = segT_i;
        segModes(sIdx) = segM_i;
        segStages(sIdx)= segSg_i;
        segPass(sIdx)  = segP_i;
        segSortie(sIdx)= segSo_i;
        segPowerW(sIdx)= segPw_i;

        pCur = pCur + size(pts_i,1);
        sCur = sCur + numel(segT_i);
    end
end

function heading = computeHeadingFromPath(pathXYZ)
    nPts = size(pathXYZ,1);
    heading = nan(nPts,1);
    for i = 1:nPts-1
        dP = pathXYZ(i+1,:) - pathXYZ(i,:);
        if norm(dP(1:2)) > 1e-12
            heading(i) = atan2(dP(1), dP(2));
        end
    end
    for i = 2:nPts
        if isnan(heading(i))
            heading(i) = heading(i-1);
        end
    end
    for i = nPts-1:-1:1
        if isnan(heading(i))
            heading(i) = heading(i+1);
        end
    end
    if all(isnan(heading))
        heading(:) = 0;
    end
    if isnan(heading(end))
        heading(end) = heading(find(~isnan(heading),1,'last'));
    end
end

function wrapTheta = wrapToPiLocal(theta)
    wrapTheta = atan2(sin(theta), cos(theta));
end

function [displayXYZ, displayTimeReal, displayRoll, displayGamma, ...
    displaySegModes, displaySegStages, displaySegPass, displaySegSortie] = ...
    buildDisplayMission(pathXYZ, pathTime, pathRoll, pathGamma, segModes, segStages, segPass, segSortie, ...
    dsSpray, dsTurn, dsTransit)

    codes = missionCodes();
    nPts = size(pathXYZ,1);
    nSeg = numel(segModes);
    if nPts == 0 || nSeg == 0
        displayXYZ = zeros(0,3); displayTimeReal = zeros(0,1);
        displayRoll = zeros(0,1); displayGamma = zeros(0,1);
        displaySegModes = zeros(0,1); displaySegStages = zeros(0,1);
        displaySegPass = nan(0,1); displaySegSortie = zeros(0,1);
        return
    end

    keep = false(nPts,1);
    keep(1) = true;
    distSinceKeep = 0;
    for i = 1:nSeg
        cat = stageToDisplayCategory(segStages(i));
        if i > 1
            boundary = segModes(i) ~= segModes(i-1) || segStages(i) ~= segStages(i-1) || ...
                       segSortie(i) ~= segSortie(i-1) || ~isequaln(segPass(i), segPass(i-1));
            if boundary
                keep(i) = true;
                distSinceKeep = 0;
            end
        end

        if cat == codes.displayCategory.spray
            dsTarget = dsSpray;
        elseif cat == codes.displayCategory.turn
            dsTarget = dsTurn;
        else
            dsTarget = dsTransit;
        end

        dSeg = norm(pathXYZ(i+1,:) - pathXYZ(i,:));
        distSinceKeep = distSinceKeep + dSeg;

        isBoundaryToNext = i == nSeg || segModes(i+1) ~= segModes(i) || ...
            segStages(i+1) ~= segStages(i) || segSortie(i+1) ~= segSortie(i) || ...
            ~isequaln(segPass(i+1), segPass(i));

        if distSinceKeep >= dsTarget - 1e-12 || isBoundaryToNext
            keep(i+1) = true;
            distSinceKeep = 0;
        end
    end

    keep(end) = true;
    idxKeep = find(keep);

    displayXYZ = pathXYZ(idxKeep,:);
    displayTimeReal = pathTime(idxKeep);
    displayRoll = pathRoll(idxKeep);
    displayGamma = pathGamma(idxKeep);
    displaySegModes = segModes(idxKeep(1:end-1));
    displaySegStages = segStages(idxKeep(1:end-1));
    displaySegPass = segPass(idxKeep(1:end-1));
    displaySegSortie = segSortie(idxKeep(1:end-1));
end

function [sliceXYZ, sliceTimeReal, sliceSegModes, sliceSegStages, sliceSegPass, sliceSegSortie] = ...
    extractDisplaySliceForSortie(pathXYZ, pathTimeReal, segModes, segStages, segPass, segSortie, sortieNumber)

    idxSeg = find(segSortie == sortieNumber);
    if isempty(idxSeg)
        sliceXYZ = zeros(0,3); sliceTimeReal = zeros(0,1);
        sliceSegModes = zeros(0,1); sliceSegStages = zeros(0,1);
        sliceSegPass = nan(0,1); sliceSegSortie = zeros(0,1);
        return
    end
    firstSeg = idxSeg(1); lastSeg = idxSeg(end);
    sliceXYZ = pathXYZ(firstSeg:lastSeg+1,:);
    sliceTimeReal = pathTimeReal(firstSeg:lastSeg+1);
    sliceSegModes = segModes(firstSeg:lastSeg);
    sliceSegStages = segStages(firstSeg:lastSeg);
    sliceSegPass = segPass(firstSeg:lastSeg);
    sliceSegSortie = segSortie(firstSeg:lastSeg);
end

function pack = buildPlaybackPackSimple(name, pathXYZ, pathTimeReal, segModes, segStages, segPass, segSortie, ...
    compressTurnaroundInPlayback, turnaroundPlaybackTime_s, targetPlaybackDuration_s, playbackFPS, stateInterp)

    codes = missionCodes();
    pack.name = name;
    pack.pathXYZ = pathXYZ;
    pack.pathTimeReal = pathTimeReal;
    pack.segModes = segModes;
    pack.segStages = segStages;
    pack.segPass = segPass;
    pack.segSortie = segSortie;

    if isempty(pathXYZ) || numel(pathTimeReal) < 2
        pack.visualDuration = 0;
        pack.nFrames = 1;
        pack.frameVisualTime = 0;
        pack.frameRealTime = 0;
        pack.frameX = 0; pack.frameY = 0; pack.frameZ = 0;
        pack.frameRollDeg = 0; pack.frameGammaDeg = 0; pack.frameHeadingDeg = 0;
        pack.frameBatteryPct = 0; pack.framePayloadL = 0; pack.frameLitres = 0;
        pack.frameAreaHa = 0; pack.frameWeightKg = 0;
        pack.frameMode = 0; pack.frameStage = 0; pack.framePass = NaN; pack.frameSortie = 0;
        pack.frameDisplayCat = 3;
        return
    end

    segTimesReal = diff(pathTimeReal);
    visualSegTimes = segTimesReal;
    if compressTurnaroundInPlayback
        visualSegTimes(segStages == codes.stage.turnaroundPause) = max(turnaroundPlaybackTime_s, eps);
    end
    visualPathTime = [0; cumsum(visualSegTimes)];
    visualDuration = visualPathTime(end);
    nFrames = max(2, round(targetPlaybackDuration_s * playbackFPS));
    frameVisualTime = linspace(0, visualDuration, nFrames)';

    FVisToReal = makeSafeInterpolant(visualPathTime, pathTimeReal, 'linear');
    frameRealTime = FVisToReal(frameVisualTime);

    pack.visualDuration = visualDuration;
    pack.nFrames = nFrames;
    pack.frameVisualTime = frameVisualTime;
    pack.frameRealTime = frameRealTime;
    pack.frameX = stateInterp.Fx(frameRealTime);
    pack.frameY = stateInterp.Fy(frameRealTime);
    pack.frameZ = stateInterp.Fz(frameRealTime);
    pack.frameRollDeg = rad2deg(stateInterp.FRoll(frameRealTime));
    pack.frameGammaDeg = rad2deg(stateInterp.FGamma(frameRealTime));
    pack.frameHeadingDeg = rad2deg(wrapToPiLocal(stateInterp.FHeading(frameRealTime)));
    pack.frameBatteryPct = stateInterp.FBatteryPct(frameRealTime);
    pack.framePayloadL = stateInterp.FPayloadRem(frameRealTime);
    pack.frameLitres = stateInterp.FTotalLitres(frameRealTime);
    pack.frameAreaHa = stateInterp.FAreaSprayed(frameRealTime);
    pack.frameWeightKg = stateInterp.FWeightKg(frameRealTime);

    pack.frameMode = zeros(nFrames,1);
    pack.frameStage = zeros(nFrames,1);
    pack.framePass = nan(nFrames,1);
    pack.frameSortie = zeros(nFrames,1);
    pack.frameDisplayCat = zeros(nFrames,1);

    for i = 1:nFrames
        idx = findSegmentIndex(frameRealTime(i), pathTimeReal);
        pack.frameMode(i) = segModes(idx);
        pack.frameStage(i)= segStages(idx);
        pack.framePass(i) = segPass(idx);
        pack.frameSortie(i)= segSortie(idx);
        pack.frameDisplayCat(i) = stageToDisplayCategory(segStages(idx));
    end
end

function terrainModel = getOptionalTerrainModelForSimulation(planningData)
    terrainModel = [];
    if ~isfield(planningData, 'geodata') || ~isstruct(planningData.geodata) || ...
            ~isfield(planningData.geodata, 'terrainModel') || ~isstruct(planningData.geodata.terrainModel)
        return
    end
    if isfield(planningData.geodata.terrainModel, 'pointsXY') && ...
            isfield(planningData.geodata.terrainModel, 'elevationM') && ...
            ~isempty(planningData.geodata.terrainModel.pointsXY) && ...
            ~isempty(planningData.geodata.terrainModel.elevationM)
        terrainModel = planningData.geodata.terrainModel;
    end
end

function value = getOptionalMissionScalar(missionCfg, fieldName, defaultValue)
    value = defaultValue;
    if isfield(missionCfg, fieldName) && isscalar(missionCfg.(fieldName)) && ...
            isfinite(missionCfg.(fieldName))
        value = missionCfg.(fieldName);
    end
end

function turnaroundTime_s = selectTurnaroundTime(stopReason, defaultTurnaroundTime_s, ...
    payloadTurnaroundTime_s, batteryTurnaroundTime_s)
    switch lower(stopReason)
        case 'payload'
            turnaroundTime_s = payloadTurnaroundTime_s;
        case 'battery'
            turnaroundTime_s = batteryTurnaroundTime_s;
        otherwise
            turnaroundTime_s = defaultTurnaroundTime_s;
    end
end

function elevation_m = sampleTerrainElevationForSimulation(terrainModel, xy)
    if isempty(terrainModel)
        elevation_m = zeros(size(xy,1), 1);
        return
    end

    interpolant = scatteredInterpolant(terrainModel.pointsXY(:,1), terrainModel.pointsXY(:,2), ...
        terrainModel.elevationM(:), 'natural', 'nearest');
    elevation_m = interpolant(xy(:,1), xy(:,2));
end

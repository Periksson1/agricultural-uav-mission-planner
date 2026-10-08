%% Compare current planned mission against a DJI Agras T50
% Run runMission first so missionCfg,
% planningData, and simulationData exist in the workspace.
%
% DJI Agras T50 source assumptions are from DJI public specification pages:
% - 40 L spray tank / 40 kg spray payload
% - 52.22 V, 30 Ah DB1560 battery
% - 10 m/s maximum operating flight speed
% - 4-11 m effective spray width
% - 16 L/min maximum flow with two sprinklers
% - 9-12 min battery charge time
% - up to 21 ha/hr field operation rate

if ~exist('missionCfg', 'var') || ~exist('planningData', 'var') || ~exist('simulationData', 'var')
    error(['Run runMission before this comparison script. ' ...
        'It needs missionCfg, planningData, and simulationData in the workspace.']);
end

agrasOptions = struct();

% Main editable assumptions. These are intentionally kept at the top.
agrasOptions.sprayWidth_m = missionCfg.sprayWidth;       % same lane output as our drone
agrasOptions.sprayPayload_L = 40.0;
agrasOptions.spraySpeed_mps = 10.0;
agrasOptions.transferSpeed_mps = 10.0;
agrasOptions.batteryReturnThreshold_pct = 20.0;
agrasOptions.turnaroundTime_s = 10.5 * 60;               % midpoint of DJI 9-12 min charge
agrasOptions.settleTimePerLane_s = 0.0;                  % multirotor can yaw/hover at lane ends
agrasOptions.batteryChargeCostPerReturn_GBP = missionCfg.batteryChargeCostPerReturn_GBP;
agrasOptions.maintenanceRatePerFlightHour_GBP = missionCfg.maintenanceRatePerFlightHour_GBP;

agrasSpec = defaultDJIAgrasT50Spec(missionCfg, planningData, agrasOptions);
ourResult = summariseOurDroneMission(missionCfg, simulationData);
try
    agrasResult = simulateAgrasT50SameMission(missionCfg, planningData, agrasSpec);
catch comparisonError
    if ~strcmp(comparisonError.identifier, 'agras:InfeasibleMission')
        rethrow(comparisonError);
    end
    agrasComparison = struct('assumptions',agrasSpec, 'ourDrone',ourResult, ...
        'djiAgrasT50',[], 'feasible',false, 'reason',comparisonError.message, 'table',table());
    fprintf('\nT50 comparison: infeasible under the current model assumptions.\n%s\n', comparisonError.message);
    fprintf('No T50 mission totals are reported. Inspect agrasComparison.reason or use a smaller/closer layout.\n');
    return
end

agrasComparison = struct();
agrasComparison.feasible = true;
agrasComparison.reason = '';
agrasComparison.assumptions = agrasSpec;
agrasComparison.ourDrone = ourResult;
agrasComparison.djiAgrasT50 = agrasResult;
agrasComparison.table = buildComparisonTable(ourResult, agrasResult);

fprintf('\n=== DJI AGRAS T50 SAME-MISSION COMPARISON ===\n');
fprintf('Agras path model                    = direct multirotor transfers between existing lane outputs\n');
fprintf('Agras spray width used              = %.2f m\n', agrasSpec.sprayWidth_m);
fprintf('Agras spray speed used              = %.2f m/s\n', agrasSpec.spraySpeed_mps);
fprintf('Agras transfer speed used           = %.2f m/s\n', agrasSpec.transferSpeed_mps);
fprintf('Agras battery energy                = %.3f kWh\n', agrasSpec.batteryEnergy_Wh / 1000);
fprintf('Agras battery reserve               = %.1f %%\n', agrasSpec.batteryReturnThreshold_pct);
fprintf('Agras assumed average power         = %.2f kW\n', agrasSpec.operatingPower_W / 1000);
fprintf('Agras turnaround                    = %.1f min\n', agrasSpec.turnaroundTime_s / 60);
fprintf('Agras flow required while spraying  = %.2f L/min\n', agrasResult.requiredFlow_L_min);
fprintf('Agras max flow limit                = %.2f L/min\n', agrasSpec.maxFlow_L_min);
fprintf('\n');
disp(agrasComparison.table);

if agrasResult.requiredFlow_L_min > agrasSpec.maxFlow_L_min + 1e-9
    warning(['Required T50 flow exceeds the published two-sprinkler max flow. ' ...
        'Reduce speed, reduce application rate, or model the optional four-sprinkler kit.']);
end

plotAgrasComparison(ourResult, agrasResult);
plotAgrasMissionPath(planningData, agrasResult);

%% -------------------- LOCAL FUNCTIONS --------------------
function spec = defaultDJIAgrasT50Spec(missionCfg, planningData, options)
    spec = struct();
    spec.name = 'DJI Agras T50';
    spec.source = 'DJI public specifications';

    spec.emptyMassInclBattery_kg = 52.0;
    spec.maxTakeoffMassSpray_kg = 92.0;
    spec.sprayPayload_L = getOption(options, 'sprayPayload_L', 40.0);
    spec.sprayTankVolume_L = 40.0;
    spec.batteryVoltage_V = 52.22;
    spec.batteryCapacity_Ah = 30.0;
    spec.batteryEnergy_Wh = spec.batteryVoltage_V * spec.batteryCapacity_Ah;
    spec.batteryReturnThreshold_pct = getOption(options, 'batteryReturnThreshold_pct', 20.0);
    spec.reserveBattery_Wh = spec.batteryEnergy_Wh * spec.batteryReturnThreshold_pct / 100;

    spec.maxOperatingSpeed_mps = 10.0;
    spec.spraySpeed_mps = min(getOption(options, 'spraySpeed_mps', 10.0), spec.maxOperatingSpeed_mps);
    spec.transferSpeed_mps = min(getOption(options, 'transferSpeed_mps', 10.0), spec.maxOperatingSpeed_mps);
    spec.publishedFieldCapacity_ha_per_hr = 21.0;
    spec.effectiveSprayWidthRange_m = [4.0 11.0];
    spec.sprayWidth_m = getOption(options, 'sprayWidth_m', missionCfg.sprayWidth);
    spec.maxFlow_L_min = 16.0;
    spec.optionalFourSprinklerMaxFlow_L_min = 24.0;
    spec.turnaroundTime_s = getOption(options, 'turnaroundTime_s', 10.5 * 60);
    spec.settleTimePerLane_s = getOption(options, 'settleTimePerLane_s', 0.0);
    spec.batteryChargeCostPerReturn_GBP = getOption(options, 'batteryChargeCostPerReturn_GBP', 1.0);
    spec.maintenanceRatePerFlightHour_GBP = getOption(options, 'maintenanceRatePerFlightHour_GBP', 20.0);

    payloadArea_ha = spec.sprayPayload_L / missionCfg.applicationRate;
    payloadTimeAtPublishedRate_s = payloadArea_ha / spec.publishedFieldCapacity_ha_per_hr * 3600;
    usableBattery_Wh = spec.batteryEnergy_Wh - spec.reserveBattery_Wh;
    spec.operatingPower_W = usableBattery_Wh / max(payloadTimeAtPublishedRate_s / 3600, eps);
    spec.powerAssumption = ['Average operating power inferred from DJI 21 ha/hr field rate, ' ...
        '40 L tank, current application rate, and the selected battery reserve.'];

    spec.litresPerSprayMetre = planningData.litresPerSprayMetre;
    spec.coverageWidthForPayload_m = planningData.coverageWidthForPayload;
end

function result = simulateAgrasT50SameMission(missionCfg, planningData, spec)
    lanes = planningData.laneMissionStructs(:);
    if isempty(lanes)
        error('planningData.laneMissionStructs is empty; no spray lanes are available to compare.');
    end

    gsXYZ = planningData.groundStationGroundXYZ;
    litresPerM = planningData.litresPerSprayMetre;
    tol = 1e-8;

    pathXYZ = zeros(0,3);
    pathTime = 0;
    segmentTimes = zeros(0,1);
    segmentModes = cell(0,1);
    segmentDistances = zeros(0,1);
    segmentEnergy_Wh = zeros(0,1);
    segmentSortie = zeros(0,1);

    sortieLog = repmat(makeAgrasSortieLog(), 0, 1);
    payloadInterruptPoints = zeros(0,3);
    batteryInterruptPoints = zeros(0,3);

    missionTime_s = 0;
    laneIdx = 1;
    laneProgress_m = 0;
    currentPos = gsXYZ;
    sortieNum = 0;
    sprayedDistance_m = 0;
    totalLitresUsed_L = 0;
    payloadTriggerCount = 0;
    batteryTriggerCount = 0;

    while laneIdx <= numel(lanes)
        sortieNum = sortieNum + 1;
        sortieStartTime_s = missionTime_s;
        batteryWh = spec.batteryEnergy_Wh;
        payloadL = spec.sprayPayload_L;
        reason = 'complete';
        madeProgressThisSortie = false;
        sprayedDistanceAtSortieStart_m = sprayedDistance_m;

        if norm(currentPos - gsXYZ) > tol
            currentPos = gsXYZ;
        end

        while laneIdx <= numel(lanes)
            lane = lanes(laneIdx);
            laneLength = max(lane.sprayLength_m, 0);
            sprayStart = pointAlongLane(lane, laneProgress_m);

            if norm(currentPos - sprayStart) > tol
                [canTransfer, transferEnergyWh, transferTime_s] = canCompleteTransferAndReturn( ...
                    currentPos, sprayStart, gsXYZ, batteryWh, spec);
                if ~canTransfer
                    if ~madeProgressThisSortie
                        error('agras:InfeasibleMission', ...
                            'The modelled T50 cannot reach lane %d and return to this ground station with its required reserve.', laneIdx);
                    end
                    reason = 'battery';
                    batteryTriggerCount = batteryTriggerCount + 1;
                    batteryInterruptPoints(end+1,:) = currentPos; %#ok<AGROW>
                    break
                end

                [pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, segmentSortie] = ...
                    appendAgrasSegment(pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, ...
                    segmentSortie, currentPos, sprayStart, transferTime_s, 'transfer', transferEnergyWh, sortieNum);
                missionTime_s = missionTime_s + transferTime_s;
                pathTime(end+1,1) = missionTime_s; %#ok<AGROW>
                batteryWh = batteryWh - transferEnergyWh;
                currentPos = sprayStart;
                madeProgressThisSortie = true;
            end

            remainingLane_m = max(laneLength - laneProgress_m, 0);
            if remainingLane_m <= tol
                laneIdx = laneIdx + 1;
                laneProgress_m = 0;
                continue
            end

            payloadLimitedDistance_m = payloadL / max(litresPerM, eps);
            batteryLimitedDistance_m = maxSprayDistanceBeforeBatteryReturn( ...
                currentPos, lane, laneProgress_m, remainingLane_m, gsXYZ, batteryWh, spec);

            sprayDistance_m = min([remainingLane_m, payloadLimitedDistance_m, batteryLimitedDistance_m]);
            if sprayDistance_m <= tol
                if payloadLimitedDistance_m <= tol
                    reason = 'payload';
                    payloadTriggerCount = payloadTriggerCount + 1;
                    payloadInterruptPoints(end+1,:) = currentPos; %#ok<AGROW>
                else
                    reason = 'battery';
                    batteryTriggerCount = batteryTriggerCount + 1;
                    batteryInterruptPoints(end+1,:) = currentPos; %#ok<AGROW>
                end
                break
            end

            nextPos = pointAlongLane(lane, laneProgress_m + sprayDistance_m);
            sprayTime_s = sprayDistance_m / spec.spraySpeed_mps;
            sprayEnergyWh = spec.operatingPower_W * sprayTime_s / 3600;
            litresUsed = sprayDistance_m * litresPerM;

            [pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, segmentSortie] = ...
                appendAgrasSegment(pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, ...
                segmentSortie, currentPos, nextPos, sprayTime_s, 'spray', sprayEnergyWh, sortieNum);
            missionTime_s = missionTime_s + sprayTime_s;
            pathTime(end+1,1) = missionTime_s; %#ok<AGROW>

            currentPos = nextPos;
            laneProgress_m = laneProgress_m + sprayDistance_m;
            sprayedDistance_m = sprayedDistance_m + sprayDistance_m;
            totalLitresUsed_L = totalLitresUsed_L + litresUsed;
            payloadL = max(payloadL - litresUsed, 0);
            batteryWh = max(batteryWh - sprayEnergyWh, 0);
            madeProgressThisSortie = true;

            if laneProgress_m >= laneLength - tol
                laneIdx = laneIdx + 1;
                laneProgress_m = 0;
                if spec.settleTimePerLane_s > 0 && laneIdx <= numel(lanes)
                    settleEnergyWh = spec.operatingPower_W * spec.settleTimePerLane_s / 3600;
                    [pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, segmentSortie] = ...
                        appendAgrasSegment(pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, ...
                        segmentSortie, currentPos, currentPos, spec.settleTimePerLane_s, ...
                        'settle', settleEnergyWh, sortieNum);
                    missionTime_s = missionTime_s + spec.settleTimePerLane_s;
                    pathTime(end+1,1) = missionTime_s; %#ok<AGROW>
                    batteryWh = max(batteryWh - settleEnergyWh, 0);
                end
                if laneIdx <= numel(lanes) && payloadL <= tol
                    reason = 'payload';
                    payloadTriggerCount = payloadTriggerCount + 1;
                    payloadInterruptPoints(end+1,:) = currentPos; %#ok<AGROW>
                    break
                end
            else
                if payloadLimitedDistance_m <= batteryLimitedDistance_m + tol
                    reason = 'payload';
                    payloadTriggerCount = payloadTriggerCount + 1;
                    payloadInterruptPoints(end+1,:) = currentPos; %#ok<AGROW>
                else
                    reason = 'battery';
                    batteryTriggerCount = batteryTriggerCount + 1;
                    batteryInterruptPoints(end+1,:) = currentPos; %#ok<AGROW>
                end
                break
            end
        end

        if laneIdx <= numel(lanes) && sprayedDistance_m <= sprayedDistanceAtSortieStart_m + tol
            error('agras:InfeasibleMission', ...
                'The modelled T50 cannot make further spray progress at lane %d with its required return reserve.', laneIdx);
        end
        returnDistance_m = norm(currentPos - gsXYZ);
        returnTime_s = returnDistance_m / spec.transferSpeed_mps;
        returnEnergyWh = spec.operatingPower_W * returnTime_s / 3600;
        [pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, segmentSortie] = ...
            appendAgrasSegment(pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, ...
            segmentSortie, currentPos, gsXYZ, returnTime_s, 'return', returnEnergyWh, sortieNum);
        missionTime_s = missionTime_s + returnTime_s;
        pathTime(end+1,1) = missionTime_s; %#ok<AGROW>
        batteryWh = max(batteryWh - returnEnergyWh, 0);
        currentPos = gsXYZ;

        sortieLog(end+1,1) = makeAgrasSortieLog(); %#ok<AGROW>
        sortieLog(end).sortieNumber = sortieNum;
        sortieLog(end).startMissionT = sortieStartTime_s;
        sortieLog(end).interruptReason = reason;
        sortieLog(end).arrivalGST = missionTime_s;
        sortieLog(end).batteryPctAtArrival = 100 * batteryWh / spec.batteryEnergy_Wh;
        sortieLog(end).payloadRemainingAtReturn_L = payloadL;
        sortieLog(end).completedLaneIndex = min(laneIdx, numel(lanes));
        sortieLog(end).laneProgress_m = laneProgress_m;

        if laneIdx <= numel(lanes)
            missionTime_s = missionTime_s + spec.turnaroundTime_s;
            pathTime(end+1,1) = missionTime_s; %#ok<AGROW>
            sortieLog(end).readyAgainT = missionTime_s;
        else
            sortieLog(end).completeT = missionTime_s;
        end
    end

    sprayedArea_ha = sprayedDistance_m * planningData.coverageWidthForPayload / 10000;
    totalEnergy_kWh = sum(segmentEnergy_Wh) / 1000;
    sprayMask = strcmp(segmentModes, 'spray');
    transferMask = strcmp(segmentModes, 'transfer') | strcmp(segmentModes, 'return');
    turnaroundTime_s = max(0, missionTime_s - sum(segmentTimes));
    sprayTime_s = sum(segmentTimes(sprayMask));
    transferTime_s = sum(segmentTimes(transferMask));
    returnCount = sortieNum;
    batteryChargeCount = max(returnCount - 1, 0);
    flightTime_s = missionTime_s - turnaroundTime_s;

    result = struct();
    result.name = spec.name;
    result.nSorties = sortieNum;
    result.returnCount = returnCount;
    result.payloadTriggerCount = payloadTriggerCount;
    result.batteryTriggerCount = batteryTriggerCount;
    result.sprayedArea_ha = sprayedArea_ha;
    result.unsprayedArea_ha = max(planningData.totalFieldArea_ha - sprayedArea_ha, 0);
    result.totalDistance_m = sum(segmentDistances);
    result.sprayDistance_m = sprayedDistance_m;
    result.transferDistance_m = sum(segmentDistances(transferMask));
    result.totalTime_s = missionTime_s;
    result.flightTime_s = flightTime_s;
    result.sprayTime_s = sprayTime_s;
    result.transferTime_s = transferTime_s;
    result.turnaroundTime_s = turnaroundTime_s;
    result.totalPayloadUsed_L = totalLitresUsed_L;
    result.missionEnergyDemand_kWh = totalEnergy_kWh;
    result.energyPerHa_kWh = totalEnergy_kWh / max(sprayedArea_ha, eps);
    result.overallFieldCapacity_ha_per_hr = sprayedArea_ha / max(missionTime_s / 3600, eps);
    result.sprayRate_ha_per_hr = sprayedArea_ha / max(sprayTime_s / 3600, eps);
    result.averageAreaPerSortie_ha = sprayedArea_ha / max(sortieNum, 1);
    result.requiredFlow_L_min = spec.spraySpeed_mps * planningData.litresPerSprayMetre * 60;
    result.finalBatteryPct = sortieLog(end).batteryPctAtArrival;
    result.finalPayload_L = sortieLog(end).payloadRemainingAtReturn_L;
    result.totalRevenue_GBP = missionCfg.revenuePerHa_GBP * sprayedArea_ha;
    result.missionLabourBillableHours = ceil((missionTime_s / 3600) * 2) / 2;
    result.labourCost_GBP = missionCfg.workerRatePerHour_GBP * missionCfg.numberOfWorkers * result.missionLabourBillableHours;
    result.batteryChargingCost_GBP = spec.batteryChargeCostPerReturn_GBP * batteryChargeCount;
    result.maintenanceCost_GBP = spec.maintenanceRatePerFlightHour_GBP * (flightTime_s / 3600);
    result.totalCost_GBP = missionCfg.transportCost_GBP + result.labourCost_GBP + ...
        result.batteryChargingCost_GBP + result.maintenanceCost_GBP;
    result.netProfit_GBP = result.totalRevenue_GBP - result.totalCost_GBP;
    result.pathXYZ = pathXYZ;
    result.pathTime_s = pathTime(:);
    result.segmentTimes_s = segmentTimes;
    result.segmentModes = segmentModes;
    result.segmentDistances_m = segmentDistances;
    result.segmentEnergy_Wh = segmentEnergy_Wh;
    result.segmentSortie = segmentSortie;
    result.sortieLog = sortieLog;
    result.payloadInterruptPoints = payloadInterruptPoints;
    result.batteryInterruptPoints = batteryInterruptPoints;
end

function result = summariseOurDroneMission(missionCfg, simulationData)
    codes = missionCodes();
    segmentDistances = vecnorm(diff(simulationData.pathXYZ,1,1),2,2);
    sprayMask = simulationData.segmentStages == codes.stage.sprayLeg;
    turnaroundTime_s = simulationData.totalTime * simulationData.turnaroundShare_pct / 100;
    flightTime_s = simulationData.totalTime - turnaroundTime_s;

    result = struct();
    result.name = 'Our drone';
    result.nSorties = simulationData.nSorties;
    result.returnCount = simulationData.returnCount;
    result.payloadTriggerCount = sum(strcmp({simulationData.sortieLog.interruptReason}, 'payload'));
    result.batteryTriggerCount = sum(strcmp({simulationData.sortieLog.interruptReason}, 'battery'));
    result.sprayedArea_ha = simulationData.sprayedArea_ha(end);
    result.unsprayedArea_ha = simulationData.areaNotSprayed_ha;
    result.totalDistance_m = simulationData.totalDistance;
    result.sprayDistance_m = sum(segmentDistances(sprayMask));
    result.transferDistance_m = max(simulationData.totalDistance - result.sprayDistance_m, 0);
    result.totalTime_s = simulationData.totalTime;
    result.flightTime_s = flightTime_s;
    result.sprayTime_s = sum(simulationData.segmentTimes(sprayMask));
    result.transferTime_s = max(flightTime_s - result.sprayTime_s, 0);
    result.turnaroundTime_s = turnaroundTime_s;
    result.totalPayloadUsed_L = simulationData.totalPayloadUsed_L;
    result.missionEnergyDemand_kWh = simulationData.missionEnergyDemand_kWh;
    result.energyPerHa_kWh = simulationData.energyPerHa_kWh;
    result.overallFieldCapacity_ha_per_hr = simulationData.overallFieldCapacity_ha_per_hr;
    result.sprayRate_ha_per_hr = simulationData.sprayRate_ha_per_hr;
    result.averageAreaPerSortie_ha = simulationData.averageAreaPerSortie_ha;
    result.requiredFlow_L_min = missionCfg.uavAirSpeed * missionCfg.applicationRate / 10000 * missionCfg.sprayWidth * 60;
    result.finalBatteryPct = simulationData.finalBatteryPct;
    result.finalPayload_L = simulationData.payloadRemaining_L(end);
    result.totalRevenue_GBP = simulationData.totalRevenue_GBP;
    result.missionLabourBillableHours = simulationData.missionLabourBillableHours;
    result.labourCost_GBP = simulationData.labourCost_GBP;
    result.batteryChargingCost_GBP = simulationData.batteryChargingCost_GBP;
    result.maintenanceCost_GBP = simulationData.maintenanceCost_GBP;
    result.totalCost_GBP = simulationData.totalCost_GBP;
    result.netProfit_GBP = simulationData.netProfit_GBP;
end

function comparisonTable = buildComparisonTable(ourResult, agrasResult)
    metric = { ...
        'Sprayed area'; ...
        'Total mission time'; ...
        'Flight time'; ...
        'Turnaround time'; ...
        'Number of sorties'; ...
        'Payload-triggered returns'; ...
        'Battery-triggered returns'; ...
        'Total distance'; ...
        'Spray distance'; ...
        'Transit/return distance'; ...
        'Overall field capacity'; ...
        'Spray-only field rate'; ...
        'Payload used'; ...
        'Mission energy demand'; ...
        'Energy per hectare'; ...
        'Average area per sortie'; ...
        'Required spray flow'; ...
        'Total cost'; ...
        'Net profit'};
    unit = { ...
        'ha'; 'min'; 'min'; 'min'; 'count'; 'count'; 'count'; 'km'; 'km'; 'km'; ...
        'ha/hr'; 'ha/hr'; 'L'; 'kWh'; 'kWh/ha'; 'ha/sortie'; 'L/min'; 'GBP'; 'GBP'};
    our = [ ...
        ourResult.sprayedArea_ha; ...
        ourResult.totalTime_s / 60; ...
        ourResult.flightTime_s / 60; ...
        ourResult.turnaroundTime_s / 60; ...
        ourResult.nSorties; ...
        ourResult.payloadTriggerCount; ...
        ourResult.batteryTriggerCount; ...
        ourResult.totalDistance_m / 1000; ...
        ourResult.sprayDistance_m / 1000; ...
        ourResult.transferDistance_m / 1000; ...
        ourResult.overallFieldCapacity_ha_per_hr; ...
        ourResult.sprayRate_ha_per_hr; ...
        ourResult.totalPayloadUsed_L; ...
        ourResult.missionEnergyDemand_kWh; ...
        ourResult.energyPerHa_kWh; ...
        ourResult.averageAreaPerSortie_ha; ...
        ourResult.requiredFlow_L_min; ...
        ourResult.totalCost_GBP; ...
        ourResult.netProfit_GBP];
    agras = [ ...
        agrasResult.sprayedArea_ha; ...
        agrasResult.totalTime_s / 60; ...
        agrasResult.flightTime_s / 60; ...
        agrasResult.turnaroundTime_s / 60; ...
        agrasResult.nSorties; ...
        agrasResult.payloadTriggerCount; ...
        agrasResult.batteryTriggerCount; ...
        agrasResult.totalDistance_m / 1000; ...
        agrasResult.sprayDistance_m / 1000; ...
        agrasResult.transferDistance_m / 1000; ...
        agrasResult.overallFieldCapacity_ha_per_hr; ...
        agrasResult.sprayRate_ha_per_hr; ...
        agrasResult.totalPayloadUsed_L; ...
        agrasResult.missionEnergyDemand_kWh; ...
        agrasResult.energyPerHa_kWh; ...
        agrasResult.averageAreaPerSortie_ha; ...
        agrasResult.requiredFlow_L_min; ...
        agrasResult.totalCost_GBP; ...
        agrasResult.netProfit_GBP];

    difference = agras - our;
    ratio = agras ./ max(abs(our), eps);
    comparisonTable = table(metric, unit, our, agras, difference, ratio, ...
        'VariableNames', {'Metric','Unit','OurDrone','DJIAgrasT50','AgrasMinusOur','AgrasOverOur'});
end

function plotAgrasComparison(ourResult, agrasResult)
    figure('Name', 'DJI Agras T50 Same-Mission Comparison', 'Color', 'w');
    tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    labels = categorical({'Our drone','DJI Agras T50'});
    labels = reordercats(labels, {'Our drone','DJI Agras T50'});

    nexttile;
    bar(labels, [ourResult.totalTime_s, agrasResult.totalTime_s] / 60);
    ylabel('Mission time [min]');
    title('Total Time');
    grid on;

    nexttile;
    bar(labels, [ourResult.overallFieldCapacity_ha_per_hr, agrasResult.overallFieldCapacity_ha_per_hr]);
    ylabel('ha/hr');
    title('Overall Field Capacity');
    grid on;

    nexttile;
    bar(labels, [ourResult.nSorties, agrasResult.nSorties]);
    ylabel('Sorties');
    title('Sortie Count');
    grid on;

    nexttile;
    bar(labels, [ourResult.totalDistance_m, agrasResult.totalDistance_m] / 1000);
    ylabel('Distance [km]');
    title('Path Length');
    grid on;
end

function plotAgrasMissionPath(planningData, agrasResult)
    figure('Name', 'DJI Agras T50 Mission Path and Return-to-Base Events', 'Color', 'w');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    axMap = nexttile;
    hold(axMap, 'on');
    grid(axMap, 'on');
    axis(axMap, 'equal');
    xlabel(axMap, 'East [m]');
    ylabel(axMap, 'North [m]');
    title(axMap, 'Agras T50 Same-Mission Path');

    plotFieldBoundaries(axMap, planningData);
    plot(axMap, planningData.groundStationGroundXYZ(1), planningData.groundStationGroundXYZ(2), ...
        'p', 'MarkerSize', 14, 'MarkerFaceColor', [1.00 0.78 0.10], ...
        'MarkerEdgeColor', [0.35 0.24 0.00], 'DisplayName', 'Ground station');

    pathXYZ = agrasResult.pathXYZ;
    modes = agrasResult.segmentModes;
    sorties = agrasResult.segmentSortie;
    nSeg = numel(modes);
    modeStyles = agrasMissionModeStyles();
    usedModes = containers.Map();

    for iSeg = 1:nSeg
        p0 = pathXYZ(iSeg, :);
        p1 = pathXYZ(iSeg+1, :);
        modeName = modes{iSeg};
        style = modeStyles.(modeName);
        displayName = style.displayName;
        if isKey(usedModes, modeName)
            displayName = '';
        else
            usedModes(modeName) = true;
        end

        segmentHandle = plot(axMap, [p0(1) p1(1)], [p0(2) p1(2)], ...
            'Color', style.color, 'LineStyle', style.lineStyle, ...
            'LineWidth', style.lineWidth, 'DisplayName', displayName);
        if isempty(displayName), set(segmentHandle,'HandleVisibility','off'); end

        if strcmp(modeName, 'return')
            label = sprintf('RTB S%d', sorties(iSeg));
            text(axMap, p0(1), p0(2), ['  ' label], ...
                'Color', style.color, 'FontWeight', 'bold', 'FontSize', 8, ...
                'Clipping', 'on');
        end
    end

    if ~isempty(agrasResult.payloadInterruptPoints)
        plot(axMap, agrasResult.payloadInterruptPoints(:,1), agrasResult.payloadInterruptPoints(:,2), ...
            'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.20 0.20], ...
            'MarkerEdgeColor', 'w', 'DisplayName', 'Payload RTB trigger');
    end
    if ~isempty(agrasResult.batteryInterruptPoints)
        plot(axMap, agrasResult.batteryInterruptPoints(:,1), agrasResult.batteryInterruptPoints(:,2), ...
            's', 'MarkerSize', 8, 'MarkerFaceColor', [0.20 0.30 0.85], ...
            'MarkerEdgeColor', 'w', 'DisplayName', 'Battery RTB trigger');
    end

    if size(pathXYZ, 1) > 1
        plot(axMap, pathXYZ(1,1), pathXYZ(1,2), ...
            '^', 'MarkerSize', 9, 'MarkerFaceColor', [0.10 0.45 0.15], ...
            'MarkerEdgeColor', 'w', 'DisplayName', 'Mission start');
        plot(axMap, pathXYZ(end,1), pathXYZ(end,2), ...
            'v', 'MarkerSize', 9, 'MarkerFaceColor', [0.20 0.20 0.20], ...
            'MarkerEdgeColor', 'w', 'DisplayName', 'Mission end');
    end

    legend(axMap, 'Location', 'bestoutside');

    axTimeline = nexttile;
    hold(axTimeline, 'on');
    grid(axTimeline, 'on');
    title(axTimeline, 'Sortie Timeline');
    xlabel(axTimeline, 'Mission time [min]');
    ylabel(axTimeline, 'Sortie');

    plotAgrasSortieTimeline(axTimeline, agrasResult, modeStyles);
end

function plotFieldBoundaries(ax, planningData)
    if ~isfield(planningData, 'fields') || isempty(planningData.fields)
        return
    end

    fieldColor = [0.72 0.88 0.62];
    for iField = 1:numel(planningData.fields)
        fieldDef = planningData.fields(iField);
        loops = {};
        if isfield(fieldDef, 'displayBoundaryLoops') && ~isempty(fieldDef.displayBoundaryLoops)
            loops = fieldDef.displayBoundaryLoops;
        elseif isfield(fieldDef, 'boundaryXY') && ~isempty(fieldDef.boundaryXY)
            loops = {fieldDef.boundaryXY};
        end

        for iLoop = 1:numel(loops)
            loopXY = loops{iLoop};
            if size(loopXY, 1) >= 3
                fill(ax, loopXY(:,1), loopXY(:,2), fieldColor, ...
                    'FaceAlpha', 0.18, 'EdgeColor', [0.20 0.42 0.12], ...
                    'LineWidth', 1.2, 'DisplayName', '');
            end
        end
    end
end

function modeStyles = agrasMissionModeStyles()
    modeStyles = struct();
    modeStyles.spray = struct( ...
        'color', [0.00 0.48 0.18], ...
        'lineStyle', '-', ...
        'lineWidth', 2.6, ...
        'displayName', 'Spray');
    modeStyles.transfer = struct( ...
        'color', [0.10 0.38 0.88], ...
        'lineStyle', '-', ...
        'lineWidth', 1.6, ...
        'displayName', 'Direct transfer');
    modeStyles.return = struct( ...
        'color', [0.82 0.15 0.12], ...
        'lineStyle', '--', ...
        'lineWidth', 2.1, ...
        'displayName', 'Return to base');
    modeStyles.settle = struct( ...
        'color', [0.45 0.45 0.45], ...
        'lineStyle', ':', ...
        'lineWidth', 1.4, ...
        'displayName', 'Lane-end hover');
end

function plotAgrasSortieTimeline(ax, agrasResult, modeStyles)
    segStartTime = agrasResult.pathTime_s(1:end-1);
    segEndTime = agrasResult.pathTime_s(2:end);
    for iSeg = 1:numel(agrasResult.segmentModes)
        modeName = agrasResult.segmentModes{iSeg};
        sortieNum = agrasResult.segmentSortie(iSeg);
        style = modeStyles.(modeName);
        plot(ax, [segStartTime(iSeg) segEndTime(iSeg)] / 60, [sortieNum sortieNum], ...
            'Color', style.color, 'LineStyle', style.lineStyle, 'LineWidth', 5);
    end

    for iSortie = 1:numel(agrasResult.sortieLog)
        logItem = agrasResult.sortieLog(iSortie);
        if isfinite(logItem.arrivalGST)
            plot(ax, logItem.arrivalGST / 60, logItem.sortieNumber, ...
                'kx', 'MarkerSize', 8, 'LineWidth', 1.2);
            text(ax, logItem.arrivalGST / 60, logItem.sortieNumber, ...
                ['  ' logItem.interruptReason], 'FontSize', 8, 'Clipping', 'on');
        end
        if isfinite(logItem.readyAgainT)
            plot(ax, [logItem.arrivalGST logItem.readyAgainT] / 60, ...
                [logItem.sortieNumber logItem.sortieNumber], ...
                'Color', [0.25 0.25 0.25], 'LineStyle', ':', 'LineWidth', 2.5);
        end
    end

    if ~isempty(agrasResult.sortieLog)
        ylim(ax, [0.5 numel(agrasResult.sortieLog) + 0.5]);
        yticks(ax, 1:numel(agrasResult.sortieLog));
    end
end

function [canTransfer, energyWh, time_s] = canCompleteTransferAndReturn(startXYZ, endXYZ, gsXYZ, batteryWh, spec)
    dist_m = norm(endXYZ - startXYZ);
    time_s = dist_m / spec.transferSpeed_mps;
    energyWh = spec.operatingPower_W * time_s / 3600;
    returnEnergyWh = requiredReturnEnergyWh(endXYZ, gsXYZ, spec);
    canTransfer = (batteryWh - energyWh) >= (spec.reserveBattery_Wh + returnEnergyWh - 1e-9);
end

function maxDistance_m = maxSprayDistanceBeforeBatteryReturn(startXYZ, lane, laneProgress_m, remainingLane_m, gsXYZ, batteryWh, spec)
    endPos = pointAlongLane(lane, laneProgress_m + remainingLane_m);
    fullSprayTime_s = remainingLane_m / spec.spraySpeed_mps;
    fullSprayEnergyWh = spec.operatingPower_W * fullSprayTime_s / 3600;
    if batteryWh - fullSprayEnergyWh >= spec.reserveBattery_Wh + requiredReturnEnergyWh(endPos, gsXYZ, spec) - 1e-9
        maxDistance_m = remainingLane_m;
        return
    end

    if batteryWh <= spec.reserveBattery_Wh + requiredReturnEnergyWh(startXYZ, gsXYZ, spec)
        maxDistance_m = 0;
        return
    end

    lo = 0;
    hi = remainingLane_m;
    for i = 1:45
        mid = 0.5 * (lo + hi);
        pos = pointAlongLane(lane, laneProgress_m + mid);
        sprayEnergyWh = spec.operatingPower_W * (mid / spec.spraySpeed_mps) / 3600;
        marginWh = batteryWh - sprayEnergyWh - spec.reserveBattery_Wh - requiredReturnEnergyWh(pos, gsXYZ, spec);
        if marginWh >= 0
            lo = mid;
        else
            hi = mid;
        end
    end
    maxDistance_m = lo;
end

function energyWh = requiredReturnEnergyWh(posXYZ, gsXYZ, spec)
    returnTime_s = norm(posXYZ - gsXYZ) / spec.transferSpeed_mps;
    energyWh = spec.operatingPower_W * returnTime_s / 3600;
end

function pointXYZ = pointAlongLane(lane, distanceAlong_m)
    startXYZ = lane.startXYZ(:).';
    endXYZ = lane.endXYZ(:).';
    laneVec = endXYZ - startXYZ;
    laneLength = norm(laneVec);
    if laneLength <= eps
        pointXYZ = startXYZ;
        return
    end
    frac = min(max(distanceAlong_m / laneLength, 0), 1);
    pointXYZ = startXYZ + frac * laneVec;
end

function [pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, segmentSortie] = ...
    appendAgrasSegment(pathXYZ, segmentTimes, segmentModes, segmentDistances, segmentEnergy_Wh, ...
    segmentSortie, startXYZ, endXYZ, duration_s, modeName, energyWh, sortieNum)

    startXYZ = startXYZ(:).';
    endXYZ = endXYZ(:).';
    if isempty(pathXYZ)
        pathXYZ = startXYZ;
    elseif norm(pathXYZ(end,:) - startXYZ) > 1e-7
        pathXYZ(end+1,:) = startXYZ;
    end
    pathXYZ(end+1,:) = endXYZ;
    segmentTimes(end+1,1) = duration_s;
    segmentModes{end+1,1} = modeName;
    segmentDistances(end+1,1) = norm(endXYZ - startXYZ);
    segmentEnergy_Wh(end+1,1) = energyWh;
    segmentSortie(end+1,1) = sortieNum;
end

function sortieLog = makeAgrasSortieLog()
    sortieLog = struct( ...
        'sortieNumber', NaN, ...
        'startMissionT', NaN, ...
        'interruptReason', '', ...
        'arrivalGST', NaN, ...
        'readyAgainT', NaN, ...
        'completeT', NaN, ...
        'batteryPctAtArrival', NaN, ...
        'payloadRemainingAtReturn_L', NaN, ...
        'completedLaneIndex', NaN, ...
        'laneProgress_m', NaN);
end

function value = getOption(options, fieldName, defaultValue)
    if isstruct(options) && isfield(options, fieldName) && ~isempty(options.(fieldName))
        value = options.(fieldName);
    else
        value = defaultValue;
    end
end

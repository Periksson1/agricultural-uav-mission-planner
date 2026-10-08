function printMissionSummary(missionCfg, planningData, simulationData)
    fprintf('Coordinate system used              = x: East, y: North, z: Up\n');
    fprintf('Original fields drawn               = %d\n', planningData.originalFieldCount);
    fprintf('Combined spray blocks               = %d\n', planningData.combinedFieldCount);
    fprintf('Field visit order                   = %s\n', mat2str(planningData.fieldVisitOrder));
    for i = 1:planningData.fieldCount
        fprintf('%s\n', planningData.fieldSummaryLines{i});
    end
    fprintf('Ground station North                = %.2f m\n', missionCfg.groundStationNorth_m);
    fprintf('Ground station East                 = %.2f m\n', missionCfg.groundStationEast_m);
    fprintf('Ground station size                 = %.2f x %.2f m\n', missionCfg.groundStationSize, missionCfg.groundStationSize);
    fprintf('GS to first checkpoint              = %.2f m\n', planningData.groundStationToFirstCheckpointDistance);
    fprintf('VTOL altitude                       = %.2f m\n', missionCfg.vtolAltitude_m);
    fprintf('Field-entry / transfer height       = %.2f m\n', missionCfg.fieldTransferAltitude_m);
    fprintf('Pre-descent level segment           = %.2f m\n', missionCfg.preDescentLevelDistance_m);

    fprintf('\n--- MISSION SUMMARY ---\n');
    fprintf('Fields in plan                      = %d\n', planningData.fieldCount);
    fprintf('Total field area                    = %.3f ha\n', planningData.totalFieldArea_ha);
    fprintf('Total lane count                    = %d\n', planningData.totalLaneCount);
    fprintf('Lane-change count                   = %d\n', planningData.laneTurnCount);
    fprintf('Number of sorties                   = %d\n', simulationData.nSorties);
    fprintf('Returns to ground station           = %d\n', simulationData.returnCount);
    fprintf('Sprayed area                        = %.3f ha\n', simulationData.sprayedArea_ha(end));
    fprintf('Unsprayed area                      = %.3f ha\n', simulationData.areaNotSprayed_ha);
    fprintf('Total distance                      = %.2f m\n', simulationData.totalDistance);
    fprintf('Total mission time                  = %s\n', formatDurationHM(simulationData.totalTime));

    fprintf('\n--- PRODUCTIVITY ---\n');
    fprintf('Payload used                        = %.2f L\n', simulationData.totalPayloadUsed_L);
    fprintf('Time spent spraying                 = %.2f %%\n', simulationData.sprayTimeFraction_pct);
    fprintf('Non-productive share                = %.2f %%\n', simulationData.nonProductiveTime_pct);
    fprintf('Turnaround share                    = %.2f %%\n', simulationData.turnaroundShare_pct);
    fprintf('Transit + VTOL share                = %.2f %%\n', simulationData.transitShare_pct);
    fprintf('Inter-pass share                    = %.2f %%\n', simulationData.turnShare_pct);
    fprintf('Spray rate                          = %.3f ha/hr\n', simulationData.sprayRate_ha_per_hr);
    fprintf('Overall field capacity              = %.3f ha/hr\n', simulationData.overallFieldCapacity_ha_per_hr);
    fprintf('Average area per sortie             = %.3f ha\n', simulationData.averageAreaPerSortie_ha);

    fprintf('\n--- BATTERY / PAYLOAD ---\n');
    fprintf('Mission energy demand               = %.3f kWh\n', simulationData.missionEnergyDemand_kWh);
    fprintf('Energy per hectare                  = %.3f kWh/ha\n', simulationData.energyPerHa_kWh);
    fprintf('Cruise power                        = %.2f kW\n', missionCfg.cruisePowerDraw_kW);
    fprintf('Cruise climb power                  = %.2f kW\n', missionCfg.cruiseClimbPowerDraw_kW);
    fprintf('VTOL hover power                    = %.2f kW\n', missionCfg.vtolHoverPowerDraw_kW);
    fprintf('VTOL climb power                    = %.2f kW\n', missionCfg.vtolClimbPowerDraw_kW);
    fprintf('Final onboard payload               = %.2f L\n', simulationData.payloadRemaining_L(end));
    fprintf('Final onboard battery               = %.2f %%\n', simulationData.finalBatteryPct);
    fprintf('Final drone mass                    = %.2f kg\n', simulationData.finalWeightKg);
    fprintf('Average battery at RTB trigger      = %.2f %%\n', simulationData.avgBatteryPctAtRTBTrigger);
    fprintf('Average battery at GS arrival       = %.2f %%\n', simulationData.avgBatteryPctAtGSArrival);

    fprintf('\n--- ECONOMICS ---\n');
    fprintf('Revenue                             = %.2f GBP\n', simulationData.totalRevenue_GBP);
    fprintf('Transport cost                      = %.2f GBP\n', missionCfg.transportCost_GBP);
    fprintf('Labour billed hours                 = %.2f hr\n', simulationData.missionLabourBillableHours);
    fprintf('Labour cost                         = %.2f GBP\n', simulationData.labourCost_GBP);
    fprintf('Battery charging cost               = %.2f GBP\n', simulationData.batteryChargingCost_GBP);
    fprintf('Maintenance cost                    = %.2f GBP\n', simulationData.maintenanceCost_GBP);
    fprintf('Total cost                          = %.2f GBP\n', simulationData.totalCost_GBP);
    fprintf('Net profit                          = %.2f GBP\n', simulationData.netProfit_GBP);

    fprintf('\n--- COMMAND EXPORT ---\n');
    fprintf('sortieCommandExports(i) contains cruise-only exports for sortie i (no VTOL / turnaround).\n');
    fprintf('rollCmdTimeBySortie, rollCmdRadBySortie, rollCmdDegBySortie, rollCmdMatrixBySortie, rollCmdTSBySortie\n');
    fprintf('flightPathCmdTimeBySortie, flightPathCmdRadBySortie, flightPathCmdDegBySortie, flightPathCmdMatrixBySortie, flightPathCmdTSBySortie\n');
    fprintf('headingCmdTimeBySortie, headingCmdRadBySortie, headingCmdDegBySortie, headingCmdMatrixBySortie, headingCmdTSBySortie\n');
    fprintf('altitudeCmdTimeBySortie, altitudeCmdMBySortie, altitudeCmdMatrixBySortie, altitudeCmdTSBySortie\n');
    fprintf('initialHeadingAfterVTOLRadBySortie, initialHeadingAfterVTOLDegBySortie, initialHeightAfterVTOL_mBySortie\n');
    fprintf('initialEastAfterVTOL_mBySortie, initialNorthAfterVTOL_mBySortie, initialUpAfterVTOL_mBySortie\n');
    fprintf('InitialPosBySortie [North East Down], InitialHeadingRadBySortie, InitialHeadingDegBySortie\n');
    fprintf('sortieInitialConditionExports(i).InitialPosNED and .InitialHeadingRad replace the old eval-based exports.\n');
    fprintf('commandExportBundle stores the full export bundle in a single struct.\n');
    fprintf('missionExport.sorties(i) stores a cleaner Simulink-friendly export bundle per sortie.\n');
    fprintf('planningData.laneMissionStructs stores one struct per planned spray lane.\n');
    fprintf('upCmdTimeBySortie, upCmdMBySortie, upCmdMatrixBySortie, upCmdTSBySortie\n');
end

function out = formatDurationHM(seconds)
    seconds = max(seconds, 0);
    h = floor(seconds / 3600);
    m = round((seconds - 3600*h) / 60);
    if m == 60
        h = h + 1;
        m = 0;
    end
    if h > 0
        out = sprintf('%d h %d min', h, m);
    else
        out = sprintf('%d min', m);
    end
end

function [crossTrackError_m, crossTrackRate_mps, alongTrackRate_mps, ...
    activeLegIdxOut, switchFlag, distanceToWaypointB_m, closestPointNED, ...
    altitudeCmd_m] = ...
    crossTrackErrorABNED(vehiclePosNED, vehicleVelNED, waypointsNED, ...
    activeLegIdxIn, waypointRadius_m)
%#codegen
% crossTrackErrorABNED
% Simulink-friendly cross-track calculation with ArduPilot-style mission
% leg advance logic for a waypoint list in NED coordinates.
%
% Inputs:
%   vehiclePosNED   [1x3] or [3x1] current position [North East Down] [m]
%   vehicleVelNED   [1x3] or [3x1] current velocity [North East Down] [m/s]
%   waypointsNED    [Nx3] waypoint matrix [North East Down] [m]
%   activeLegIdxIn  active leg index. Leg i uses rows i and i+1.
%   waypointRadius_m mission waypoint radius used for leg switching [m]
%
% Outputs:
%   crossTrackError_m     signed horizontal cross-track error [m]
%   crossTrackRate_mps    signed horizontal cross-track rate [m/s]
%   alongTrackRate_mps    along-track rate [m/s]
%   activeLegIdxOut       updated active leg index after switch logic
%   switchFlag            1 when a leg switch occurred this call, else 0
%   distanceToWaypointB_m horizontal distance to the current waypoint B [m]
%   closestPointNED       closest point on the active waypoint leg [N E D]
%   altitudeCmd_m         desired altitude [m], positive Up
%
% Notes:
% - This matches the ArduPilot-style horizontal calculation on the active
%   waypoint leg using the N/E components only.
% - altitudeCmd_m follows purePursuitGuidanceNED and commands the altitude
%   of the next active waypoint directly.
% - Sign convention follows the 2D scalar cross product:
%       u %% v = u_N*v_E - u_E*v_N
% - Mission leg advance occurs when either:
%       1) distance to waypoint B <= waypointRadius_m, or
%       2) the aircraft crosses the finish line at waypoint B, defined as
%          the line through B perpendicular to the inbound leg A->B

    pos = zeros(1,3);
    vel = zeros(1,3);
    pos(1) = vehiclePosNED(1);
    pos(2) = vehiclePosNED(2);
    pos(3) = vehiclePosNED(3);
    vel(1) = vehicleVelNED(1);
    vel(2) = vehicleVelNED(2);
    vel(3) = vehicleVelNED(3);

    crossTrackError_m = 0.0;
    crossTrackRate_mps = 0.0;
    alongTrackRate_mps = 0.0;
    activeLegIdxOut = 1;
    switchFlag = 0.0;
    distanceToWaypointB_m = 0.0;
    closestPointNED = pos;
    altitudeCmd_m = -pos(3);
    switchReasonCode = 0.0;

    waypointCount = size(waypointsNED, 1);
    if waypointCount < 2 || size(waypointsNED, 2) < 3
        if waypointCount == 1 && size(waypointsNED, 2) >= 3
            altitudeCmd_m = -waypointsNED(1,3);
        end
        return
    end

    if ~isfinite(activeLegIdxIn)
        activeLegIdxIn = 1;
    end
    activeLegIdxOut = floor(activeLegIdxIn);
    activeLegIdxOut = min(max(activeLegIdxOut, 1), waypointCount - 1);

    if ~isfinite(waypointRadius_m) || waypointRadius_m < 0.0
        waypointRadius_m = 0.0;
    end

    while activeLegIdxOut < waypointCount
        wpA = waypointsNED(activeLegIdxOut, :);
        wpB = waypointsNED(activeLegIdxOut + 1, :);
        AB_NE = [wpB(1) - wpA(1), wpB(2) - wpA(2)];
        AB_norm = hypot(AB_NE(1), AB_NE(2));

        if AB_norm <= 1e-9
            if activeLegIdxOut < waypointCount - 1
                activeLegIdxOut = activeLegIdxOut + 1;
                switchReasonCode = 3.0;
                continue
            else
                closestPointNED = wpB;
                altitudeCmd_m = -wpB(3);
                switchFlag = double(switchReasonCode ~= 0.0);
                return
            end
        end

        distanceToWaypointB_m = hypot(pos(1) - wpB(1), pos(2) - wpB(2));
        finishLineCrossed = ((pos(1) - wpB(1)) * AB_NE(1) + ...
            (pos(2) - wpB(2)) * AB_NE(2)) >= 0.0;

        if activeLegIdxOut < waypointCount - 1 && distanceToWaypointB_m <= waypointRadius_m
            activeLegIdxOut = activeLegIdxOut + 1;
            switchReasonCode = 1.0;
            continue
        end

        if activeLegIdxOut < waypointCount - 1 && finishLineCrossed
            activeLegIdxOut = activeLegIdxOut + 1;
            switchReasonCode = 2.0;
            continue
        end

        break
    end

    wpA = waypointsNED(activeLegIdxOut, :);
    wpB = waypointsNED(activeLegIdxOut + 1, :);
    AB_NE = [wpB(1) - wpA(1), wpB(2) - wpA(2)];
    AB_norm = hypot(AB_NE(1), AB_NE(2));

    if AB_norm <= 1e-9
        closestPointNED = wpB;
        distanceToWaypointB_m = hypot(pos(1) - wpB(1), pos(2) - wpB(2));
        altitudeCmd_m = -wpB(3);
        switchFlag = double(switchReasonCode ~= 0.0);
        return
    end

    AB_unit = AB_NE / AB_norm;
    A_air_NE = [pos(1) - wpA(1), pos(2) - wpA(2)];
    vel_NE = [vel(1), vel(2)];
    sStarRaw = (A_air_NE(1) * AB_NE(1) + A_air_NE(2) * AB_NE(2)) / (AB_norm^2);
    sStarClamped = min(max(sStarRaw, 0.0), 1.0);

    closestPointNED = wpA + sStarClamped * (wpB - wpA);
    distanceToWaypointB_m = hypot(pos(1) - wpB(1), pos(2) - wpB(2));

    crossTrackError_m = A_air_NE(1) * AB_unit(2) - A_air_NE(2) * AB_unit(1);
    crossTrackRate_mps = vel_NE(1) * AB_unit(2) - vel_NE(2) * AB_unit(1);
    alongTrackRate_mps = vel_NE(1) * AB_unit(1) + vel_NE(2) * AB_unit(2);
    altitudeCmd_m = -wpB(3);
    switchFlag = double(switchReasonCode ~= 0.0);
end

function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildDubinsChunk(uavConn, startXYZ, startHeading, goalXYZ, goalHeading, ds, modeCode, stageCode, uavAirSpeed, passNumber, powerW)

    if norm(goalXYZ - startXYZ) < 1e-9
        pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
        segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
        segPass = nan(0,1); segPowerW = zeros(0,1);
        return
    end

    startPose = [startXYZ(2), startXYZ(1), -startXYZ(3), startHeading];
    goalPose  = [goalXYZ(2),  goalXYZ(1),  -goalXYZ(3),  goalHeading];
    [pathCell, ~] = connect(uavConn, startPose, goalPose);
    pathObj = pathCell{1};

    s = buildExactSampleVector(0, pathObj.Length, ds);
    posesNED = interpolate(pathObj, s);
    pts = [posesNED(:,2), posesNED(:,1), -posesNED(:,3)];
    roll = posesNED(:,6);
    gamma = posesNED(:,5);

    segLengths = vecnorm(diff(pts,1,1),2,2);
    segTimes = segLengths / uavAirSpeed;
    segModes = repmat(modeCode, numel(segTimes), 1);
    segStages = repmat(stageCode, numel(segTimes), 1);
    segPass = repmat(passNumber, numel(segTimes), 1);
    segPowerW = repmat(powerW, numel(segTimes), 1);
end

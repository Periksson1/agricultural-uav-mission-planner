function [pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW] = ...
    buildStraightChunk(startXYZ, endXYZ, ds, uavAirSpeed, modeCode, stageCode, passNumber, powerW)

    dXYZ = endXYZ - startXYZ;
    len = norm(dXYZ);
    if len <= 1e-12
        pts = zeros(0,3); roll = zeros(0,1); gamma = zeros(0,1);
        segTimes = zeros(0,1); segModes = zeros(0,1); segStages = zeros(0,1);
        segPass = nan(0,1); segPowerW = zeros(0,1);
        return
    end

    s = buildExactSampleVector(0, len, ds);
    frac = s(:) / len;
    pts = startXYZ + frac .* dXYZ;
    roll = zeros(numel(s),1);
    gammaVal = atan2(dXYZ(3), max(norm(dXYZ(1:2)), eps));
    gamma = gammaVal * ones(numel(s),1);

    segLengths = vecnorm(diff(pts,1,1),2,2);
    segTimes = segLengths / uavAirSpeed;
    segModes = repmat(modeCode, numel(segTimes), 1);
    segStages = repmat(stageCode, numel(segTimes), 1);
    segPass = repmat(passNumber, numel(segTimes), 1);
    segPowerW = repmat(powerW, numel(segTimes), 1);
end

function [allPts, allRoll, allGamma, allSegTimes, allSegModes, allSegStages, allSegPass, allSegPowerW] = ...
    appendChunkData(allPts, allRoll, allGamma, allSegTimes, allSegModes, allSegStages, allSegPass, allSegPowerW, ...
    pts, roll, gamma, segTimes, segModes, segStages, segPass, segPowerW)

    connectionTol = 1e-6;

    if isempty(segTimes)
        return
    end

    if isempty(allSegTimes)
        allPts = pts;
        allRoll = roll(:);
        allGamma = gamma(:);
        allSegTimes = segTimes(:);
        allSegModes = segModes(:);
        allSegStages = segStages(:);
        allSegPass = segPass(:);
        allSegPowerW = segPowerW(:);
    else
        if norm(allPts(end,:) - pts(1,:)) > connectionTol
            error('Attempted to append a non-connected chunk.');
        end
        pts(1,:) = allPts(end,:);
        allPts = [allPts; pts(2:end,:)];
        allRoll = [allRoll; roll(2:end)];
        allGamma = [allGamma; gamma(2:end)];
        allSegTimes = [allSegTimes; segTimes(:)];
        allSegModes = [allSegModes; segModes(:)];
        allSegStages = [allSegStages; segStages(:)];
        allSegPass = [allSegPass; segPass(:)];
        allSegPowerW = [allSegPowerW; segPowerW(:)];
    end
end

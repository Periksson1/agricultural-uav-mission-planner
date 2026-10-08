function cat = stageToDisplayCategory(stageCode)
    codes = missionCodes();

    if stageCode == codes.stage.sprayLeg
        cat = codes.displayCategory.spray;
    elseif ismember(stageCode, codes.stageGroups.interPass)
        cat = codes.displayCategory.turn;
    elseif ismember(stageCode, codes.stageGroups.vtol)
        cat = codes.displayCategory.vtol;
    else
        cat = codes.displayCategory.transit;
    end
end

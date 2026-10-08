function str = stageCodeToString(stageCode)
    codes = missionCodes();

    switch stageCode
        case codes.stage.sprayLeg
            str = 'Spray leg';
        case codes.stage.vtolOutboundClimb
            str = 'VTOL climb / descend';
        case codes.stage.vtolOutboundHover
            str = 'Hover';
        case codes.stage.transferCruise
            str = 'Outbound / ingress';
        case codes.stage.transferRejoin
            str = 'Inter-field / rejoin';
        case codes.stage.returnCruise
            str = 'Return cruise';
        case codes.stage.preLandingHover
            str = 'Pre-landing hover';
        case codes.stage.vtolLandingVertical
            str = 'VTOL vertical';
        case codes.stage.turnaroundPause
            str = 'Turnaround';
        case codes.stage.interPassClimbOut
            str = 'Inter-pass climb-out';
        case codes.stage.interPassElevatedTurn
            str = 'Elevated banked turn';
        case codes.stage.interPassDescent
            str = 'Inter-pass descent';
        case codes.stage.interPassRunIn
            str = 'Level run-in';
        case codes.stage.interPassRunOut
            str = 'Level run-out after spray';
        case codes.stage.returnPullUp
            str = 'Return pull-up before GS turn';
        case codes.stage.returnRunOut
            str = 'Return run-out before pull-up';
        otherwise
            str = 'Unknown';
    end
end

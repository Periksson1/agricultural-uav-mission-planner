function str = modeGroupCodeToString(modeCode, passNum)
    codes = missionCodes();

    switch modeCode
        case codes.mode.spray
            if ~isnan(passNum)
                str = sprintf('SPRAYING (Pass %d)', passNum);
            else
                str = 'SPRAYING';
            end
        case codes.mode.interPassTurn
            str = 'INTER-PASS TURN';
        case codes.mode.transfer
            str = 'TRANSFER / INGRESS';
        case codes.mode.returnToBase
            str = 'RETURN / LANDING';
        case codes.mode.turnaround
            str = 'TURNAROUND / REFILL';
        otherwise
            str = 'UNKNOWN';
    end
end

function createMissionFigure(missionCfg, planningData, ~, plotData, displayOptions)
    if nargin < 5
        displayOptions = struct();
    end

    showMissionAssumptions = getDisplayOption(displayOptions, 'showMissionAssumptions', true);
    showCurrentAircraftState = getDisplayOption(displayOptions, 'showCurrentAircraftState', true);
    showSortieLog = getDisplayOption(displayOptions, 'showSortieLog', true);
    usePlaybackOnlyLayout = ~showMissionAssumptions && ~showCurrentAircraftState && ~showSortieLog;

    fieldCount = planningData.fieldCount;
    fieldVisitOrder = planningData.fieldVisitOrder;
    fieldDefs = planningData.fields;
    fieldInfo = planningData.fieldInfo;
    fieldFirstCheckpoints = planningData.fieldFirstCheckpoints;
    groundStationCentreXY = planningData.groundStationCentreXY;
    groundStationX = planningData.groundStationX;
    groundStationY = planningData.groundStationY;
    groundStationZ = planningData.groundStationZ;
    originalFieldCount = planningData.originalFieldCount;
    combinedFieldCount = planningData.combinedFieldCount;
    totalFieldArea_ha = planningData.totalFieldArea_ha;
    actualTurnRadius = planningData.actualTurnRadius;

    fig = figure('Color','w');
    ax = axes('Parent', fig);
    if usePlaybackOnlyLayout
        set(fig, 'Name', 'Mission playback and visualisation');
        set(ax, 'Position', [0.07 0.24 0.86 0.66]);
    else
        set(ax, 'Position', [0.24 0.23 0.54 0.68]);
    end
    hold(ax, 'on')
    drawRealWorldGroundContext(ax, planningData);

    fieldPatchHandles = gobjects(fieldCount,1);
    fieldFaceColor = [0.60 0.90 0.60];
    completedFieldLaneColor = [0.00 0.42 0.00];
    laneCompletionPatchHandles = cell(planningData.totalLaneCount,1);
    allFieldX = [];
    allFieldY = [];

    for visitIdx = 1:fieldCount
        fIdx = fieldVisitOrder(visitIdx);
        displayLoops = fieldInfo(fIdx).displayBoundaryLoops;
        firstHandleMade = false;

        for iLoop = 1:numel(displayLoops)
            cxy = displayLoops{iLoop};
            if isempty(cxy) || size(cxy,1) < 3
                continue
            end
            hLoop = fill3(ax, cxy(:,1), cxy(:,2), groundZForXY(planningData, cxy), fieldFaceColor, ...
                'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.22);
            if ~firstHandleMade
                fieldPatchHandles(fIdx) = hLoop;
                firstHandleMade = true;
            end
            allFieldX = [allFieldX; cxy(:,1)]; %#ok<AGROW>
            allFieldY = [allFieldY; cxy(:,2)]; %#ok<AGROW>
        end

        for iLane = 1:fieldInfo(fIdx).nLanes
            passNumber = fieldInfo(fIdx).lanePassNumbers(iLane);
            laneLoops = fieldInfo(fIdx).lanePatchLoops{iLane};
            laneHandles = gobjects(0,1);
            for iLoop = 1:numel(laneLoops)
                xyLoop = laneLoops{iLoop};
                if isempty(xyLoop) || size(xyLoop,1) < 3
                    continue
                end
                laneHandles(end+1,1) = fill3(ax, xyLoop(:,1), xyLoop(:,2), groundZForXY(planningData, xyLoop), completedFieldLaneColor, ...
                    'EdgeColor', 'none', 'FaceAlpha', 0.72, 'Visible', 'off', 'HitTest', 'off'); %#ok<AGROW>
            end
            laneCompletionPatchHandles{passNumber} = laneHandles;
        end

        fieldLabelZ = groundZForXY(planningData, fieldInfo(fIdx).centreXY);
        text(ax, fieldInfo(fIdx).centreXY(1), fieldInfo(fIdx).centreXY(2), fieldLabelZ, ...
            buildMissionFieldLabel(fieldDefs(fIdx), fieldInfo(fIdx)), 'FontSize', 9, ...
            'FontWeight', 'bold', 'Color', [0 0 0], 'HitTest', 'off', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end

    hLegendField = firstValidHandle(fieldPatchHandles);
    if ~isgraphics(hLegendField)
        hLegendField = patch(ax, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
            'FaceColor', fieldFaceColor, 'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.22);
    end

    hCompletedFieldLaneLegend = patch(ax, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
        'FaceColor', completedFieldLaneColor, 'EdgeColor', 'none', 'FaceAlpha', 0.72);

    hGroundStation = fill3(ax, groundStationX, groundStationY, groundStationZ, [1.0 0.83 0.25], ...
        'EdgeColor', [0.60 0.40 0.00], 'LineWidth', 2, 'FaceAlpha', 0.80);

    hBasePath = plot3(ax, plotData.baseDisplayXYZ(:,1), plotData.baseDisplayXYZ(:,2), plotData.baseDisplayXYZ(:,3), ...
        '-', 'Color', [0.88 0.88 0.88], 'LineWidth', 0.7);

    hMissionPath = plot3(ax, plotData.displayPathXYZ(:,1), plotData.displayPathXYZ(:,2), plotData.displayPathXYZ(:,3), ...
        '-', 'Color', [0.82 0.82 0.82], 'LineWidth', 0.9);

    hSelectedSortiePath = plot3(ax, NaN, NaN, NaN, ...
        '-', 'Color', [0.82 0.82 0.82], 'LineWidth', 0.9, 'Visible', 'off');

    sprayColor      = [0.10 0.70 0.10];
    laneTurnColor   = [1.00 0.20 0.70];
    transferColor   = [1.00 0.55 0.10];
    vtolColor       = [0.10 0.45 1.00];

    hExecutedSpray    = plot3(ax, NaN, NaN, NaN, '-', 'Color', sprayColor,    'LineWidth', 2.8);
    hExecutedTurn     = plot3(ax, NaN, NaN, NaN, '-', 'Color', laneTurnColor, 'LineWidth', 2.3);
    hExecutedTransfer = plot3(ax, NaN, NaN, NaN, '-', 'Color', transferColor, 'LineWidth', 2.6);
    hExecutedVTOL     = plot3(ax, NaN, NaN, NaN, '-', 'Color', vtolColor,     'LineWidth', 2.6);

    hUAV = plot3(ax, plotData.playbackModes{1}.frameX(1), plotData.playbackModes{1}.frameY(1), plotData.playbackModes{1}.frameZ(1), ...
        'o', 'Color', transferColor, 'MarkerFaceColor', transferColor, 'MarkerSize', 8);

    hStart = plot3(ax, plotData.pathXYZ(1,1), plotData.pathXYZ(1,2), plotData.pathXYZ(1,3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    hEnd   = plot3(ax, plotData.pathXYZ(end,1), plotData.pathXYZ(end,2), plotData.pathXYZ(end,3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

    plot3(ax, fieldFirstCheckpoints(:,1), fieldFirstCheckpoints(:,2), fieldFirstCheckpoints(:,3), ...
        'md', 'MarkerFaceColor', 'm', 'MarkerSize', 8, 'LineStyle', 'none');
    for i = 1:fieldCount
        text(ax, fieldFirstCheckpoints(i,1), fieldFirstCheckpoints(i,2), fieldFirstCheckpoints(i,3), sprintf('  CP%d', i), ...
            'FontSize', 9, 'FontWeight', 'bold', 'Color', 'm', 'HitTest', 'off');
    end

    if ~isempty(plotData.payloadInterruptPoints)
        hPayloadStops = plot3(ax, plotData.payloadInterruptPoints(:,1), plotData.payloadInterruptPoints(:,2), plotData.payloadInterruptPoints(:,3), ...
            'rx', 'LineWidth', 2.5, 'MarkerSize', 10);
    else
        hPayloadStops = plot3(ax, NaN, NaN, NaN, 'rx', 'LineWidth', 2.5, 'MarkerSize', 10);
    end

    if ~isempty(plotData.batteryInterruptPoints)
        hBatteryStops = plot3(ax, plotData.batteryInterruptPoints(:,1), plotData.batteryInterruptPoints(:,2), plotData.batteryInterruptPoints(:,3), ...
            'bx', 'LineWidth', 2.5, 'MarkerSize', 10);
    else
        hBatteryStops = plot3(ax, NaN, NaN, NaN, 'bx', 'LineWidth', 2.5, 'MarkerSize', 10);
    end

    text(ax, groundStationCentreXY(1), groundStationCentreXY(2), 0, '  Ground station', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.50 0.30 0.00], 'HitTest', 'off');

    grid(ax, 'on')
    axis(ax, 'equal')
    daspect(ax, [1 1 1/max(missionCfg.verticalExaggeration, eps)])
    xlabel(ax, 'East [m]')
    ylabel(ax, 'North [m]')
    zlabel(ax, 'Up [m]')
    title(ax, 'Multi-field spray mission')

    xCandidates = [allFieldX(:); groundStationX(:); plotData.displayPathXYZ(:,1)];
    yCandidates = [allFieldY(:); groundStationY(:); plotData.displayPathXYZ(:,2)];
    xRange = max(xCandidates) - min(xCandidates);
    yRange = max(yCandidates) - min(yCandidates);
    xMargin = max(5, 0.03*max(xRange,1));
    yMargin = max(5, 0.03*max(yRange,1));
    zCandidates = [plotData.displayPathXYZ(:,3); groundStationZ(:); collectGroundElevationCandidates(planningData)];
    zRange = max(zCandidates) - min(zCandidates);
    zMargin = max(1, 0.20*max(zRange, max(missionCfg.vtolAltitude_m, missionCfg.turnClimbAltitude_m)));

    xlim(ax, [min(xCandidates)-xMargin, max(xCandidates)+xMargin])
    ylim(ax, [min(yCandidates)-yMargin, max(yCandidates)+yMargin])
    zlim(ax, [min(zCandidates)-zMargin, max(zCandidates)+zMargin])

    legend(ax, ...
        [hLegendField, hCompletedFieldLaneLegend, hGroundStation, hMissionPath, hSelectedSortiePath, ...
         hExecutedSpray, hExecutedTurn, hExecutedTransfer, hExecutedVTOL, ...
         hPayloadStops, hBatteryStops, hStart, hEnd], ...
        'Fields', 'Completed field lane', 'Ground station', 'Executed mission path (grey)', 'Sortie-only path', ...
        'Spraying', 'Inter-pass turn', 'Transfer', 'VTOL', ...
        'Payload-empty interrupt', 'Battery-threshold interrupt', 'Start', 'End', ...
        'Location', 'northeast');

    if showMissionAssumptions
        fieldInfoLines = regexprep(plotData.fieldSummaryLines, ' \| Lane yaw [-+]?\d+(\.\d+)? deg', '');
        fieldInfoLines = regexprep(fieldInfoLines, ' \| Deferred short edge lanes \d+', '');
        fieldInfoBlock = strjoin(fieldInfoLines, newline);
        infoText = sprintf([ ...
            'Assumptions\n' ...
            'Original fields drawn: %d\n' ...
            'Combined spray blocks: %d\n' ...
            'Fields in plan: %d total (visit order %s)\n' ...
            'Visit-order optimisation active: %s\n' ...
            'Total field area: %.3f ha\n' ...
            '%s\n' ...
            'Spray width: %.1f m\n' ...
            'Overlap: %.0f %%\n' ...
            'Lane spacing: %.1f m\n' ...
            'Spray height: %.1f m\n' ...
            'Airspeed: %.2f m/s\n' ...
            'Max roll angle: %.1f deg\n' ...
            'Turn radius: %.2f m\n' ...
            'Inter-pass climb height: %.1f m\n' ...
            'Return pull-up height before GS turn: %.1f m\n' ...
            'Pre-descent level segment: %.1f m\n' ...
            'Pre-spray level run-in: %.1f m\n' ...
            'Field-entry / transfer height: %.1f m\n' ...
            'VTOL altitude: %.1f m\n' ...
            'VTOL climb rate: %.2f m/s\n' ...
            'GS North/East: [%.1f, %.1f] m\n' ...
            'GS size: %.1f x %.1f m\n' ...
            'Battery reserve after landing: %.1f %%\n' ...
            'Turnaround time: %.1f min\n' ...
            'Payload: %.1f L\n' ...
            'Application rate: %.1f L/ha\n' ...
            'Battery energy: %.3f kWh\n' ...
            'Cruise power: %.2f kW\n' ...
            'Cruise climb power: %.2f kW\n' ...
            'VTOL hover power: %.2f kW\n' ...
            'VTOL climb power: %.2f kW'], ...
            originalFieldCount, combinedFieldCount, planningData.fieldCount, mat2str(planningData.fieldVisitOrder), ...
            ternary(planningData.fieldVisitOrderWasOptimized, 'yes', 'no'), totalFieldArea_ha, fieldInfoBlock, ...
            missionCfg.sprayWidth, missionCfg.overlap*100, planningData.laneSpacing, missionCfg.flightHeight, missionCfg.uavAirSpeed, ...
            rad2deg(missionCfg.maxRollAngle), actualTurnRadius, missionCfg.turnClimbAltitude_m, missionCfg.returnPullUpAltitude_m, ...
            missionCfg.preDescentLevelDistance_m, missionCfg.preSprayLevelDistance_m, missionCfg.fieldTransferAltitude_m, missionCfg.vtolAltitude_m, missionCfg.vtolClimbRate_mps, ...
            missionCfg.groundStationNorth_m, missionCfg.groundStationEast_m, missionCfg.groundStationSize, missionCfg.groundStationSize, ...
            missionCfg.batteryReturnThreshold_pct, missionCfg.turnaroundTime_s/60, missionCfg.payloadLitres, missionCfg.applicationRate, ...
            planningData.batteryEnergy_kWh, missionCfg.cruisePowerDraw_kW, missionCfg.cruiseClimbPowerDraw_kW, ...
            missionCfg.vtolHoverPowerDraw_kW, missionCfg.vtolClimbPowerDraw_kW);

        annotation(fig, 'textbox', [0.02 0.37 0.20 0.52], ...
            'String', infoText, 'FitBoxToText', 'on', 'BackgroundColor', 'w', ...
            'EdgeColor', 'k', 'FontSize', 8.8, 'Interpreter', 'none');
    end

    if usePlaybackOnlyLayout
        sliderLabelPosition = [0.12 0.135 0.62 0.03];
        playbackModeLabelPosition = [0.12 0.175 0.11 0.025];
        playbackModePopupPosition = [0.24 0.175 0.24 0.035];
        viewTogglePosition = [0.50 0.175 0.10 0.035];
        sliderStartPosition = [0.12 0.045 0.08 0.025];
        sliderEndPosition = [0.58 0.045 0.12 0.025];
        sliderPosition = [0.12 0.075 0.58 0.04];
        playPausePosition = [0.72 0.075 0.08 0.045];
    else
        sliderLabelPosition = [0.31 0.135 0.44 0.03];
        playbackModeLabelPosition = [0.31 0.175 0.11 0.025];
        playbackModePopupPosition = [0.42 0.175 0.24 0.035];
        viewTogglePosition = [0.67 0.175 0.10 0.035];
        sliderStartPosition = [0.31 0.045 0.08 0.025];
        sliderEndPosition = [0.68 0.045 0.12 0.025];
        sliderPosition = [0.31 0.075 0.37 0.04];
        playPausePosition = [0.70 0.075 0.08 0.045];
    end

    hTimeText = annotation(fig, 'textbox', [0.03 0.08 0.19 0.23], ...
        'String', '', 'FitBoxToText', 'off', 'BackgroundColor', 'w', ...
        'EdgeColor', 'k', 'FontSize', 8.4, 'Interpreter', 'none', ...
        'Visible', ternary(showCurrentAircraftState, 'on', 'off'));

    hSliderLabel = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', sliderLabelPosition, 'String', '', ...
        'BackgroundColor', get(fig,'Color'), 'HorizontalAlignment', 'left', ...
        'FontSize', 10, 'FontWeight', 'bold');

    uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', playbackModeLabelPosition, 'String', 'Playback mode', ...
        'BackgroundColor', get(fig,'Color'), 'HorizontalAlignment', 'left', ...
        'FontSize', 10, 'FontWeight', 'bold');

    hPlaybackModePopup = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
        'Position', playbackModePopupPosition, 'String', plotData.playbackModeNames, ...
        'Value', 1, 'BackgroundColor', 'w', 'FontSize', 10);

    hViewToggle = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', viewTogglePosition, 'String', 'Switch to 3D', ...
        'BackgroundColor', 'w', 'FontSize', 10, 'FontWeight', 'bold');

    uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', sliderStartPosition, 'String', '0 s', ...
        'BackgroundColor', get(fig,'Color'), 'HorizontalAlignment', 'left', 'FontSize', 9);

    hSliderEndLabel = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', sliderEndPosition, 'String', '', ...
        'BackgroundColor', get(fig,'Color'), 'HorizontalAlignment', 'right', 'FontSize', 9);

    hSlider = uicontrol(fig, 'Style', 'slider', ...
        'Min', 0, 'Max', max(plotData.playbackModes{1}.visualDuration, eps), 'Value', 0, ...
        'SliderStep', [1/max(plotData.playbackModes{1}.nFrames-1,1), min(20/max(plotData.playbackModes{1}.nFrames-1,1),1)], ...
        'Units', 'normalized', 'Position', sliderPosition);

    hPlayPause = uicontrol(fig, 'Style', 'togglebutton', 'Units', 'normalized', ...
        'Position', playPausePosition, 'String', 'Play', ...
        'FontSize', 10, 'FontWeight', 'bold');

    if showSortieLog
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.82 0.34 0.16 0.03], 'String', 'Sortie log', ...
            'BackgroundColor', get(fig,'Color'), 'HorizontalAlignment', 'left', ...
            'FontSize', 10, 'FontWeight', 'bold');

        uicontrol(fig, 'Style', 'listbox', 'Units', 'normalized', ...
            'Position', [0.82 0.08 0.16 0.26], 'String', plotData.sortieLines, ...
            'FontSize', 9, 'BackgroundColor', 'w');
    end

    graphicsHandles.ax = ax;
    graphicsHandles.hBasePath = hBasePath;
    graphicsHandles.hMissionPath = hMissionPath;
    graphicsHandles.hSelectedSortiePath = hSelectedSortiePath;
    graphicsHandles.hExecutedSpray = hExecutedSpray;
    graphicsHandles.hExecutedTurn = hExecutedTurn;
    graphicsHandles.hExecutedTransfer = hExecutedTransfer;
    graphicsHandles.hExecutedVTOL = hExecutedVTOL;
    graphicsHandles.laneCompletionPatchHandles = laneCompletionPatchHandles;
    graphicsHandles.passCompletionMissionTime = plotData.passCompletionMissionTime;
    graphicsHandles.hUAV = hUAV;
    graphicsHandles.hPayloadStops = hPayloadStops;
    graphicsHandles.hBatteryStops = hBatteryStops;
    graphicsHandles.payloadInterruptPoints = plotData.payloadInterruptPoints;
    graphicsHandles.batteryInterruptPoints = plotData.batteryInterruptPoints;
    graphicsHandles.payloadInterruptSorties = plotData.payloadInterruptSorties;
    graphicsHandles.batteryInterruptSorties = plotData.batteryInterruptSorties;
    graphicsHandles.hTimeText = hTimeText;
    graphicsHandles.hSliderLabel = hSliderLabel;
    graphicsHandles.hSlider = hSlider;
    graphicsHandles.hSliderEndLabel = hSliderEndLabel;
    graphicsHandles.hPlayPause = hPlayPause;
    graphicsHandles.hPlaybackModePopup = hPlaybackModePopup;
    graphicsHandles.hViewToggle = hViewToggle;
    graphicsHandles.showCurrentAircraftState = showCurrentAircraftState;
    graphicsHandles.sprayColor = sprayColor;
    graphicsHandles.turnColor = laneTurnColor;
    graphicsHandles.transferColor = transferColor;
    graphicsHandles.vtolColor = vtolColor;

    setappdata(fig, 'playbackModes', plotData.playbackModes);
    setappdata(fig, 'activePlaybackIndex', 1);
    setappdata(fig, 'currentFrameIdx', 1);
    setappdata(fig, 'graphicsHandles', graphicsHandles);
    setappdata(fig, 'verticalExaggeration', missionCfg.verticalExaggeration);
    setappdata(fig, 'isTopDownView', true);
    setappdata(fig, 'viewAzimuth_deg', missionCfg.viewAzimuth_deg);
    setappdata(fig, 'viewElevation_deg', missionCfg.viewElevation_deg);
    setappdata(fig, 'totalMissionTime', plotData.totalTime);

    playbackTimer = timer('ExecutionMode', 'fixedRate', 'Period', 1/missionCfg.playbackFPS, ...
        'BusyMode', 'drop', 'TimerFcn', @(~,~) timerStepFrames(fig));
    setappdata(fig, 'playbackTimer', playbackTimer);

    hPlaybackModePopup.Callback = @(~,~) onPlaybackModeChanged(fig);
    hSlider.Callback = @(~,~) onSliderChanged(fig);
    sliderListener = addlistener(hSlider, 'ContinuousValueChange', @(~,~) onSliderChanged(fig));
    setappdata(fig, 'sliderListener', sliderListener);
    hPlayPause.Callback = @(~,~) togglePlayPauseFrames(fig);
    hViewToggle.Callback = @(~,~) toggleViewMode(fig);
    fig.CloseRequestFcn = @(src,~) closeFigureCleanup(src);

    configurePlaybackUI(fig, 1);
    applyViewMode(fig);
    renderPlaybackFrame(fig, 1);
end

function drawRealWorldGroundContext(~, ~)
end

function zCandidates = collectGroundElevationCandidates(~)
    zCandidates = 0;
end

function z = groundZForXY(~, xy)
    z = zeros(size(xy,1), 1);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function value = getDisplayOption(options, fieldName, defaultValue)
    value = defaultValue;
    if isstruct(options) && isfield(options, fieldName)
        value = options.(fieldName);
    end
end

function h = firstValidHandle(handleArray)
    h = gobjects(1);
    for i = 1:numel(handleArray)
        if isgraphics(handleArray(i))
            h = handleArray(i);
            return
        end
    end
end

function labelText = buildMissionFieldLabel(fieldDef, info)
    area_m2 = info.area_ha * 10000;
    if isfield(fieldDef, 'type') && strcmpi(fieldDef.type, 'rectangle') && ...
            isfield(fieldDef, 'length_m') && isfield(fieldDef, 'width_m') && ...
            isfinite(fieldDef.length_m) && isfinite(fieldDef.width_m)
        labelText = sprintf('%s\n%.1f x %.1f m\n%.3f ha', ...
            info.name, fieldDef.length_m, fieldDef.width_m, info.area_ha);
    else
        labelText = sprintf('%s\n%.0f m^2 | %.3f ha', ...
            info.name, area_m2, info.area_ha);
    end
end

function configurePlaybackUI(fig, playbackIndex)
    playbackModes = getappdata(fig, 'playbackModes');
    gh = getappdata(fig, 'graphicsHandles');
    pack = playbackModes{playbackIndex};

    set(gh.hSlider, 'Min', 0, 'Max', max(pack.visualDuration, eps), 'Value', 0, ...
        'SliderStep', [1/max(pack.nFrames-1,1), min(20/max(pack.nFrames-1,1),1)]);
    set(gh.hSliderEndLabel, 'String', sprintf('%.1f s', pack.visualDuration));

    if playbackIndex == 1
        set(gh.hBasePath, 'Visible', 'on');
        set(gh.hMissionPath, 'Visible', 'on');
        set(gh.hSelectedSortiePath, 'Visible', 'off', 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        updateInterruptMarkersForSortie(gh, NaN);
    else
        set(gh.hBasePath, 'Visible', 'off');
        set(gh.hMissionPath, 'Visible', 'off');
        set(gh.hSelectedSortiePath, 'Visible', 'on', ...
            'XData', pack.pathXYZ(:,1), 'YData', pack.pathXYZ(:,2), 'ZData', pack.pathXYZ(:,3));
        updateInterruptMarkersForSortie(gh, playbackIndex - 1);
    end
end

function updateInterruptMarkersForSortie(gh, sortieNumber)
    setInterruptMarkerData(gh.hPayloadStops, gh.payloadInterruptPoints, gh.payloadInterruptSorties, sortieNumber);
    setInterruptMarkerData(gh.hBatteryStops, gh.batteryInterruptPoints, gh.batteryInterruptSorties, sortieNumber);
end

function setInterruptMarkerData(hMarker, points, sorties, sortieNumber)
    if isempty(points)
        set(hMarker, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        return
    end

    if isfinite(sortieNumber)
        mask = sorties == sortieNumber;
        points = points(mask,:);
    end

    if isempty(points)
        set(hMarker, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    else
        set(hMarker, 'XData', points(:,1), 'YData', points(:,2), 'ZData', points(:,3));
    end
end

function onPlaybackModeChanged(fig)
    gh = getappdata(fig, 'graphicsHandles');
    playbackTimer = getappdata(fig, 'playbackTimer');
    newIdx = get(gh.hPlaybackModePopup, 'Value');

    if isa(playbackTimer, 'timer') && isvalid(playbackTimer) && strcmp(playbackTimer.Running,'on')
        stop(playbackTimer);
    end
    set(gh.hPlayPause, 'Value', 0, 'String', 'Play');
    setappdata(fig, 'activePlaybackIndex', newIdx);
    setappdata(fig, 'currentFrameIdx', 1);
    configurePlaybackUI(fig, newIdx);
    renderPlaybackFrame(fig, 1);
end

function onSliderChanged(fig)
    playbackModes = getappdata(fig, 'playbackModes');
    activeIdx = getappdata(fig, 'activePlaybackIndex');
    gh = getappdata(fig, 'graphicsHandles');
    pack = playbackModes{activeIdx};

    if pack.visualDuration <= 0 || pack.nFrames <= 1
        frameIdx = 1;
    else
        frameIdx = 1 + round((pack.nFrames - 1) * get(gh.hSlider, 'Value') / pack.visualDuration);
        frameIdx = max(1, min(pack.nFrames, frameIdx));
    end
    setappdata(fig, 'currentFrameIdx', frameIdx);
    set(gh.hSlider, 'Value', pack.frameVisualTime(frameIdx));
    renderPlaybackFrame(fig, frameIdx);
end

function togglePlayPauseFrames(fig)
    gh = getappdata(fig, 'graphicsHandles');
    playbackTimer = getappdata(fig, 'playbackTimer');
    playbackModes = getappdata(fig, 'playbackModes');
    activeIdx = getappdata(fig, 'activePlaybackIndex');
    currentFrameIdx = getappdata(fig, 'currentFrameIdx');
    pack = playbackModes{activeIdx};

    if get(gh.hPlayPause, 'Value') == 1
        if currentFrameIdx >= pack.nFrames
            currentFrameIdx = 1;
            setappdata(fig, 'currentFrameIdx', currentFrameIdx);
            renderPlaybackFrame(fig, currentFrameIdx);
        end
        set(gh.hPlayPause, 'String', 'Pause');
        if strcmp(playbackTimer.Running,'off')
            start(playbackTimer);
        end
    else
        set(gh.hPlayPause, 'String', 'Play');
        if strcmp(playbackTimer.Running,'on')
            stop(playbackTimer);
        end
    end
end

function toggleViewMode(fig)
    setappdata(fig, 'isTopDownView', ~getappdata(fig, 'isTopDownView'));
    applyViewMode(fig);
end

function applyViewMode(fig)
    gh = getappdata(fig, 'graphicsHandles');
    ax = gh.ax;
    if getappdata(fig, 'isTopDownView')
        view(ax, 2);
        camproj(ax, 'orthographic');
        title(ax, 'Multi-field spray mission (Top-down 2D)');
        set(gh.hViewToggle, 'String', 'Switch to 3D');
    else
        view(ax, getappdata(fig,'viewAzimuth_deg'), getappdata(fig,'viewElevation_deg'));
        camproj(ax, 'perspective');
        title(ax, 'Multi-field spray mission (3D)');
        set(gh.hViewToggle, 'String', 'Switch to 2D');
    end
    daspect(ax, [1 1 1/max(getappdata(fig,'verticalExaggeration'), eps)])
end

function timerStepFrames(fig)
    if ~isgraphics(fig)
        return
    end
    playbackModes = getappdata(fig, 'playbackModes');
    activeIdx = getappdata(fig, 'activePlaybackIndex');
    gh = getappdata(fig, 'graphicsHandles');
    playbackTimer = getappdata(fig, 'playbackTimer');
    pack = playbackModes{activeIdx};

    nextFrameIdx = min(getappdata(fig,'currentFrameIdx') + 1, pack.nFrames);
    setappdata(fig, 'currentFrameIdx', nextFrameIdx);
    set(gh.hSlider, 'Value', pack.frameVisualTime(nextFrameIdx));
    renderPlaybackFrame(fig, nextFrameIdx);

    if nextFrameIdx >= pack.nFrames
        if strcmp(playbackTimer.Running,'on')
            stop(playbackTimer);
        end
        set(gh.hPlayPause, 'Value', 0, 'String', 'Play');
    end
end

function renderPlaybackFrame(fig, frameIdx)
    codes = missionCodes();
    playbackModes = getappdata(fig, 'playbackModes');
    activeIdx = getappdata(fig, 'activePlaybackIndex');
    gh = getappdata(fig, 'graphicsHandles');
    totalMissionTime = getappdata(fig, 'totalMissionTime');
    pack = playbackModes{activeIdx};
    frameIdx = max(1, min(pack.nFrames, frameIdx));

    xNow = pack.frameX(frameIdx);
    yNow = pack.frameY(frameIdx);
    zNow = pack.frameZ(frameIdx);

    xSpr = []; ySpr = []; zSpr = [];
    xTurn = []; yTurn = []; zTurn = [];
    xTra = []; yTra = []; zTra = [];
    xV = []; yV = []; zV = [];

    realTimeNow = pack.frameRealTime(frameIdx);

    lanePatchHandles = gh.laneCompletionPatchHandles;
    passCompletionMissionTime = gh.passCompletionMissionTime;
    for passNum = 1:min(numel(lanePatchHandles), numel(passCompletionMissionTime))
        hLane = lanePatchHandles{passNum};
        if isempty(hLane)
            continue
        end
        if isfinite(passCompletionMissionTime(passNum)) && realTimeNow >= passCompletionMissionTime(passNum) - 1e-10
            set(hLane, 'Visible', 'on');
        else
            set(hLane, 'Visible', 'off');
        end
    end

    for i = 1:numel(pack.segStages)
        segStartT = pack.pathTimeReal(i);
        segEndT = pack.pathTimeReal(i+1);

        if realTimeNow <= segStartT + 1e-12
            continue
        end

        p0 = pack.pathXYZ(i,:);
        p1 = pack.pathXYZ(i+1,:);
        cat = stageToDisplayCategory(pack.segStages(i));

        if realTimeNow >= segEndT - 1e-12
            if cat == codes.displayCategory.spray
                [xSpr, ySpr, zSpr] = appendDisplayLine(xSpr, ySpr, zSpr, p0, p1);
            elseif cat == codes.displayCategory.turn
                [xTurn, yTurn, zTurn] = appendDisplayLine(xTurn, yTurn, zTurn, p0, p1);
            elseif cat == codes.displayCategory.vtol
                [xV, yV, zV] = appendDisplayLine(xV, yV, zV, p0, p1);
            else
                [xTra, yTra, zTra] = appendDisplayLine(xTra, yTra, zTra, p0, p1);
            end
        else
            frac = (realTimeNow - segStartT) / max(segEndT - segStartT, eps);
            frac = max(0, min(1, frac));
            pNow = p0 + frac * (p1 - p0);

            if cat == codes.displayCategory.spray
                [xSpr, ySpr, zSpr] = appendDisplayLine(xSpr, ySpr, zSpr, p0, pNow);
            elseif cat == codes.displayCategory.turn
                [xTurn, yTurn, zTurn] = appendDisplayLine(xTurn, yTurn, zTurn, p0, pNow);
            elseif cat == codes.displayCategory.vtol
                [xV, yV, zV] = appendDisplayLine(xV, yV, zV, p0, pNow);
            else
                [xTra, yTra, zTra] = appendDisplayLine(xTra, yTra, zTra, p0, pNow);
            end
            break
        end
    end

    set(gh.hExecutedSpray,    'XData', xSpr,  'YData', ySpr,  'ZData', zSpr);
    set(gh.hExecutedTurn,     'XData', xTurn, 'YData', yTurn, 'ZData', zTurn);
    set(gh.hExecutedTransfer, 'XData', xTra,  'YData', yTra,  'ZData', zTra);
    set(gh.hExecutedVTOL,     'XData', xV,    'YData', yV,    'ZData', zV);

    catNow = pack.frameDisplayCat(frameIdx);
    if catNow == codes.displayCategory.spray
        markerColor = gh.sprayColor;
    elseif catNow == codes.displayCategory.turn
        markerColor = gh.turnColor;
    elseif catNow == codes.displayCategory.vtol
        markerColor = gh.vtolColor;
    else
        markerColor = gh.transferColor;
    end

    set(gh.hUAV, 'XData', xNow, 'YData', yNow, 'ZData', zNow, ...
        'Color', markerColor, 'MarkerFaceColor', markerColor);

    if gh.showCurrentAircraftState
        modeStr = modeGroupCodeToString(pack.frameMode(frameIdx), pack.framePass(frameIdx));
        stageStr = stageCodeToString(pack.frameStage(frameIdx));

        gh.hTimeText.String = sprintf([ ...
            'Current state\n' ...
            'Playback: %s\n' ...
            'Sortie: %d\n' ...
            'Mode group: %s\n' ...
            'Stage: %s\n' ...
            'Battery: %.1f %%\n' ...
            'Payload: %.2f L\n' ...
            'Weight: %.2f kg\n' ...
            'Total dispensed: %.2f L\n' ...
            'Area sprayed: %.3f ha\n' ...
            'Visual time: %.1f s\n' ...
            'Mission time: %s / %s\n' ...
            'East: %.1f m\n' ...
            'North: %.1f m\n' ...
            'Up: %.1f m\n' ...
            'Roll: %.2f deg\n' ...
            'FPA: %.2f deg\n' ...
            'Heading: %.2f deg'], ...
            pack.name, pack.frameSortie(frameIdx), modeStr, stageStr, ...
            pack.frameBatteryPct(frameIdx), pack.framePayloadL(frameIdx), pack.frameWeightKg(frameIdx), ...
            pack.frameLitres(frameIdx), pack.frameAreaHa(frameIdx), pack.frameVisualTime(frameIdx), ...
            formatDurationClock(realTimeNow), formatDurationClock(totalMissionTime), ...
            xNow, yNow, zNow, pack.frameRollDeg(frameIdx), pack.frameGammaDeg(frameIdx), pack.frameHeadingDeg(frameIdx));
    end

    gh.hSliderLabel.String = sprintf( ...
        'Playback mode: %s   |   Visual time: %.1f / %.1f s   |   Mission time: %s / %s', ...
        pack.name, pack.frameVisualTime(frameIdx), pack.visualDuration, ...
        formatDurationClock(realTimeNow), formatDurationClock(totalMissionTime));

    drawnow limitrate nocallbacks
end

function [xArr, yArr, zArr] = appendDisplayLine(xArr, yArr, zArr, p0, p1)
    if isempty(xArr)
        xArr = [p0(1); p1(1)];
        yArr = [p0(2); p1(2)];
        zArr = [p0(3); p1(3)];
    else
        xArr = [xArr; NaN; p0(1); p1(1)];
        yArr = [yArr; NaN; p0(2); p1(2)];
        zArr = [zArr; NaN; p0(3); p1(3)];
    end
end

function out = formatDurationClock(seconds)
    seconds = max(round(seconds), 0);
    h = floor(seconds / 3600);
    rem1 = seconds - 3600*h;
    m = floor(rem1 / 60);
    s = rem1 - 60*m;
    out = sprintf('%02d:%02d:%02d', h, m, s);
end

function closeFigureCleanup(fig)
    playbackTimer = getappdata(fig, 'playbackTimer');
    if isa(playbackTimer, 'timer') && isvalid(playbackTimer)
        try
            if strcmp(playbackTimer.Running,'on')
                stop(playbackTimer);
            end
            delete(playbackTimer);
        catch
        end
    end
    delete(fig);
end

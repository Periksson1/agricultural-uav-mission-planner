function [fields, fieldCount, fieldVisitOrder, groundStationEast_m, groundStationNorth_m] = ...
    designMissionLayoutInteractive(defaultFieldCount, groundStationSize, ...
    mapXLim, mapYLim, defaultGroundStationEast_m, defaultGroundStationNorth_m, ...
    figurePosition, defaultFieldLength_m, defaultFieldWidth_m, defaultFieldRotation_deg)

    if usejava('desktop')
        answer = inputdlg( ...
            {'Number of rectangular fields to draw:'}, ...
            'Interactive mission layout', ...
            [1 55], ...
            {num2str(defaultFieldCount)});
        if isempty(answer)
            error('Interactive mission-layout setup was cancelled before any fields were drawn.');
        end
        fieldCount = round(str2double(answer{1}));
    else
        fieldCount = defaultFieldCount;
    end

    assert(isfinite(fieldCount) && fieldCount >= 1 && abs(fieldCount - round(fieldCount)) < 1e-12, ...
        'The number of fields must be a positive integer.');

    fieldVisitOrder = 1:fieldCount;
    groundStationEast_m = defaultGroundStationEast_m;
    groundStationNorth_m = defaultGroundStationNorth_m;
    fig = figure('Color', 'w', ...
        'Name', 'Interactive mission layout setup', ...
        'NumberTitle', 'off', ...
        'WindowButtonMotionFcn', @onWindowButtonMotion, ...
        'WindowButtonUpFcn', @onWindowButtonUp, ...
        'WindowKeyPressFcn', @onKeyPress, ...
        'CloseRequestFcn', @onCloseRequest);

    controlPanel = uipanel('Parent', fig, 'Title', 'Field editor', ...
        'Units', 'normalized', 'Position', [0.02 0.06 0.25 0.88], ...
        'BackgroundColor', 'w');

    ax = axes('Parent', fig);
    set(ax, 'Position', [0.31 0.08 0.66 0.84]);
    hold(ax, 'on')
    grid(ax, 'on')
    axis(ax, 'equal')
    xlabel(ax, 'East [m]')
    ylabel(ax, 'North [m]')
    title(ax, {'Mission layout setup', ...
        'Draw each field, drag handles to resize/rotate, double-click the field to confirm'}, ...
        'FontWeight', 'bold')
    xlim(ax, mapXLim)
    ylim(ax, mapYLim)

    plot(ax, defaultGroundStationEast_m, defaultGroundStationNorth_m, 'k+', ...
        'MarkerSize', 10, 'LineWidth', 1.4, 'HitTest', 'off');
    text(ax, defaultGroundStationEast_m, defaultGroundStationNorth_m, ...
        '  Default GS', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.25 0.25 0.25], 'HitTest', 'off');

    ui = createSidebarControls(controlPanel);

    state = struct();
    state.ax = ax;
    state.ui = ui;
    state.fieldCount = fieldCount;
    state.fields = repmat(makeEmptyInteractiveField(), fieldCount, 1);
    state.selectedField = NaN;
    state.isProgrammaticFieldSelectorUpdate = false;
    state.stage = 'placement';
    state.waitingFieldIdx = NaN;
    state.lastPlacementAction = '';
    state.finalFieldsConfirmed = false;
    state.setupCancelled = false;
    state.interaction = makeEmptyInteraction();
    state.minLength_m = 1.0;
    state.minWidth_m = 1.0;
    state.rotationHandleOffset_m = 18.0;
    state.mapXLim = mapXLim;
    state.mapYLim = mapYLim;
    state.groundStationHandle = gobjects(0);
    state.groundStationTextHandle = gobjects(0);
    setappdata(fig, 'layoutState', state);

    set(ui.hEditCentreEast, 'Callback', @onNumericEdit);
    set(ui.hEditCentreNorth, 'Callback', @onNumericEdit);
    set(ui.hEditLength, 'Callback', @onNumericEdit);
    set(ui.hEditWidth, 'Callback', @onNumericEdit);
    set(ui.hEditRotation, 'Callback', @onNumericEdit);
    set(ui.hFieldSelector, 'Callback', @onFieldSelectorChanged);
    set(ui.hConfirmAllFields, 'Callback', @onConfirmAllFields);
    refreshSelectedFieldUI();

    nextDefaultLength_m = defaultFieldLength_m;
    nextDefaultWidth_m = defaultFieldWidth_m;
    nextDefaultRotation_deg = defaultFieldRotation_deg;

    for placementFieldIdx = 1:fieldCount
        fieldConfirmed = false;
        while ~fieldConfirmed
            if ~isgraphics(fig)
                error('Interactive mission-layout setup was cancelled.');
            end

            st = getappdata(fig, 'layoutState');
            st.stage = 'placement';
            st.waitingFieldIdx = placementFieldIdx;
            st.lastPlacementAction = '';
            setappdata(fig, 'layoutState', st);

            updatePlacementInstructions(placementFieldIdx);

            [rectSpec, wasCancelled] = captureInitialRectangleInteractive(ax, placementFieldIdx, ...
                nextDefaultLength_m, nextDefaultWidth_m, nextDefaultRotation_deg);

            if wasCancelled
                if isgraphics(fig)
                    delete(fig);
                end
                error('Interactive mission-layout setup was cancelled before all fields were drawn.');
            end

            createOrReplaceField(placementFieldIdx, rectSpec);
            selectField(placementFieldIdx, true);
            updatePlacementInstructions(placementFieldIdx);
            uiwait(fig);

            if ~isgraphics(fig)
                error('Interactive mission-layout setup was cancelled.');
            end

            st = getappdata(fig, 'layoutState');
            if strcmp(st.lastPlacementAction, 'confirmed')
                fieldConfirmed = true;
                nextDefaultLength_m = st.fields(placementFieldIdx).spec.length_m;
                nextDefaultWidth_m = st.fields(placementFieldIdx).spec.width_m;
                nextDefaultRotation_deg = st.fields(placementFieldIdx).spec.rotation_deg;
            elseif strcmp(st.lastPlacementAction, 'cancelled')
                fieldConfirmed = false;
            else
                fieldConfirmed = st.fields(placementFieldIdx).confirmed;
            end
        end
    end

    st = getappdata(fig, 'layoutState');
    st.stage = 'review';
    st.waitingFieldIdx = NaN;
    st.lastPlacementAction = '';
    setappdata(fig, 'layoutState', st);
    set(ui.hConfirmAllFields, 'Enable', 'on');
    deselectCurrentField(false);
    updateReviewInstructions();

    uiwait(fig);

    if ~isgraphics(fig)
        error('Interactive mission-layout setup was cancelled.');
    end
    st = getappdata(fig, 'layoutState');
    if st.setupCancelled
        error('Interactive mission-layout setup was cancelled.');
    end
    if ~st.finalFieldsConfirmed
        error('Interactive mission-layout setup was cancelled before the field layout was confirmed.');
    end

    updateGroundStationInstructions();
    setFieldGraphicsInteractivity(false);
    restoreFieldInteractivity = onCleanup(@() restoreFieldGraphicsInteractivitySafely(fig, true));
    [groundStationEast_m, groundStationNorth_m] = capturePointInteractive(ax);

    gsHalf = groundStationSize / 2;
    gsX = groundStationEast_m + gsHalf * [-1 1 1 -1];
    gsY = groundStationNorth_m + gsHalf * [-1 -1 1 1];

    st = getappdata(fig, 'layoutState');
    if isgraphics(st.groundStationHandle)
        delete(st.groundStationHandle);
    end
    if isgraphics(st.groundStationTextHandle)
        delete(st.groundStationTextHandle);
    end
    st.groundStationHandle = fill(ax, gsX, gsY, [1.00 0.83 0.25], ...
        'FaceAlpha', 0.85, 'EdgeColor', [0.60 0.40 0.00], 'LineWidth', 2.0, ...
        'HitTest', 'off');
    st.groundStationTextHandle = text(ax, groundStationEast_m, groundStationNorth_m, '  Ground station', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.45 0.28 0.00], 'HitTest', 'off');
    setappdata(fig, 'layoutState', st);

    title(ax, {'Interactive layout captured', ...
        'Continuing directly into the mission run'}, ...
        'FontWeight', 'bold');
    set(ui.hInstructions, 'String', sprintf([ ...
        'Layout complete.\n\n' ...
        'Fields have been confirmed and the ground station has been placed.\n\n' ...
        'The setup figure will now close automatically and the mission will continue.']));
    drawnow
    pause(0.15);

    st = getappdata(fig, 'layoutState');
    fields = repmat(emptyFieldDefTemplate(), fieldCount, 1);
    for iField = 1:fieldCount
        spec = st.fields(iField).spec;
        fields(iField) = createRectangleFieldDef(sprintf('Field %d', iField), ...
            spec.centreEast_m, spec.centreNorth_m, spec.length_m, spec.width_m, spec.rotation_deg);
    end

    if isgraphics(fig)
        delete(fig);
    end

    function ui = createSidebarControls(parentPanel)
        delete(allchild(parentPanel));

        ui.hTitle = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.94 0.90 0.03], 'String', 'Interactive rectangle editor', ...
            'BackgroundColor', 'w', 'FontSize', 11, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left');

        ui.hInstructions = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.79 0.90 0.12], 'String', '', ...
            'BackgroundColor', 'w', 'FontSize', 9.5, 'Max', 20, ...
            'HorizontalAlignment', 'left');

        ui.hStatus = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.74 0.90 0.04], 'String', 'No field selected.', ...
            'BackgroundColor', 'w', 'FontSize', 9.5, ...
            'HorizontalAlignment', 'left', 'ForegroundColor', [0.00 0.45 0.00]);

        uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.69 0.48 0.04], 'String', 'Field selection', ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9.5);
        ui.hFieldSelector = uicontrol(parentPanel, 'Style', 'popupmenu', 'Units', 'normalized', ...
            'Position', [0.58 0.695 0.30 0.04], 'BackgroundColor', 'w', ...
            'String', buildFieldSelectorEntries(), 'Value', 1);

        labelX = 0.05;
        editX = 0.58;
        labelW = 0.48;
        editW = 0.30;
        rowH = 0.05;
        rowY = [0.61 0.54 0.47 0.40 0.33];

        uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [labelX rowY(1) labelW rowH], 'String', 'Centre East [m]', ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9.5);
        ui.hEditCentreEast = uicontrol(parentPanel, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [editX rowY(1)+0.005 editW rowH], 'BackgroundColor', 'w', ...
            'String', '');

        uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [labelX rowY(2) labelW rowH], 'String', 'Centre North [m]', ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9.5);
        ui.hEditCentreNorth = uicontrol(parentPanel, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [editX rowY(2)+0.005 editW rowH], 'BackgroundColor', 'w', ...
            'String', '');

        uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [labelX rowY(3) labelW rowH], 'String', 'Length [m]', ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9.5);
        ui.hEditLength = uicontrol(parentPanel, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [editX rowY(3)+0.005 editW rowH], 'BackgroundColor', 'w', ...
            'String', '');

        uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [labelX rowY(4) labelW rowH], 'String', 'Width [m]', ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9.5);
        ui.hEditWidth = uicontrol(parentPanel, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [editX rowY(4)+0.005 editW rowH], 'BackgroundColor', 'w', ...
            'String', '');

        uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [labelX rowY(5) labelW rowH], 'String', 'Rotation [deg]', ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9.5);
        ui.hEditRotation = uicontrol(parentPanel, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [editX rowY(5)+0.005 editW rowH], 'BackgroundColor', 'w', ...
            'String', '');

        ui.hLiveMetrics = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.21 0.90 0.10], 'String', '', ...
            'BackgroundColor', 'w', 'FontSize', 9.5, 'Max', 20, ...
            'HorizontalAlignment', 'left');

        ui.hTotalAreaTitle = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.17 0.40 0.03], 'String', 'Total area', ...
            'BackgroundColor', 'w', 'FontSize', 9.5, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left');

        ui.hTotalAreaValue = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.45 0.17 0.43 0.03], 'String', '0.0000 ha', ...
            'BackgroundColor', 'w', 'FontSize', 9.5, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'right', 'ForegroundColor', [0.00 0.45 0.00]);

        ui.hAreaSummaryTitle = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.13 0.90 0.03], 'String', 'Field areas [ha]', ...
            'BackgroundColor', 'w', 'FontSize', 9.5, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left');

        ui.hAreaSummary = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.07 0.90 0.06], 'String', {'No fields drawn yet.'}, ...
            'BackgroundColor', 'w', 'FontSize', 9.0, 'Max', 50, ...
            'HorizontalAlignment', 'left');

        ui.hConfirmAllFields = uicontrol(parentPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
            'Position', [0.05 0.025 0.90 0.04], 'String', 'Confirm fields and continue to GS placement', ...
            'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'Enable', 'off');

        ui.hHint = uicontrol(parentPanel, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 0.00 0.90 0.03], ...
            'String', 'Drag patch/centre to move, drag corners to resize, drag circle to rotate, Esc cancels current edit.', ...
            'BackgroundColor', 'w', 'FontSize', 8.0, 'Max', 20, ...
            'HorizontalAlignment', 'left');
    end

    function field = makeEmptyInteractiveField()
        field = struct( ...
            'initialized', false, ...
            'confirmed', false, ...
            'isEditing', false, ...
            'spec', makeDefaultRectSpec(defaultGroundStationEast_m, defaultGroundStationNorth_m, ...
                defaultFieldLength_m, defaultFieldWidth_m, defaultFieldRotation_deg), ...
            'lastConfirmedSpec', makeDefaultRectSpec(defaultGroundStationEast_m, defaultGroundStationNorth_m, ...
                defaultFieldLength_m, defaultFieldWidth_m, defaultFieldRotation_deg), ...
            'patch', gobjects(1), ...
            'label', gobjects(1), ...
            'centerHandle', gobjects(1), ...
            'cornerHandles', gobjects(4,1), ...
            'rotationStem', gobjects(1), ...
            'rotationHandle', gobjects(1));
    end

    function spec = makeDefaultRectSpec(cE, cN, L, W, rot)
        spec = struct('centreEast_m', cE, 'centreNorth_m', cN, ...
            'length_m', L, 'width_m', W, 'rotation_deg', rot);
    end

    function interaction = makeEmptyInteraction()
        interaction = struct('mode', 'none', 'fieldIdx', NaN, 'handleIdx', NaN, ...
            'mouseStartXY', [NaN NaN], 'specStart', makeDefaultRectSpec(NaN, NaN, NaN, NaN, NaN));
    end

    function updatePlacementInstructions(iField)
        set(ui.hInstructions, 'String', sprintf([ ...
            'Field %d of %d.\n\n' ...
            '1. Drag an initial rectangle on the map.\n' ...
            '2. Resize with the corner handles.\n' ...
            '3. Rotate with the circular handle.\n' ...
            '4. Use the sidebar selector to jump back to any drawn field.\n' ...
            '5. Double-click the selected field to confirm it and move on.\n' ...
            '6. Press Esc to cancel the current field and redraw it.'], ...
            iField, fieldCount));
        title(ax, {sprintf('Draw Field %d of %d', iField, fieldCount), ...
            'After the initial drag, resize, rotate, or move it directly on the map. Double-click to confirm.'}, ...
            'FontWeight', 'bold');
    end

    function updateReviewInstructions()
        set(ui.hInstructions, 'String', sprintf([ ...
            'All fields have an initial rectangle.\n\n' ...
            'Double-click any field to reopen it for editing.\n' ...
            'You can choose the active field from the sidebar dropdown.\n' ...
            'Double-click the selected field again to confirm that field.\n' ...
            'You can also type exact values in the boxes.\n' ...
            'When satisfied, click "Confirm fields and continue to GS placement".']));
        title(ax, {'Review and edit fields', ...
            'Double-click a field to edit it again, then press the confirm button when finished.'}, ...
            'FontWeight', 'bold');
    end

    function updateGroundStationInstructions()
        deselectCurrentField(false);
        set(ui.hInstructions, 'String', sprintf([ ...
            'Fields are confirmed.\n\n' ...
            'Click once on the map to place the ground-station centre.\n\n' ...
            'Ground station size: %.1f m square.'], groundStationSize));
        title(ax, {sprintf('Click once to place the ground-station centre (size %.1f m square)', groundStationSize), ...
            'The ground station is only used at this clicked location'}, ...
            'FontWeight', 'bold');
        set(ui.hStatus, 'String', 'Fields confirmed. Waiting for ground-station placement.', ...
            'ForegroundColor', [0.00 0.45 0.00]);
    end

    function [rectSpec, wasCancelled] = captureInitialRectangleInteractive(axLocal, fieldIdx, defaultLength, defaultWidth, defaultRotation)
        rectSpec = makeDefaultRectSpec(defaultGroundStationEast_m, defaultGroundStationNorth_m, defaultLength, defaultWidth, defaultRotation);
        wasCancelled = false;
        setFieldGraphicsInteractivity(false);
        restoreFieldInteractivity = onCleanup(@() restoreFieldGraphicsInteractivitySafely(fig, true));
        savedMotionFcn = get(fig, 'WindowButtonMotionFcn');
        savedButtonUpFcn = get(fig, 'WindowButtonUpFcn');
        set(fig, 'WindowButtonMotionFcn', [], 'WindowButtonUpFcn', []);
        restoreFigureCallbacks = onCleanup(@() restoreFigureCallbacksSafely(fig, savedMotionFcn, savedButtonUpFcn));

        drawRectangleAvailable = (exist('drawrectangle', 'file') == 2) || (exist('drawrectangle', 'builtin') == 5);

        if drawRectangleAvailable
            try
                roi = drawrectangle(axLocal, 'Color', [0.00 0.55 0.00], 'LineWidth', 2.0);
                wait(roi);
                if ~isvalid(roi)
                    wasCancelled = true;
                    return
                end
                position = roi.Position;
                delete(roi);
            catch ME
                if isgraphics(fig)
                    rethrow(ME);
                else
                    wasCancelled = true;
                    return
                end
            end
        else
            try
                title(axLocal, {sprintf('Draw initial rectangle for Field %d', fieldIdx), ...
                    'Click and drag the initial length and width, then release the mouse'}, ...
                    'FontWeight', 'bold');
                position = getrect(axLocal);
            catch
                wasCancelled = true;
                return
            end
        end

        rectSpec = rectanglePositionToSpec(position, defaultLength, defaultWidth, defaultRotation);
        rectSpec = sanitizeInteractiveRectangleSpec(rectSpec, fieldIdx);
    end

    function createOrReplaceField(fieldIdx, rectSpec)
        st = getappdata(fig, 'layoutState');

        if st.fields(fieldIdx).initialized
            deleteFieldGraphics(st.fields(fieldIdx));
        end

        st.fields(fieldIdx).initialized = true;
        st.fields(fieldIdx).confirmed = false;
        st.fields(fieldIdx).isEditing = true;
        st.fields(fieldIdx).spec = rectSpec;
        st.fields(fieldIdx).lastConfirmedSpec = rectSpec;

        boundaryXY = buildRectangleBoundaryXY(rectSpec.centreEast_m, rectSpec.centreNorth_m, ...
            rectSpec.length_m, rectSpec.width_m, rectSpec.rotation_deg);

        st.fields(fieldIdx).patch = patch(ax, ...
            'XData', boundaryXY(:,1), 'YData', boundaryXY(:,2), ...
            'FaceColor', [0.30 0.70 0.95], 'FaceAlpha', 0.18, ...
            'EdgeColor', [0.05 0.35 0.70], 'LineWidth', 2.2, ...
            'ButtonDownFcn', @(src, evt) onFieldGraphicButtonDown(src, evt, fieldIdx, 'patch'));

        st.fields(fieldIdx).label = text(ax, rectSpec.centreEast_m, rectSpec.centreNorth_m, ...
            buildInteractiveFieldLabel(fieldIdx, rectSpec), ...
            'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0 0], ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'HitTest', 'on', 'PickableParts', 'all', ...
            'ButtonDownFcn', @(src, evt) onFieldGraphicButtonDown(src, evt, fieldIdx, 'label'));

        st.fields(fieldIdx).centerHandle = plot(ax, rectSpec.centreEast_m, rectSpec.centreNorth_m, ...
            's', 'MarkerSize', 7, 'MarkerFaceColor', [0.05 0.35 0.70], ...
            'MarkerEdgeColor', 'w', 'Visible', 'off', ...
            'ButtonDownFcn', @(src, evt) onFieldGraphicButtonDown(src, evt, fieldIdx, 'center'));

        st.fields(fieldIdx).cornerHandles = gobjects(4,1);
        for iCorner = 1:4
            st.fields(fieldIdx).cornerHandles(iCorner) = plot(ax, NaN, NaN, 's', ...
                'MarkerSize', 8, 'MarkerFaceColor', [1.00 1.00 1.00], ...
                'MarkerEdgeColor', [0.05 0.35 0.70], 'LineWidth', 1.5, 'Visible', 'off', ...
                'ButtonDownFcn', @(src, evt) onFieldGraphicButtonDown(src, evt, fieldIdx, sprintf('corner%d', iCorner)));
        end

        st.fields(fieldIdx).rotationStem = plot(ax, NaN, NaN, '-', ...
            'Color', [0.85 0.40 0.05], 'LineWidth', 1.8, 'Visible', 'off', ...
            'HitTest', 'off');

        st.fields(fieldIdx).rotationHandle = plot(ax, NaN, NaN, 'o', ...
            'MarkerSize', 9, 'MarkerFaceColor', [1.00 1.00 1.00], ...
            'MarkerEdgeColor', [0.85 0.40 0.05], 'LineWidth', 1.8, 'Visible', 'off', ...
            'ButtonDownFcn', @(src, evt) onFieldGraphicButtonDown(src, evt, fieldIdx, 'rotate'));

        setappdata(fig, 'layoutState', st);
        updateFieldGraphics(fieldIdx);
        refreshSelectedFieldUI();
    end

    function deleteFieldGraphics(fieldStruct)
        deleteIfGraphics(fieldStruct.patch);
        deleteIfGraphics(fieldStruct.label);
        deleteIfGraphics(fieldStruct.centerHandle);
        for iCorner = 1:numel(fieldStruct.cornerHandles)
            deleteIfGraphics(fieldStruct.cornerHandles(iCorner));
        end
        deleteIfGraphics(fieldStruct.rotationStem);
        deleteIfGraphics(fieldStruct.rotationHandle);
    end

    function deleteIfGraphics(h)
        if isgraphics(h)
            delete(h);
        end
    end

    function setFieldGraphicsInteractivity(enableInteractivity)
        restoreFieldGraphicsInteractivitySafely(fig, enableInteractivity);
    end

    function setInteractiveGraphicState(h, enableInteractivity)
        if ~isgraphics(h)
            return
        end

        if enableInteractivity
            set(h, 'HitTest', 'on');
            if isprop(h, 'PickableParts')
                set(h, 'PickableParts', 'all');
            end
        else
            set(h, 'HitTest', 'off');
            if isprop(h, 'PickableParts')
                set(h, 'PickableParts', 'none');
            end
        end
    end

    function onFieldGraphicButtonDown(~, ~, fieldIdx, hitType)
        if ~isgraphics(fig)
            return
        end

        st = getappdata(fig, 'layoutState');
        if ~st.fields(fieldIdx).initialized
            return
        end

        selectionType = get(fig, 'SelectionType');

        if strcmp(selectionType, 'open')
            if st.fields(fieldIdx).isEditing && fieldIdx == st.selectedField
                confirmField(fieldIdx, true);
            else
                selectField(fieldIdx, true);
            end
            return
        end

        if ~(st.fields(fieldIdx).isEditing && fieldIdx == st.selectedField)
            selectField(fieldIdx, true);
            st = getappdata(fig, 'layoutState');
        end

        mouseXY = getCurrentAxesPoint();
        st.interaction = makeEmptyInteraction();
        st.interaction.fieldIdx = fieldIdx;
        st.interaction.mouseStartXY = mouseXY;
        st.interaction.specStart = st.fields(fieldIdx).spec;

        switch hitType
            case {'patch', 'center', 'label'}
                st.interaction.mode = 'move';
            case 'rotate'
                st.interaction.mode = 'rotate';
            otherwise
                if strncmp(hitType, 'corner', 6)
                    st.interaction.mode = 'resize_corner';
                    st.interaction.handleIdx = str2double(hitType(7:end));
                end
        end

        setappdata(fig, 'layoutState', st);
    end

    function onWindowButtonMotion(~, ~)
        if ~isgraphics(fig)
            return
        end

        st = getappdata(fig, 'layoutState');
        if ~strcmp(st.interaction.mode, 'none') && isfinite(st.interaction.fieldIdx)
            fieldIdx = st.interaction.fieldIdx;
            spec0 = st.interaction.specStart;
            mouseXY = getCurrentAxesPoint();
            specNew = spec0;

            switch st.interaction.mode
                case 'move'
                    dXY = mouseXY - st.interaction.mouseStartXY;
                    specNew.centreEast_m = spec0.centreEast_m + dXY(1);
                    specNew.centreNorth_m = spec0.centreNorth_m + dXY(2);

                case 'rotate'
                    vec = mouseXY - [spec0.centreEast_m, spec0.centreNorth_m];
                    if norm(vec) > 1e-9
                        psi = atan2(vec(2), vec(1)) - pi/2;
                        specNew.rotation_deg = rad2deg(psi);
                    end

                case 'resize_corner'
                    localPt = globalPointsToLocal(mouseXY, ...
                        [spec0.centreEast_m, spec0.centreNorth_m], deg2rad(spec0.rotation_deg));
                    specNew.length_m = max(2 * abs(localPt(1)), st.minLength_m);
                    specNew.width_m = max(2 * abs(localPt(2)), st.minWidth_m);
            end

            specNew = sanitizeInteractiveRectangleSpec(specNew, fieldIdx);
            st.fields(fieldIdx).spec = specNew;
            setappdata(fig, 'layoutState', st);
            updateFieldGraphics(fieldIdx);
            refreshSelectedFieldUI(false);
        end
    end

    function onWindowButtonUp(~, ~)
        if ~isgraphics(fig)
            return
        end
        st = getappdata(fig, 'layoutState');
        st.interaction = makeEmptyInteraction();
        setappdata(fig, 'layoutState', st);
        refreshSelectedFieldUI();
    end

    function onNumericEdit(~, ~)
        if ~isgraphics(fig)
            return
        end

        st = getappdata(fig, 'layoutState');
        fieldIdx = st.selectedField;
        if ~(isfinite(fieldIdx) && st.fields(fieldIdx).initialized)
            set(ui.hStatus, 'String', 'Select a field first from the sidebar or by double-clicking it on the map.', ...
                'ForegroundColor', [0.75 0 0]);
            return
        end

        spec = st.fields(fieldIdx).spec;

        [v, ok] = parsePositiveOrFinite(get(ui.hEditCentreEast, 'String'), true);
        if ~ok
            set(ui.hStatus, 'String', 'Centre East must be finite.', 'ForegroundColor', [0.75 0 0]);
            refreshSelectedFieldUI();
            return
        end
        spec.centreEast_m = v;

        [v, ok] = parsePositiveOrFinite(get(ui.hEditCentreNorth, 'String'), true);
        if ~ok
            set(ui.hStatus, 'String', 'Centre North must be finite.', 'ForegroundColor', [0.75 0 0]);
            refreshSelectedFieldUI();
            return
        end
        spec.centreNorth_m = v;

        [v, ok] = parsePositiveOrFinite(get(ui.hEditLength, 'String'), false);
        if ~ok
            set(ui.hStatus, 'String', 'Length must be > 0.', 'ForegroundColor', [0.75 0 0]);
            refreshSelectedFieldUI();
            return
        end
        spec.length_m = max(v, st.minLength_m);

        [v, ok] = parsePositiveOrFinite(get(ui.hEditWidth, 'String'), false);
        if ~ok
            set(ui.hStatus, 'String', 'Width must be > 0.', 'ForegroundColor', [0.75 0 0]);
            refreshSelectedFieldUI();
            return
        end
        spec.width_m = max(v, st.minWidth_m);

        [v, ok] = parsePositiveOrFinite(get(ui.hEditRotation, 'String'), true);
        if ~ok
            set(ui.hStatus, 'String', 'Rotation must be finite.', 'ForegroundColor', [0.75 0 0]);
            refreshSelectedFieldUI();
            return
        end
        spec.rotation_deg = v;

        st.fields(fieldIdx).spec = sanitizeInteractiveRectangleSpec(spec, fieldIdx);
        st.fields(fieldIdx).isEditing = true;
        st.selectedField = fieldIdx;
        setappdata(fig, 'layoutState', st);
        updateFieldGraphics(fieldIdx);
        refreshSelectedFieldUI();
        set(ui.hStatus, 'String', sprintf('Field %d updated from numeric inputs.', fieldIdx), ...
            'ForegroundColor', [0.00 0.45 0.00]);
    end

    function [value, ok] = parsePositiveOrFinite(strValue, allowAnyFinite)
        value = str2double(strValue);
        ok = isfinite(value);
        if ok && ~allowAnyFinite
            ok = value > 0;
        end
    end

    function onFieldSelectorChanged(~, ~)
        if ~isgraphics(fig)
            return
        end

        st = getappdata(fig, 'layoutState');
        if isfield(st, 'isProgrammaticFieldSelectorUpdate') && st.isProgrammaticFieldSelectorUpdate
            return
        end
        selectorValue = get(ui.hFieldSelector, 'Value');

        if selectorValue <= 1
            deselectCurrentField(true);
            return
        end

        fieldIdx = selectorValue - 1;
        if fieldIdx > numel(st.fields) || ~st.fields(fieldIdx).initialized
            set(ui.hStatus, 'String', sprintf('Field %d has not been drawn yet.', fieldIdx), ...
                'ForegroundColor', [0.75 0 0]);
            updateFieldSelectorUI();
            return
        end

        selectField(fieldIdx, true);
    end

    function selectField(fieldIdx, editMode)
        st = getappdata(fig, 'layoutState');

        if isfinite(st.selectedField) && st.selectedField ~= fieldIdx && st.fields(st.selectedField).initialized
            st.fields(st.selectedField).isEditing = false;
            oldIdx = st.selectedField;
            st.selectedField = fieldIdx;
            st.fields(fieldIdx).isEditing = editMode;
            setappdata(fig, 'layoutState', st);
            updateFieldGraphics(oldIdx);
        else
            st.selectedField = fieldIdx;
            st.fields(fieldIdx).isEditing = editMode;
            setappdata(fig, 'layoutState', st);
        end

        updateFieldGraphics(fieldIdx);
        refreshSelectedFieldUI();
        set(ui.hStatus, 'String', sprintf('Field %d selected for editing.', fieldIdx), ...
            'ForegroundColor', [0.00 0.45 0.00]);
    end

    function deselectCurrentField(clearStatus)
        st = getappdata(fig, 'layoutState');
        if isfinite(st.selectedField) && st.fields(st.selectedField).initialized
            idx = st.selectedField;
            st.fields(idx).isEditing = false;
            st.selectedField = NaN;
            setappdata(fig, 'layoutState', st);
            updateFieldGraphics(idx);
        else
            st.selectedField = NaN;
            setappdata(fig, 'layoutState', st);
        end

        set(ui.hEditCentreEast, 'String', '');
        set(ui.hEditCentreNorth, 'String', '');
        set(ui.hEditLength, 'String', '');
        set(ui.hEditWidth, 'String', '');
        set(ui.hEditRotation, 'String', '');
        set(ui.hLiveMetrics, 'String', '');
        updateFieldSelectorUI();
        updateFieldAreaSummaryUI();
        if clearStatus
            set(ui.hStatus, 'String', 'No field selected.', 'ForegroundColor', [0.10 0.10 0.10]);
        end
    end

    function refreshSelectedFieldUI(includeHeavyUI)
        if nargin < 1
            includeHeavyUI = true;
        end

        st = getappdata(fig, 'layoutState');
        fieldIdx = st.selectedField;

        if ~(isfinite(fieldIdx) && st.fields(fieldIdx).initialized)
            set(ui.hEditCentreEast, 'String', '');
            set(ui.hEditCentreNorth, 'String', '');
            set(ui.hEditLength, 'String', '');
            set(ui.hEditWidth, 'String', '');
            set(ui.hEditRotation, 'String', '');
            set(ui.hLiveMetrics, 'String', '');
            if includeHeavyUI
                updateFieldSelectorUI();
                updateFieldAreaSummaryUI();
            end
            return
        end

        spec = st.fields(fieldIdx).spec;
        area_m2 = spec.length_m * spec.width_m;
        area_ha = area_m2 / 10000;

        set(ui.hEditCentreEast, 'String', num2str(spec.centreEast_m, '%.6g'));
        set(ui.hEditCentreNorth, 'String', num2str(spec.centreNorth_m, '%.6g'));
        set(ui.hEditLength, 'String', num2str(spec.length_m, '%.6g'));
        set(ui.hEditWidth, 'String', num2str(spec.width_m, '%.6g'));
        set(ui.hEditRotation, 'String', num2str(spec.rotation_deg, '%.6g'));

        set(ui.hLiveMetrics, 'String', sprintf([ ...
            'Selected field: %d\n' ...
            'Length: %.2f m\n' ...
            'Width: %.2f m\n' ...
            'Rotation: %.2f deg\n' ...
            'Area: %.1f m^2 | %.4f ha'], ...
            fieldIdx, spec.length_m, spec.width_m, spec.rotation_deg, area_m2, area_ha));
        if includeHeavyUI
            updateFieldSelectorUI();
            updateFieldAreaSummaryUI();
        end
    end

    function updateFieldSelectorUI()
        st = getappdata(fig, 'layoutState');
        selectorEntries = buildFieldSelectorEntries();

        if isfinite(st.selectedField)
            selectorValue = st.selectedField + 1;
        else
            selectorValue = 1;
        end

        st.isProgrammaticFieldSelectorUpdate = true;
        setappdata(fig, 'layoutState', st);
        set(ui.hFieldSelector, 'String', selectorEntries, 'Value', selectorValue);
        st = getappdata(fig, 'layoutState');
        st.isProgrammaticFieldSelectorUpdate = false;
        setappdata(fig, 'layoutState', st);
    end

    function updateFieldAreaSummaryUI()
        st = getappdata(fig, 'layoutState');
        areaLines = cell(0,1);
        totalArea_ha = 0;

        for iField = 1:numel(st.fields)
            if st.fields(iField).initialized
                area_ha = (st.fields(iField).spec.length_m * st.fields(iField).spec.width_m) / 10000;
                areaLines{end+1,1} = sprintf('Field %d: %.4f ha', iField, area_ha); %#ok<AGROW>
                totalArea_ha = totalArea_ha + area_ha;
            else
                areaLines{end+1,1} = sprintf('Field %d: --', iField); %#ok<AGROW>
            end
        end

        if isempty(areaLines)
            areaLines = {'No fields drawn yet.'};
        end

        set(ui.hAreaSummary, 'String', areaLines);
        set(ui.hTotalAreaValue, 'String', sprintf('%.4f ha', totalArea_ha));
    end

    function selectorEntries = buildFieldSelectorEntries()
        st = getappdata(fig, 'layoutState');
        selectorEntries = cell(fieldCount + 1, 1);
        selectorEntries{1} = 'No field selected';

        fieldsInitialized = false(fieldCount, 1);
        fieldSpecs = repmat(makeDefaultRectSpec(NaN, NaN, NaN, NaN, NaN), fieldCount, 1);
        fieldIsEditing = false(fieldCount, 1);
        fieldIsConfirmed = false(fieldCount, 1);

        if isstruct(st) && isfield(st, 'fields') && numel(st.fields) == fieldCount
            for iField = 1:fieldCount
                fieldsInitialized(iField) = st.fields(iField).initialized;
                fieldSpecs(iField) = st.fields(iField).spec;
                fieldIsEditing(iField) = st.fields(iField).isEditing;
                fieldIsConfirmed(iField) = st.fields(iField).confirmed;
            end
        end

        for iField = 1:fieldCount
            if fieldsInitialized(iField)
                area_ha = (fieldSpecs(iField).length_m * fieldSpecs(iField).width_m) / 10000;
                if fieldIsEditing(iField)
                    fieldState = 'editing';
                elseif fieldIsConfirmed(iField)
                    fieldState = 'confirmed';
                else
                    fieldState = 'drawn';
                end
                selectorEntries{iField + 1} = sprintf('Field %d (%.4f ha, %s)', ...
                    iField, area_ha, fieldState);
            else
                selectorEntries{iField + 1} = sprintf('Field %d (not drawn yet)', iField);
            end
        end
    end

    function confirmField(fieldIdx, resumeIfWaiting)
        st = getappdata(fig, 'layoutState');
        if ~(st.fields(fieldIdx).initialized)
            return
        end

        st.fields(fieldIdx).confirmed = true;
        st.fields(fieldIdx).lastConfirmedSpec = st.fields(fieldIdx).spec;
        st.fields(fieldIdx).isEditing = false;
        st.interaction = makeEmptyInteraction();

        if strcmp(st.stage, 'placement') && st.waitingFieldIdx == fieldIdx
            st.lastPlacementAction = 'confirmed';
            msg = sprintf('Field %d confirmed. Moving to the next field.', fieldIdx);
        else
            msg = sprintf('Field %d confirmed.', fieldIdx);
        end

        setappdata(fig, 'layoutState', st);
        updateFieldGraphics(fieldIdx);
        refreshSelectedFieldUI();
        set(ui.hStatus, 'String', msg, 'ForegroundColor', [0.00 0.45 0.00]);

        if resumeIfWaiting && strcmp(st.stage, 'placement') && st.waitingFieldIdx == fieldIdx
            uiresume(fig);
        end
    end

    function cancelCurrentField()
        st = getappdata(fig, 'layoutState');
        fieldIdx = st.selectedField;
        if ~(isfinite(fieldIdx) && st.fields(fieldIdx).initialized)
            return
        end

        if strcmp(st.stage, 'placement') && st.waitingFieldIdx == fieldIdx
            deleteFieldGraphics(st.fields(fieldIdx));
            st.fields(fieldIdx) = makeEmptyInteractiveField();
            st.selectedField = NaN;
            st.interaction = makeEmptyInteraction();
            st.lastPlacementAction = 'cancelled';
            setappdata(fig, 'layoutState', st);
            deselectCurrentField(false);
            set(ui.hStatus, 'String', sprintf('Field %d cancelled. Redraw it.', fieldIdx), ...
                'ForegroundColor', [0.75 0 0]);
            uiresume(fig);
            return
        end

        st.fields(fieldIdx).spec = st.fields(fieldIdx).lastConfirmedSpec;
        st.fields(fieldIdx).isEditing = false;
        st.interaction = makeEmptyInteraction();
        setappdata(fig, 'layoutState', st);
        updateFieldGraphics(fieldIdx);
        deselectCurrentField(false);
        set(ui.hStatus, 'String', sprintf('Field %d edit cancelled. Reverted to last confirmed version.', fieldIdx), ...
            'ForegroundColor', [0.75 0 0]);
    end

    function onConfirmAllFields(~, ~)
        if ~isgraphics(fig)
            return
        end
        st = getappdata(fig, 'layoutState');

        if any(~[st.fields.initialized])
            set(ui.hStatus, 'String', 'All fields must exist before confirming the layout.', ...
                'ForegroundColor', [0.75 0 0]);
            return
        end

        for iField = 1:numel(st.fields)
            st.fields(iField).confirmed = true;
            st.fields(iField).lastConfirmedSpec = st.fields(iField).spec;
            st.fields(iField).isEditing = false;
        end
        st.selectedField = NaN;
        st.interaction = makeEmptyInteraction();
        st.finalFieldsConfirmed = true;
        setappdata(fig, 'layoutState', st);

        for iField = 1:numel(st.fields)
            updateFieldGraphics(iField);
        end
        deselectCurrentField(false);
        set(ui.hStatus, 'String', 'Field layout confirmed. Proceeding to ground-station placement.', ...
            'ForegroundColor', [0.00 0.45 0.00]);
        uiresume(fig);
    end

    function updateFieldGraphics(fieldIdx)
        st = getappdata(fig, 'layoutState');
        if ~(st.fields(fieldIdx).initialized)
            return
        end

        spec = st.fields(fieldIdx).spec;
        boundaryXY = buildRectangleBoundaryXY(spec.centreEast_m, spec.centreNorth_m, ...
            spec.length_m, spec.width_m, spec.rotation_deg);

        if isgraphics(st.fields(fieldIdx).patch)
            set(st.fields(fieldIdx).patch, 'XData', boundaryXY(:,1), 'YData', boundaryXY(:,2));
        end
        if isgraphics(st.fields(fieldIdx).label)
            set(st.fields(fieldIdx).label, 'Position', [spec.centreEast_m, spec.centreNorth_m, 0], ...
                'String', buildInteractiveFieldLabel(fieldIdx, spec));
        end
        if isgraphics(st.fields(fieldIdx).centerHandle)
            set(st.fields(fieldIdx).centerHandle, 'XData', spec.centreEast_m, 'YData', spec.centreNorth_m);
        end

        localCorners = [ ...
            -spec.length_m/2, -spec.width_m/2; ...
             spec.length_m/2, -spec.width_m/2; ...
             spec.length_m/2,  spec.width_m/2; ...
            -spec.length_m/2,  spec.width_m/2];
        cornerXY = localPointsToGlobal(localCorners, [spec.centreEast_m, spec.centreNorth_m], deg2rad(spec.rotation_deg));
        for iCorner = 1:4
            if isgraphics(st.fields(fieldIdx).cornerHandles(iCorner))
                set(st.fields(fieldIdx).cornerHandles(iCorner), ...
                    'XData', cornerXY(iCorner,1), 'YData', cornerXY(iCorner,2));
            end
        end

        localRotationBase = [0, spec.width_m/2];
        localRotationTip = [0, spec.width_m/2 + st.rotationHandleOffset_m];
        rotationBaseXY = localPointsToGlobal(localRotationBase, ...
            [spec.centreEast_m, spec.centreNorth_m], deg2rad(spec.rotation_deg));
        rotationTipXY = localPointsToGlobal(localRotationTip, ...
            [spec.centreEast_m, spec.centreNorth_m], deg2rad(spec.rotation_deg));

        if isgraphics(st.fields(fieldIdx).rotationStem)
            set(st.fields(fieldIdx).rotationStem, 'XData', [rotationBaseXY(1), rotationTipXY(1)], ...
                'YData', [rotationBaseXY(2), rotationTipXY(2)]);
        end
        if isgraphics(st.fields(fieldIdx).rotationHandle)
            set(st.fields(fieldIdx).rotationHandle, 'XData', rotationTipXY(1), 'YData', rotationTipXY(2));
        end

        isSelected = isfinite(st.selectedField) && st.selectedField == fieldIdx;
        isEditing = st.fields(fieldIdx).isEditing;
        isConfirmed = st.fields(fieldIdx).confirmed;

        if isSelected && isEditing
            faceColor = [0.30 0.70 0.95];
            edgeColor = [0.05 0.35 0.70];
            faceAlpha = 0.20;
            handleVisible = 'on';
            labelColor = [0.05 0.35 0.70];
            lineWidth = 2.4;
        elseif isConfirmed
            faceColor = [0.60 0.90 0.60];
            edgeColor = [0.00 0.35 0.00];
            faceAlpha = 0.26;
            handleVisible = 'off';
            labelColor = [0.00 0.30 0.00];
            lineWidth = 2.0;
        else
            faceColor = [0.92 0.85 0.45];
            edgeColor = [0.55 0.45 0.05];
            faceAlpha = 0.22;
            handleVisible = 'off';
            labelColor = [0.30 0.25 0.00];
            lineWidth = 2.0;
        end

        if isgraphics(st.fields(fieldIdx).patch)
            set(st.fields(fieldIdx).patch, 'FaceColor', faceColor, 'EdgeColor', edgeColor, ...
                'FaceAlpha', faceAlpha, 'LineWidth', lineWidth);
        end
        if isgraphics(st.fields(fieldIdx).label)
            set(st.fields(fieldIdx).label, 'Color', labelColor);
        end
        if isgraphics(st.fields(fieldIdx).centerHandle)
            set(st.fields(fieldIdx).centerHandle, 'Visible', handleVisible);
        end
        for iCorner = 1:4
            if isgraphics(st.fields(fieldIdx).cornerHandles(iCorner))
                set(st.fields(fieldIdx).cornerHandles(iCorner), 'Visible', handleVisible);
            end
        end
        if isgraphics(st.fields(fieldIdx).rotationStem)
            set(st.fields(fieldIdx).rotationStem, 'Visible', handleVisible);
        end
        if isgraphics(st.fields(fieldIdx).rotationHandle)
            set(st.fields(fieldIdx).rotationHandle, 'Visible', handleVisible);
        end

        drawnow limitrate
    end

    function mouseXY = getCurrentAxesPoint()
        cp = get(ax, 'CurrentPoint');
        mouseXY = cp(1,1:2);
    end

    function onKeyPress(~, evt)
        if ~isgraphics(fig)
            return
        end
        if strcmpi(evt.Key, 'escape')
            cancelCurrentField();
        end
    end

    function onCloseRequest(src, ~)
        if isgraphics(src)
            st = getappdata(src, 'layoutState');
            st.setupCancelled = true;
            setappdata(src, 'layoutState', st);
            uiresume(src);
            delete(src);
        end
    end
end

function rectSpec = rectanglePositionToSpec(position, defaultLength_m, defaultWidth_m, defaultRotation_deg)
    assert(numel(position) == 4, 'Rectangle position must contain [x y w h].');

    xMin = position(1);
    yMin = position(2);
    widthX = abs(position(3));
    widthY = abs(position(4));

    centreEast_m = xMin + 0.5 * position(3);
    centreNorth_m = yMin + 0.5 * position(4);

    if widthX <= 1e-9
        widthX = defaultLength_m;
    end
    if widthY <= 1e-9
        widthY = defaultWidth_m;
    end

    rectSpec = struct();
    rectSpec.centreEast_m = centreEast_m;
    rectSpec.centreNorth_m = centreNorth_m;
    rectSpec.length_m = widthX;
    rectSpec.width_m = widthY;
    rectSpec.rotation_deg = defaultRotation_deg;
end

function rectSpec = sanitizeInteractiveRectangleSpec(rectSpec, fieldNumber)
    assert(isfinite(rectSpec.centreEast_m), 'Field %d centreEast_m must be finite.', fieldNumber);
    assert(isfinite(rectSpec.centreNorth_m), 'Field %d centreNorth_m must be finite.', fieldNumber);
    assert(isfinite(rectSpec.length_m) && rectSpec.length_m > 0, ...
        'Field %d length must be > 0.', fieldNumber);
    assert(isfinite(rectSpec.width_m) && rectSpec.width_m > 0, ...
        'Field %d width must be > 0.', fieldNumber);
    assert(isfinite(rectSpec.rotation_deg), 'Field %d rotation must be finite.', fieldNumber);
end

function boundaryXY = buildRectangleBoundaryXY(centreEast_m, centreNorth_m, length_m, width_m, rotation_deg)
    localXY = [ ...
        -length_m/2, -width_m/2; ...
         length_m/2, -width_m/2; ...
         length_m/2,  width_m/2; ...
        -length_m/2,  width_m/2];
    boundaryXY = localPointsToGlobal(localXY, [centreEast_m, centreNorth_m], deg2rad(rotation_deg));
end

function [xPoint, yPoint] = capturePointInteractive(ax)
    drawPointAvailable = (exist('drawpoint', 'file') == 2) || (exist('drawpoint', 'builtin') == 5);

    if drawPointAvailable
        roi = drawpoint(ax, 'Color', [0.85 0.35 0.05], 'MarkerSize', 8);
        wait(roi);
        if ~isvalid(roi)
            error('Ground-station placement was cancelled.');
        end
        pos = roi.Position;
        delete(roi);
        xPoint = pos(1);
        yPoint = pos(2);
    else
        [xPoint, yPoint, button] = ginput(1);
        if isempty(button)
            error('Ground-station placement was cancelled.');
        end
    end
end

function fieldDef = createRectangleFieldDef(name, centreEast_m, centreNorth_m, length_m, width_m, rotation_deg)
    fieldDef = emptyFieldDefTemplate();
    fieldDef.name = name;
    fieldDef.type = 'rectangle';
    fieldDef.centreEast_m = centreEast_m;
    fieldDef.centreNorth_m = centreNorth_m;
    fieldDef.length_m = length_m;
    fieldDef.width_m = width_m;
    fieldDef.rotation_deg = rotation_deg;
end

function fieldDef = emptyFieldDefTemplate()
    fieldDef = struct( ...
        'name', '', ...
        'type', '', ...
        'centreEast_m', NaN, ...
        'centreNorth_m', NaN, ...
        'length_m', NaN, ...
        'width_m', NaN, ...
        'rotation_deg', NaN, ...
        'boundaryXY', zeros(0,2));
end

function labelText = buildInteractiveFieldLabel(fieldIdx, spec)
    area_m2 = spec.length_m * spec.width_m;
    area_ha = area_m2 / 10000;
    labelText = sprintf('Field %d\n%.1f x %.1f m\n%.0f m^2 | %.3f ha', ...
        fieldIdx, spec.length_m, spec.width_m, area_m2, area_ha);
end

function globalXY = localPointsToGlobal(localXY, originXY, psi)
    wasRowVector = isvector(localXY) && numel(localXY) == 2;
    localXY = reshape(localXY, [], 2);
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    globalXY = (R * localXY.').';
    globalXY(:,1) = globalXY(:,1) + originXY(1);
    globalXY(:,2) = globalXY(:,2) + originXY(2);
    if wasRowVector
        globalXY = globalXY(1,:);
    end
end

function localXY = globalPointsToLocal(globalXY, originXY, psi)
    wasRowVector = isvector(globalXY) && numel(globalXY) == 2;
    globalXY = reshape(globalXY, [], 2);
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    shifted = globalXY - originXY;
    localXY = (R.' * shifted.').';
    if wasRowVector
        localXY = localXY(1,:);
    end
end

function restoreFieldGraphicsInteractivitySafely(fig, enableInteractivity)
    if ~isgraphics(fig)
        return
    end

    st = getappdata(fig, 'layoutState');
    if ~isstruct(st) || ~isfield(st, 'fields')
        return
    end

    for iField = 1:numel(st.fields)
        if ~isfield(st.fields(iField), 'initialized') || ~st.fields(iField).initialized
            continue
        end

        setInteractiveGraphicStateSafely(st.fields(iField).patch, enableInteractivity);
        setInteractiveGraphicStateSafely(st.fields(iField).label, enableInteractivity);
        setInteractiveGraphicStateSafely(st.fields(iField).centerHandle, enableInteractivity);

        if isfield(st.fields(iField), 'cornerHandles')
            for iCorner = 1:numel(st.fields(iField).cornerHandles)
                setInteractiveGraphicStateSafely(st.fields(iField).cornerHandles(iCorner), enableInteractivity);
            end
        end

        setInteractiveGraphicStateSafely(st.fields(iField).rotationHandle, enableInteractivity);
    end
end

function setInteractiveGraphicStateSafely(h, enableInteractivity)
    if ~isgraphics(h)
        return
    end

    try
        if enableInteractivity
            set(h, 'HitTest', 'on');
            if isprop(h, 'PickableParts')
                set(h, 'PickableParts', 'all');
            end
        else
            set(h, 'HitTest', 'off');
            if isprop(h, 'PickableParts')
                set(h, 'PickableParts', 'none');
            end
        end
    catch
        % Ignore teardown-time graphics errors when the figure is closing.
    end
end

function restoreFigureCallbacksSafely(fig, motionFcn, buttonUpFcn)
    if ~isgraphics(fig)
        return
    end

    try
        set(fig, 'WindowButtonMotionFcn', motionFcn, 'WindowButtonUpFcn', buttonUpFcn);
    catch
        % Ignore teardown-time figure callback restoration errors.
    end
end

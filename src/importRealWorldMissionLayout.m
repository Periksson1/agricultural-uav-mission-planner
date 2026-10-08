function [fields, fieldCount, fieldVisitOrder, groundStationEast_m, groundStationNorth_m, missionGeodata] = ...
    importRealWorldMissionLayout(defaultGroundStationEast_m, defaultGroundStationNorth_m, groundStationSize, fieldPath, selectGroundStation)
% importRealWorldMissionLayout Import real-world field inputs for mission planning.
%
% Supported field boundary uploads:
%   - GeoJSON / JSON Polygon or MultiPolygon files in lon/lat or projected XY.
%   - CSV files with lon/lat or east/north columns, optionally grouped by a
%     field/id/name column.
% Optional fieldPath and selectGroundStation inputs support repeatable runs
% without opening file-selection or ground-station-placement dialogs.

    if nargin < 4, fieldPath = ''; end
    if nargin < 5, selectGroundStation = usejava('desktop'); end
    if isempty(fieldPath)
        if usejava('desktop')
            [fileName, folderName] = uigetfile( ...
                {'*.geojson;*.json;*.csv', 'Field boundary geodata (*.geojson, *.json, *.csv)'; ...
                 '*.*', 'All files (*.*)'}, ...
                'Select field boundary geodata');
            if isequal(fileName, 0)
                error('Real-world field import was cancelled.');
            end
            fieldPath = fullfile(folderName, fileName);
        else
            fieldPath = input('Field boundary file path: ', 's');
            assert(~isempty(fieldPath), 'A field boundary file path is required.');
        end
    end
    assert(isfile(fieldPath), 'Field boundary file not found: %s', fieldPath);

    [fieldPolygons, fieldNames, coordInfo] = readFieldBoundaryFile(fieldPath);
    assert(~isempty(fieldPolygons), 'No field polygons were found in %s.', fieldPath);

    fields = repmat(emptyFieldDefTemplate(), numel(fieldPolygons), 1);
    for iField = 1:numel(fieldPolygons)
        fields(iField) = createPolygonFieldDef(fieldNames{iField}, fieldPolygons{iField});
    end

    fieldCount = numel(fields);
    fieldVisitOrder = 1:fieldCount;

    allFieldCoords = vertcat(fieldPolygons{:});
    allXY = allFieldCoords(:,1:2);
    if isempty(allFieldCoords)
        groundStationEast_m = defaultGroundStationEast_m;
        groundStationNorth_m = defaultGroundStationNorth_m;
    else
        fieldMin = min(allXY, [], 1);
        fieldMax = max(allXY, [], 1);
        fieldSpan = max(fieldMax - fieldMin);
        if ~isfinite(fieldSpan) || fieldSpan <= 0
            fieldSpan = 100;
        end
        groundStationEast_m = fieldMax(1) + max(groundStationSize * 2, 0.08 * fieldSpan);
        groundStationNorth_m = mean([fieldMin(2), fieldMax(2)]);
    end

    missionGeodata = struct();
    missionGeodata.sourceFieldPath = fieldPath;
    missionGeodata.coordinateInfo = coordInfo;
    missionGeodata.groundImage = struct();
    missionGeodata.terrainModel = struct();

    if selectGroundStation && usejava('desktop')
        [groundStationEast_m, groundStationNorth_m] = selectGroundStationOnImportedFields( ...
            fields, allXY, groundStationEast_m, groundStationNorth_m, groundStationSize);
    end

    missionGeodata.groundStationElevation_m = 0;
end

function [fieldPolygons, fieldNames, coordInfo] = readFieldBoundaryFile(fieldPath)
    [~,~,ext] = fileparts(fieldPath);
    switch lower(ext)
        case {'.geojson', '.json'}
            [rawPolygons, fieldNames] = readGeoJsonPolygons(fieldPath);
        case '.csv'
            [rawPolygons, fieldNames] = readCsvPolygons(fieldPath);
        otherwise
            error('Unsupported field boundary file type "%s". Use GeoJSON, JSON, or CSV.', ext);
    end

    allRaw = vertcat(rawPolygons{:});
    coordInfo = inferCoordinateInfo(allRaw);
    fieldPolygons = cell(size(rawPolygons));
    for i = 1:numel(rawPolygons)
        fieldPolygons{i} = coordinatesToLocalXY(rawPolygons{i}, coordInfo);
        fieldPolygons{i} = removeDuplicateClosingVertex(fieldPolygons{i});
    end
end

function [polygons, names] = readGeoJsonPolygons(fieldPath)
    geo = jsondecode(fileread(fieldPath));
    if isfield(geo, 'type') && strcmpi(geo.type, 'FeatureCollection')
        features = structArrayToCell(geo.features);
    elseif isfield(geo, 'type') && strcmpi(geo.type, 'Feature')
        features = {geo};
    elseif isfield(geo, 'type')
        features = {struct('type', 'Feature', 'geometry', geo, 'properties', struct())};
    else
        error('The JSON file does not look like GeoJSON.');
    end

    polygons = {};
    names = {};
    for iFeature = 1:numel(features)
        feature = features{iFeature};
        if ~isfield(feature, 'geometry') || isempty(feature.geometry)
            continue
        end
        featureName = sprintf('Field %d', numel(polygons) + 1);
        if isfield(feature, 'properties')
            featureName = getFeatureName(feature.properties, featureName);
        end

        geometry = feature.geometry;
        switch lower(geometry.type)
            case 'polygon'
                rings = extractGeoJsonPolygonRings(geometry.coordinates);
                if ~isempty(rings)
                    polygons{end+1,1} = rings{1}; %#ok<AGROW>
                    names{end+1,1} = featureName; %#ok<AGROW>
                end
            case 'multipolygon'
                polygonCells = structArrayToCell(geometry.coordinates);
                for iPolygon = 1:numel(polygonCells)
                    rings = extractGeoJsonPolygonRings(polygonCells{iPolygon});
                    if ~isempty(rings)
                        polygons{end+1,1} = rings{1}; %#ok<AGROW>
                        names{end+1,1} = sprintf('%s %d', featureName, iPolygon); %#ok<AGROW>
                    end
                end
            otherwise
                warning('Skipping GeoJSON geometry type "%s".', geometry.type);
        end
    end
end

function [polygons, names] = readCsvPolygons(fieldPath)
    tbl = readtable(fieldPath);
    namesLower = lower(tbl.Properties.VariableNames);
    lonIdx = findFirstName(namesLower, {'lon','lng','longitude'});
    latIdx = findFirstName(namesLower, {'lat','latitude'});
    eastIdx = findFirstName(namesLower, {'east','easting','x'});
    northIdx = findFirstName(namesLower, {'north','northing','y'});
    elevIdx = findFirstName(namesLower, {'elevation','elev','z','height','altitude'});
    groupIdx = findFirstName(namesLower, {'field','fieldid','field_id','id','name'});

    if ~isnan(lonIdx) && ~isnan(latIdx)
        coords = [tbl{:,lonIdx}, tbl{:,latIdx}];
    elseif ~isnan(eastIdx) && ~isnan(northIdx)
        coords = [tbl{:,eastIdx}, tbl{:,northIdx}];
    else
        error('CSV must contain lon/lat or east/north columns.');
    end
    if ~isnan(elevIdx)
        coords = [coords, tbl{:,elevIdx}];
    end

    valid = all(isfinite(coords(:,1:2)), 2);
    coords = coords(valid,:);
    if isnan(groupIdx)
        groups = ones(size(coords,1),1);
    else
        rawGroups = tbl{valid,groupIdx};
        if isnumeric(rawGroups)
            groups = rawGroups;
        else
            [~,~,groups] = unique(string(rawGroups), 'stable');
        end
    end

    uniqueGroups = unique(groups(:), 'stable');
    polygons = cell(numel(uniqueGroups), 1);
    names = cell(numel(uniqueGroups), 1);
    for iGroup = 1:numel(uniqueGroups)
        idx = groups == uniqueGroups(iGroup);
        polygons{iGroup} = coords(idx,:);
        names{iGroup} = sprintf('Field %d', iGroup);
    end
end

function [groundStationEast_m, groundStationNorth_m] = selectGroundStationOnImportedFields( ...
    fields, allXY, defaultGroundStationEast_m, defaultGroundStationNorth_m, groundStationSize)

    fig = figure('Color', 'w', ...
        'Name', 'Imported field setup', ...
        'NumberTitle', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on')
    grid(ax, 'on')
    axis(ax, 'equal')
    xlabel(ax, 'East [m]')
    ylabel(ax, 'North [m]')
    title(ax, {sprintf('Click once to place the ground-station centre (%.1f m square)', groundStationSize), ...
        'Imported field boundaries are shown before mission planning starts'}, ...
        'FontWeight', 'bold')

    fieldFaceColor = [0.60 0.90 0.60];
    allPlotXY = allXY;
    for iField = 1:numel(fields)
        boundaryXY = fields(iField).boundaryXY;
        if isempty(boundaryXY) || size(boundaryXY,1) < 3
            continue
        end
        fill(ax, boundaryXY(:,1), boundaryXY(:,2), fieldFaceColor, ...
            'EdgeColor', 'k', 'LineWidth', 2.0, 'FaceAlpha', 0.25);
        centreXY = mean(boundaryXY, 1);
        text(ax, centreXY(1), centreXY(2), fields(iField).name, ...
            'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'Color', [0 0 0], 'HitTest', 'off');
    end

    gsHalf = groundStationSize / 2;
    defaultGsX = defaultGroundStationEast_m + gsHalf * [-1 1 1 -1];
    defaultGsY = defaultGroundStationNorth_m + gsHalf * [-1 -1 1 1];
    fill(ax, defaultGsX, defaultGsY, [1.00 0.83 0.25], ...
        'FaceAlpha', 0.65, 'EdgeColor', [0.60 0.40 0.00], 'LineWidth', 1.5, ...
        'LineStyle', '--');
    text(ax, defaultGroundStationEast_m, defaultGroundStationNorth_m, ...
        '  Suggested GS', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.45 0.28 0.00], 'HitTest', 'off');

    allPlotXY = [allPlotXY; defaultGsX(:), defaultGsY(:)];
    setAxesAroundPoints(ax, allPlotXY);

    [groundStationEast_m, groundStationNorth_m] = capturePointInteractive(ax);

    gsX = groundStationEast_m + gsHalf * [-1 1 1 -1];
    gsY = groundStationNorth_m + gsHalf * [-1 -1 1 1];
    fill(ax, gsX, gsY, [1.00 0.83 0.25], ...
        'FaceAlpha', 0.85, 'EdgeColor', [0.60 0.40 0.00], 'LineWidth', 2.0);
    text(ax, groundStationEast_m, groundStationNorth_m, '  Ground station', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.45 0.28 0.00], ...
        'HitTest', 'off');
    title(ax, {'Ground station selected', 'Continuing directly into mission planning'}, ...
        'FontWeight', 'bold')
    drawnow
    pause(0.15);

    if isgraphics(fig)
        delete(fig);
    end
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

function setAxesAroundPoints(ax, xy)
    if isempty(xy)
        return
    end
    xMin = min(xy(:,1));
    xMax = max(xy(:,1));
    yMin = min(xy(:,2));
    yMax = max(xy(:,2));
    xRange = max(xMax - xMin, 1);
    yRange = max(yMax - yMin, 1);
    margin = max(10, 0.08 * max(xRange, yRange));
    xlim(ax, [xMin - margin, xMax + margin])
    ylim(ax, [yMin - margin, yMax + margin])
end

function coordInfo = inferCoordinateInfo(coords)
    isLonLat = all(abs(coords(:,1)) <= 180) && all(abs(coords(:,2)) <= 90);
    coordInfo = struct();
    coordInfo.isGeographic = isLonLat;
    if isLonLat
        coordInfo.originLon_deg = mean(coords(:,1), 'omitnan');
        coordInfo.originLat_deg = mean(coords(:,2), 'omitnan');
        coordInfo.earthRadius_m = 6371008.8;
        coordInfo.description = 'Geographic lon/lat converted to local tangent East/North metres.';
    else
        coordInfo.originLon_deg = NaN;
        coordInfo.originLat_deg = NaN;
        coordInfo.earthRadius_m = NaN;
        coordInfo.description = 'Projected XY interpreted directly as East/North metres.';
    end
end

function xy = coordinatesToLocalXY(coords, coordInfo)
    if coordInfo.isGeographic
        lon = coords(:,1);
        lat = coords(:,2);
        east = deg2rad(lon - coordInfo.originLon_deg) .* coordInfo.earthRadius_m .* cosd(coordInfo.originLat_deg);
        north = deg2rad(lat - coordInfo.originLat_deg) .* coordInfo.earthRadius_m;
        xy = [east, north];
    else
        xy = coords(:,1:2);
    end
    if size(coords,2) >= 3
        xy = [xy, coords(:,3)];
    end
end

function rings = extractGeoJsonPolygonRings(coords)
    rings = {};
    if isnumeric(coords)
        if ismatrix(coords) && size(coords,2) >= 2
            rings = {coords(:,1:min(size(coords,2),3))};
        elseif ndims(coords) == 3
            for iRing = 1:size(coords,1)
                ring = squeeze(coords(iRing,:,:));
                if size(ring,2) >= 2
                    rings{end+1,1} = ring(:,1:min(size(ring,2),3)); %#ok<AGROW>
                end
            end
        end
    elseif iscell(coords)
        for iRing = 1:numel(coords)
            ring = extractCoordinateMatrix(coords{iRing});
            if size(ring,1) >= 3
                rings{end+1,1} = ring(:,1:min(size(ring,2),3)); %#ok<AGROW>
            end
        end
    end
end

function coords = extractCoordinateMatrix(value)
    if isnumeric(value)
        coords = value;
        if size(coords,2) < 2 && size(coords,1) >= 2
            coords = coords.';
        end
    elseif iscell(value)
        firstPoint = value{1};
        if iscell(firstPoint)
            firstPoint = cell2mat(firstPoint);
        end
        coordCount = min(numel(firstPoint), 3);
        coords = zeros(numel(value), coordCount);
        for i = 1:numel(value)
            point = value{i};
            if iscell(point)
                point = cell2mat(point);
            end
            coords(i,:) = point(1:coordCount);
        end
    else
        coords = zeros(0,2);
    end
end

function featureName = getFeatureName(properties, fallbackName)
    featureName = fallbackName;
    candidateNames = {'name', 'Name', 'field', 'Field', 'id', 'ID'};
    for i = 1:numel(candidateNames)
        if isfield(properties, candidateNames{i})
            value = properties.(candidateNames{i});
            if ischar(value) || isstring(value) || isnumeric(value)
                featureName = char(string(value));
                return
            end
        end
    end
end

function c = structArrayToCell(value)
    if iscell(value)
        c = value;
    elseif isstruct(value)
        c = num2cell(value);
    else
        c = num2cell(value);
    end
end

function idx = findFirstName(namesLower, candidates)
    idx = NaN;
    for i = 1:numel(candidates)
        found = find(strcmp(namesLower, candidates{i}), 1, 'first');
        if ~isempty(found)
            idx = found;
            return
        end
    end
end

function polygonXY = removeDuplicateClosingVertex(polygonXY)
    if size(polygonXY,1) >= 2 && norm(polygonXY(1,1:2) - polygonXY(end,1:2)) <= 1e-9
        polygonXY = polygonXY(1:end-1,:);
    end
end

function fieldDef = createPolygonFieldDef(name, boundaryCoords)
    boundaryCoords = removeDuplicateClosingVertex(boundaryCoords);
    boundaryXY = boundaryCoords(:,1:2);
    assert(size(boundaryXY,1) >= 3 && size(boundaryXY,2) == 2, ...
        'Imported field "%s" must contain at least three XY vertices.', name);
    fieldDef = emptyFieldDefTemplate();
    fieldDef.name = name;
    fieldDef.type = 'polygon';
    fieldDef.boundaryXY = boundaryXY;
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

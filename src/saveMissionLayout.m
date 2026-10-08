function saveMissionLayout(layoutPath, fields, fieldVisitOrder, groundStationEast_m, groundStationNorth_m, missionGeodata)
    if nargin < 6 || isempty(missionGeodata)
        missionGeodata = struct();
    end

    assert(~isempty(layoutPath), 'layoutPath must be a non-empty path.');
    assert(isstruct(fields) && ~isempty(fields), 'fields must be a non-empty struct array.');

    fieldCount = numel(fields);
    if nargin < 3 || isempty(fieldVisitOrder)
        fieldVisitOrder = (1:fieldCount).';
    end
    fieldVisitOrder = fieldVisitOrder(:);

    assert(numel(fieldVisitOrder) == fieldCount, ...
        'fieldVisitOrder must contain one entry per field.');
    assert(all(sort(fieldVisitOrder(:).') == 1:fieldCount), ...
        'fieldVisitOrder must be a permutation of 1:fieldCount.');

    saveFolder = fileparts(layoutPath);
    if ~isempty(saveFolder) && ~isfolder(saveFolder)
        mkdir(saveFolder);
    end

    savedLayout = struct();
    savedLayout.formatVersion = 1;
    savedLayout.savedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    savedLayout.fields = fields;
    savedLayout.fieldCount = fieldCount;
    savedLayout.fieldVisitOrder = fieldVisitOrder;
    savedLayout.groundStationEast_m = groundStationEast_m;
    savedLayout.groundStationNorth_m = groundStationNorth_m;
    savedLayout.missionGeodata = missionGeodata;

    save(layoutPath, 'savedLayout', '-v7');
end

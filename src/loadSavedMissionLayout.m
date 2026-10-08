function [fields, fieldCount, fieldVisitOrder, groundStationEast_m, groundStationNorth_m, missionGeodata] = ...
    loadSavedMissionLayout(layoutPath)

    assert(~isempty(layoutPath), 'layoutPath must be a non-empty path.');
    assert(isfile(layoutPath), 'Saved mission layout file not found: %s', layoutPath);

    loaded = load(layoutPath);
    if isfield(loaded, 'savedLayout') && isstruct(loaded.savedLayout)
        savedLayout = loaded.savedLayout;
    else
        savedLayout = loaded;
    end

    requiredFields = {'fields', 'groundStationEast_m', 'groundStationNorth_m'};
    assertRequiredFields(savedLayout, requiredFields, 'savedLayout');

    fields = savedLayout.fields;
    assert(isstruct(fields) && ~isempty(fields), ...
        'Saved mission layout must contain a non-empty struct array of fields.');

    fieldCount = numel(fields);
    if isfield(savedLayout, 'fieldCount') && ~isempty(savedLayout.fieldCount)
        assert(savedLayout.fieldCount == fieldCount, ...
            'Saved mission layout fieldCount does not match the number of fields.');
    end

    if isfield(savedLayout, 'fieldVisitOrder') && ~isempty(savedLayout.fieldVisitOrder)
        fieldVisitOrder = savedLayout.fieldVisitOrder(:);
    else
        fieldVisitOrder = (1:fieldCount).';
    end
    assert(numel(fieldVisitOrder) == fieldCount, ...
        'Saved mission layout fieldVisitOrder must contain one entry per field.');
    assert(all(sort(fieldVisitOrder(:).') == 1:fieldCount), ...
        'Saved mission layout fieldVisitOrder must be a permutation of 1:fieldCount.');

    groundStationEast_m = savedLayout.groundStationEast_m;
    groundStationNorth_m = savedLayout.groundStationNorth_m;

    if isfield(savedLayout, 'missionGeodata') && isstruct(savedLayout.missionGeodata)
        missionGeodata = savedLayout.missionGeodata;
    else
        missionGeodata = struct();
    end
end

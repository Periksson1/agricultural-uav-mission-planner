function assertRequiredFields(s, requiredFields, structName)
    assert(isstruct(s), '%s must be a struct.', structName);

    missing = requiredFields(~isfield(s, requiredFields));
    assert(isempty(missing), '%s is missing required fields: %s', ...
        structName, strjoin(missing, ', '));
end

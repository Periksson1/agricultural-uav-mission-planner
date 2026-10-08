# Field inputs and coordinates

`runMission` uses `examples/data/SAFarmboundaries.geojson` by default. The file contains five polygons. The importer converts geographic longitude/latitude to a local East/North frame using a spherical local approximation. The origin is calculated from the supplied coordinate samples; changing the input file can change that origin.

## GeoJSON

Supported geometry types are Polygon and MultiPolygon, either directly or within Feature/FeatureCollection objects. Each MultiPolygon component becomes a separate field. **Only the exterior ring is used**: interior holes are not treated as exclusion zones. Use simple field boundaries without holes for this workflow.

Geographic versus projected coordinates are currently inferred from numeric bounds: values within longitude/latitude ranges are treated as geographic. A CRS declaration is not used for reprojection. Projected data with small local coordinates may therefore need conversion to geographic coordinates before import. Elevations do not automatically create a terrain model.

## CSV

Supply columns recognised as longitude/latitude or East/North, for example:

```csv
field,longitude,latitude
Field A,-45.7500,-7.9600
Field A,-45.7480,-7.9600
Field A,-45.7480,-7.9580
Field A,-45.7500,-7.9580
```

Recognised horizontal column names include `lon`, `lng`, `longitude`; `lat`, `latitude`; `east`, `easting`, `x`; and `north`, `northing`, `y`. An optional `field`, `fieldid`, `field_id`, `id` or `name` column groups boundary vertices. List the vertices in boundary order.

## Ground station

For the default reproducible run, the importer places the ground station just east of the bounding box and midway between its northern/southern extremes. This may differ from a position chosen interactively in previous project runs.

Use `layoutOptions.selectGroundStation = true` to choose it on the map, or `layoutOptions.groundStationXY = [east north]` for a fixed position. An explicit position overrides the automatically selected position.

## Saved and manual layouts

The bundled `rectangularLayout.mat` contains the original saved rectangular example. Use `layoutOptions = struct('mode','saved');` to select it.

To save a layout after a mission:

```matlab
saveMissionLayout(fullfile(projectRoot(),'outputs','my-layout.mat'), ...
    missionCfg.fields, missionCfg.fieldVisitOrder, ...
    missionCfg.groundStationEast_m, missionCfg.groundStationNorth_m, missionCfg.geodata);
```

Load it with:

```matlab
layoutOptions = struct('mode','saved', ...
    'savedLayoutPath',fullfile(projectRoot(),'outputs','my-layout.mat'));
runMission
```

For programmatically constructed field structs, use `layoutOptions.mode = 'manual'` and `layoutOptions.fields = fields`. `defaultMissionConfig` validates the configuration, and `runMissionPlanning` validates/preprocesses the field geometry.

## Coordinate conventions

| Stage | Position convention | Units |
| --- | --- | --- |
| Field import and planning | East, North, Up (ENU) | m |
| Plotting | East, North, Up (ENU) | m |
| Exported route/spray waypoints | North, East, Down (NED) | m |
| Simulink position | North, East, Down (NED) | m |

The waypoint export performs the axis swap and sign change. Do not feed an ENU array directly to a model expecting NED. Configuration fields ending in `_deg` use degrees; aircraft/model angle variables generally use radians.

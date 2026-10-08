# Publication cleanup and validation

Prepared on 8 October 2026 using MATLAB R2025b on Windows. The original working folder and its historical results were retained. This repository is a separate publication copy.

## Changes from the working project

- Renamed the main entry script to `runMission` and extracted common defaults into `defaultMissionConfig`.
- Added `setupProject`, repository-relative paths and a single ignored `outputs/` location for generated data and caches.
- Bundled the SA farm GeoJSON as the default input, with deterministic ground-station placement and optional interactive placement.
- Included the highest numbered aircraft model, `SIXDOF28`, and its aircraft parameter script.
- Replaced fixed sortie indices in the model with `selectedSortie`. Preparation now aligns route waypoints, spray waypoints, initial position, initial heading and stop time. Flight duration comes from the cruise-only command export, excluding vertical flight and ground turnaround.
- Updated batch tracking to configure each sortie's initial position and duration as well as its heading and waypoints.
- Shared baseline configuration across the main workflow and baseline-based economic analyses; kept the separate studies' explicit cost assumptions.
- Restored required turn/run-out configuration fields in the standalone economic studies by inheriting the current mission defaults.
- Renamed the 120 ha sensitivity script to match its actual experiment, and corrected the alternative-scenario description to match the implemented one-minute turnaround settings.
- Fixed manual-layout initialisation, allowed nested planning-option overrides, and removed mandatory dialogs from the default run.
- Made an infeasible T50 comparison return an explicit status and reason, and guarded against repeated sorties that make no spray progress. Its energy/flight assumptions were not relaxed to force feasibility.
- Excluded old versions, standalone development experiments, autosaves, caches, GIS editing projects and bulk results. Kept the small saved-layout MAT file because it is an example input.

The core planning, mission simulation, straight/Dubins path construction and cross-track calculation implementations were compared with the original source and are unchanged (apart from text line endings). Flight-model numerical settings, aerodynamic coefficients and controller gains were retained. Model initialisation expressions and stop time were changed as described above.

## Reproducing the checks

The publication checks used a fresh MATLAB process with the default path restored before adding this repository. The following checks passed during preparation:

| Check | Result |
| --- | --- |
| Small saved-layout mission and configuration checks | Passed |
| Full SA farm mission, playback and dashboard creation | Passed |
| All-sortie waypoint export | 181 sortie entries produced |
| SIXDOF28 model compilation | Passed |
| Final single flight integration | 197,958 finite position samples; stop time 1,241.55748845 s |
| Planned/actual and spray-lane diagnostic figures | Passed |
| Batch tracking on sortie 2 | 1 successful sortie; 9 complete spray lanes |
| Economic field scenarios | All 12 produced finite results |
| 120 ha price/worker-rate sensitivity | All 15 cases produced finite results |
| Economic report figure pack | Both panels generated |
| T50 comparator on the saved example | Feasible comparison produced |
| T50 comparator with an unreachable ground station | Explicit infeasible status returned |

The batch check exercised a single sortie; it does not establish performance for the entire farm. That check preceded the final adjustment to the cruise-only stop time. The final single-sortie check verifies the adjusted stop time, and the batch uses the same preparation helper.

For the small regression check:

```matlab
setupProject
runSmokeTests
```

This checks the five-field default input, a complete saved-layout mission, finite path data, monotonic timestamps, path/segment consistency, nested overrides and rejection of misspelled settings.

For the full default workflow:

```matlab
clearvars
setupProject
runMission
selectedSortie = 1;
plotSortieWaypoints
runFlightSimulation
compareSimulinkTrajectoryToPlan
plotSprayLaneCrossTrackError
compareDroneToDJIAgrasT50
exportReadmeFigures
```

For the independent economic workflows:

```matlab
runEconomicScenarioComparison
runEconomicSensitivity120haRectangle
createEconomicFigurePack
```

## Default mission reference

These values were obtained from the supplied SA farm file and automatic ground-station placement. Moving the ground station or altering settings changes the results.

| Quantity | Value |
| --- | ---: |
| Fields | 5 |
| Geometric field area | 566.3466 ha |
| Field visit order | 1, 3, 4, 5, 2 |
| Spray lanes | 863 |
| Sorties | 181 |
| Mission duration including turnaround | 156,162.18 s (43.3784 h) |
| Modelled sprayed-area equivalent | 566.7633 ha |
| Overall capacity | 13.066 ha/h |
| Revenue | GBP 11,335.27 |
| Modelled cost | GBP 3,578.57 |
| Modelled profit | GBP 7,756.70 |

The sprayed-area equivalent comes from dispensed litres divided by application rate. It can exceed geometric field area slightly because of path/lane discretisation; it is not an exact covered-polygon area. Economic values follow that modelled sprayed-area measure.

## Verification limits

The large SA farm parameter sweep, multi-iteration profit search and all-sortie flight batch are long-running studies and have not been exhaustively rerun as part of the cleanup. The interactive field designer and file-picker workflows also require manual UI use. Older MATLAB releases, other operating systems and real-aircraft behaviour have not been validated. Passing the execution checks is not a claim that every model assumption or tracking target is satisfied.

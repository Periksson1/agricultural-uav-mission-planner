%RUNFLIGHTSIMULATION Prepare and simulate one sortie; results are stored in out.
prepareFlightSimulation;
out = sim(simModelName, 'ReturnWorkspaceOutputs','on');
simulatedSortie = selectedSortie;
fprintf('Flight complete. Run compareSimulinkTrajectoryToPlan and plotSprayLaneCrossTrackError.\n');

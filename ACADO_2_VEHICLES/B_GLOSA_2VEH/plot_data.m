% ============================================================================
% script per plottare i risultati della simulazione con fasi semaforiche e stop
%==============================================================================

s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0];
s_stop = [80, 447, 756, 1084, 1304, 1507, 1822]; 
i_fig = 10;
%h = plot_covDist_platoon2(out,i_fig,s_TL,s_stop,L_platoon);
h = plot_covDist_2veh_platoon(out,1,s_TL,s_stop,L_platoon);
% ============================================================================
% script per plottare i risultati della simulazione con fasi semaforiche e stop
% B-GLOSA 1 veicolo
%==============================================================================
s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, ...
        1823.0, 1948.9];
% Deve coincidere con ACADO_1veh_4TL.m e con la chart OnlineDataArray.
L_platoon = 0.1;   % [m]
i_fig = 10;
h = plot_covDist_1veh_platoon(out, i_fig, s_TL, s_stop, L_platoon);
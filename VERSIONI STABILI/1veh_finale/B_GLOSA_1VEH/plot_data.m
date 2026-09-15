% ============================================================================
% script per plottare i risultati della simulazione con fasi semaforiche e stop
% B-GLOSA 1 veicolo
%==============================================================================
s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, ...
        1823.0, 1948.9, 2046.6, 2287.9, 2421.5, 2635.8, 2683.8, 2773.9, 2800.2, 2828.4, ...
        2981.4, 3232.6, 3420.6, 3600.4, 3764.9, 4051.8, 4215.6, 4434.6, 4648.9, 5107.8];
s_stop = [80, 447, 756, 1084, 1304, 1507, 1822];
% Deve coincidere con ACADO_1veh_4TL.m e con la chart OnlineDataArray.
L_platoon = 7.8;   % [m]
i_fig = 10;
h = plot_covDist_1veh_platoon(out, i_fig, s_TL, s_stop, L_platoon);
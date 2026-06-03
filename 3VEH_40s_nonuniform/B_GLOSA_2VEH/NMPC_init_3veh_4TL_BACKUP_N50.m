%==========================================================================
% NMPC INIT - B-GLOSA 3 veicoli indipendenti - 4 TL attivi
%==========================================================================
% Script da eseguire dopo la generazione ACADO_3veh_4TL.m e prima della
% simulazione Simulink.
%=========================================================================

%% ========================================================================
%  1. Soft Constraints / Cost Function Weights
%  ========================================================================
% Ordine dei termini h in ACADO_3veh_4TL.m:
%  1  cost_dist           avanzamento veicolo 1
%  2  cost_dist2          avanzamento veicolo 2
%  3  cost_dist3          avanzamento veicolo 3
%  4  cost_jerk           jerk veicolo 1
%  5  cost_jerk2          jerk veicolo 2
%  6  cost_jerk3          jerk veicolo 3
%  7  cost_Ax             accelerazione veicolo 1
%  8  cost_Ax2            accelerazione veicolo 2
%  9  cost_Ax3            accelerazione veicolo 3
% 10  cost_stop_target    attrazione fermata veicolo 1
% 11  cost_stop_target2   attrazione fermata veicolo 2
% 12  cost_stop_target3   attrazione fermata veicolo 3
% 13  cost_dwell_pos      posizione durante dwell veicolo 1
% 14  cost_dwell_pos2     posizione durante dwell veicolo 2
% 15  cost_dwell_pos3     posizione durante dwell veicolo 3
% 16  cost_dwell_vel      velocita' durante dwell veicolo 1
% 17  cost_dwell_vel2     velocita' durante dwell veicolo 2
% 18  cost_dwell_vel3     velocita' durante dwell veicolo 3
% 19  cost_dwell_acc      accelerazione durante dwell veicolo 1
% 20  cost_dwell_acc2     accelerazione durante dwell veicolo 2
% 21  cost_dwell_acc3     accelerazione durante dwell veicolo 3
% 22  cost_aero12         spacing/aerodinamicita' veicolo 1-2
% 23  cost_aero23         spacing/aerodinamicita' veicolo 2-3

Qx1 = [ ...
    30, 30, 30, ...       % distance progress
    2, 2, 2, ...          % jerk
    0.5, 0.5, 0.5, ...    % longitudinal acceleration
    50, 50, 50, ...       % stop target attraction
    200, 200, 200, ...    % dwell position
    500, 500, 500, ...    % dwell velocity
    100, 100, 100, ...    % dwell acceleration
    0.08, 0.08];      % spacing/aero 1-2 and 2-3
    %1e6, 1e6];        %slack gap12 slack gap23

W1 = diag(Qx1);
NMPC_Wmat1 = reshape(W1.',1,[]);

% Terminal cost: avanzamento terminale dei tre veicoli.
QN1 = [30 30 30];
WN1 = diag(QN1);
NMPC_WNmat1 = reshape(WN1.',1,[]);
N_Jterms = length(Qx1);

%% ========================================================================
%  2. Parametri generali MPC
%  ========================================================================

t_dwell = 10;
t_offset = 0;

zInit = [];
bValues = [];

N_iter = 1;
Ts = 1;
N = 50;

Vmax   = 50;
Vmax_2 = 50;
Vmax_3 = 50;

% Stato iniziale ACADO: [pos vel acc pos2 vel2 acc2 pos3 vel3 acc3]
xInit = [65 2 0 50 2 0 0 2 0];
uInit = [0 0 0];

%% ========================================================================
%  3. Traffic light data
%  ========================================================================

%load('TL_data_piola-tonale_lookupTable_1500s_12TL.mat')
load('TL_data_piola-lugano_lookupTable_1600s_30TL.mat')

tl_state_sim = zeros(round(time_bag(end))-1,size(tl_col,2));
time_bag_sim = (0:1:round(time_bag(end))-1)';

for ii = 1:length(time_bag_sim)
    for jj = 1:size(tl_col,2)
        if ii == 1
            tl_state_sim(ii,jj) = tl_col(1,jj);
        else
            tl_state_sim(ii,jj) = tl_col(10*ii-10,jj);
        end
    end
end

%% ========================================================================
%  4. Road curvature lookup table
%  ========================================================================

load('k_road_PL_jrl_chs_precomputed.mat','s_kroad_map','k_road_map','ds','s_start')
s_kroad_map = s_kroad_map(:);
k_road_map  = k_road_map(:);

addpath(fullfile('..','chs_gen'))



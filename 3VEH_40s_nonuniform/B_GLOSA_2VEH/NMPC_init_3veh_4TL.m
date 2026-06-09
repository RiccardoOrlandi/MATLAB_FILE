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
    100, 100, 100, ...       % distance progress
    10, 10, 10, ...          % jerk
    15, 15, 15, ...    % longitudinal acceleration
    10, 10, 10, ...       % stop target attraction
    10, 10, 10, ...    % dwell position
    5, 5, 5, ...    % dwell velocity
    2, 2, 2, ...    % dwell acceleration
    70, 70];      % spacing/aero 1-2 and 2-3
    %1e6, 1e6];        %slack gap12 slack gap23

W1 = diag(Qx1);
NMPC_Wmat1 = reshape(W1.',1,[]);

% Terminal cost: avanzamento terminale dei tre veicoli.
QN1 = [100 100 100];
WN1 = diag(QN1);
NMPC_WNmat1 = reshape(WN1.',1,[]);
N_Jterms = length(Qx1);

%% ========================================================================
%  2. Parametri generali MPC
%  ========================================================================

t_dwell = 7;
t_offset = 0;

zInit = [];
bValues = [];

N_iter = 1;

%==========================================================================
% Griglia temporale NMPC non uniforme
%==========================================================================
% 40 intervalli, 41 nodi:
%   nodi 1..21  : 0:1:20 s
%   nodi 22..41 : 21.5:1.5:50 s
%
% Ts resta pari a 1 s perché rappresenta il sample time del controller
% Simulink/NMPC, non il passo interno della griglia OCP.
%==========================================================================

Ts = 1.0;        % [s] sample time controller
N  = 44;         % [-] numero intervalli OCP

N_near  = 14;
N_far   = 30;
Ts_near = 1.0;
Ts_far  = 1.2;

t_near = 0:Ts_near:(N_near*Ts_near);
t_far  = t_near(end) + Ts_far*(1:N_far);

timepoints = [t_near, t_far];   % 1 x 41
time_grid  = timepoints(:);     % 41 x 1

if numel(timepoints) ~= N+1
    error('Errore griglia NMPC: numel(timepoints) deve essere N+1.');
end

if abs(timepoints(end) - 50.0) > 1e-12
    error('Errore griglia NMPC: orizzonte finale diverso da 50 s.');
end
Vmax   = 50;
Vmax_2 = 50;
Vmax_3 = 50;

% Stato iniziale ACADO: [pos vel acc pos2 vel2 acc2 pos3 vel3 acc3]
xInit = [0 0 0 -9 0 0 -19 0 0];
uInit = [0 0 0];
% s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, ...
%         1823.0, 1948.9, 2046.6, 2287.9, 2421.5, 2635.8, 2683.8, 2773.9, 2800.2, 2828.4, ...
%         2981.4, 3232.6, 3420.6, 3600.4, 3764.9, 4051.8, 4215.6, 4434.6, 4648.9, 5107.8];
% s_stop = [80, 447, 756, 1084, 1304, 1507, 1822];
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



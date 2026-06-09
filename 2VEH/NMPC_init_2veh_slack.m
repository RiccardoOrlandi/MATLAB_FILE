%==========================================================================
% NMPC INIT - B-GLOSA 2 veicoli indipendenti - 4 TL attivi
%==========================================================================
% Script da eseguire dopo ACADO_2veh_slack.m e prima della simulazione Simulink.
%==========================================================================

%% ========================================================================
%%  1. Soft Constraints / Cost Function Weights
%%  ========================================================================
% Ordine dei termini h in ACADO_2veh_slack.m:
%  1  cost_dist           avanzamento veicolo 1
%  2  cost_dist2          avanzamento veicolo 2
%  3  cost_jerk           jerk veicolo 1
%  4  cost_jerk2          jerk veicolo 2
%  5  cost_Ax             accelerazione veicolo 1
%  6  cost_Ax2            accelerazione veicolo 2
%  7  cost_stop_target    attrazione fermata veicolo 1
%  8  cost_stop_target2   attrazione fermata veicolo 2
%  9  cost_dwell_pos      posizione durante dwell veicolo 1
% 10  cost_dwell_pos2     posizione durante dwell veicolo 2
% 11  cost_dwell_vel      velocita' durante dwell veicolo 1
% 12  cost_dwell_vel2     velocita' durante dwell veicolo 2
% 13  cost_dwell_acc      accelerazione durante dwell veicolo 1
% 14  cost_dwell_acc2     accelerazione durante dwell veicolo 2
% 15  cost_aero12         spacing/aerodinamicita' veicolo 1-2
% 16  cost_Jmin1          slack jerk min veicolo 1
% 17  cost_Jmin2          slack jerk min veicolo 2

Qx1 = [ ...
    100, 100, ...           % distance progress
    30, 30, ...             % jerk
    15, 15, ...         % longitudinal acceleration
    10, 10, ...           % stop target attraction
    10, 10, ...         % dwell position
    5, 5, ...         % dwell velocity
    2, 2, ...         % dwell acceleration
    10]; ...              % spacing/aero 1-2
    %1e4, 1e4];      % slack J_min

W1 = diag(Qx1);
NMPC_Wmat1 = reshape(W1.',1,[]);

% Terminal cost: avanzamento terminale dei due veicoli.
QN1 = [100 100];
WN1 = diag(QN1);
NMPC_WNmat1 = reshape(WN1.',1,[]);
N_Jterms = length(Qx1);

%% ========================================================================
%%  2. Parametri generali MPC
%%  ========================================================================

t_dwell  = 7;
t_offset = 0;

zInit   = [];
bValues = [];

N_iter = 1;

%==========================================================================
% Griglia temporale NMPC non uniforme
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

% Stato iniziale ACADO: [pos vel acc pos2 vel2 acc2]
xInit = [0 0 0 -20 0 0];
uInit = [0 0 0 0];   % [jerk jerk2 s_Jmin1 s_Jmin2]

%% ========================================================================
%%  3. Traffic light data
%%  ========================================================================

load('TL_data_piola-lugano_lookupTable_1600s_30TL.mat')

tl_state_sim = zeros(round(time_bag(end))-1, size(tl_col,2));
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
%%  4. Road curvature lookup table
%%  ========================================================================

load('k_road_PL_jrl_chs_precomputed.mat','s_kroad_map','k_road_map','ds','s_start')
s_kroad_map = s_kroad_map(:);
k_road_map  = k_road_map(:);

addpath(fullfile('.','chs_gen'))

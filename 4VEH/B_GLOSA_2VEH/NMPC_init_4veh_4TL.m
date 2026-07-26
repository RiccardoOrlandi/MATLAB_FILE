%==========================================================================
% NMPC INIT - B-GLOSA 4 veicoli indipendenti - 4 TL attivi
%==========================================================================
% Script da eseguire dopo la generazione ACADO_4veh_4TL.m e prima della
% simulazione Simulink.
%=========================================================================

%% ========================================================================
%  1. Soft Constraints / Cost Function Weights
%  ========================================================================
% Ordine dei termini h in ACADO_4veh_4TL.m:
%  1  cost_dist           avanzamento veicolo 1
%  2  cost_dist2          avanzamento veicolo 2
%  3  cost_dist3          avanzamento veicolo 3
%  4  cost_dist4          avanzamento veicolo 4
%  5  cost_jerk           jerk veicolo 1
%  6  cost_jerk2          jerk veicolo 2
%  7  cost_jerk3          jerk veicolo 3
%  8  cost_jerk4          jerk veicolo 4
%  9  cost_Ax             accelerazione veicolo 1
% 10  cost_Ax2            accelerazione veicolo 2
% 11  cost_Ax3            accelerazione veicolo 3
% 12  cost_Ax4            accelerazione veicolo 4
% 13  cost_stop_target    attrazione fermata veicolo 1
% 14  cost_stop_target2   attrazione fermata veicolo 2
% 15  cost_stop_target3   attrazione fermata veicolo 3
% 16  cost_stop_target4   attrazione fermata veicolo 4
% 17  cost_dwell_pos      posizione durante dwell veicolo 1
% 18  cost_dwell_pos2     posizione durante dwell veicolo 2
% 19  cost_dwell_pos3     posizione durante dwell veicolo 3
% 20  cost_dwell_pos4     posizione durante dwell veicolo 4
% 21  cost_dwell_vel      velocita' durante dwell veicolo 1
% 22  cost_dwell_vel2     velocita' durante dwell veicolo 2
% 23  cost_dwell_vel3     velocita' durante dwell veicolo 3
% 24  cost_dwell_vel4     velocita' durante dwell veicolo 4
% 25  cost_dwell_acc      accelerazione durante dwell veicolo 1
% 26  cost_dwell_acc2     accelerazione durante dwell veicolo 2
% 27  cost_dwell_acc3     accelerazione durante dwell veicolo 3
% 28  cost_dwell_acc4     accelerazione durante dwell veicolo 4
% 29  cost_aero12         spacing/aerodinamicita' veicolo 1-2
% 30  cost_aero23         spacing/aerodinamicita' veicolo 2-3
% 31  cost_aero34         spacing/aerodinamicita' veicolo 3-4
%
% NOTA IMPORTANTE (da discutere prima di lanciare simulazioni definitive):
% i pesi qui sotto sono ancora una replica 1:1 dei pesi della versione a
% 3 veicoli (stesso valore per ogni veicolo, stesso valore di spacing per
% ogni coppia adiacente). Questo NON implementa ancora la normalizzazione
% "stesso comportamento del caso single-vehicle" discussa come punto 3
% della roadmap: e' solo un punto di partenza funzionante per verificare
% che l'estensione a 4 veicoli compili e giri correttamente.

Qx1 = [ ...
    100, 100, 100, 100, ...   % distance progress
    10, 10, 10, 10, ...       % jerk
    15, 15, 15, 15, ...       % longitudinal acceleration
    10, 10, 10, 10, ...       % stop target attraction
    10, 10, 10, 10, ...       % dwell position
    5, 5, 5, 5, ...           % dwell velocity
    2, 2, 2, 2, ...           % dwell acceleration
    70, 70, 70];              % spacing/aero 1-2, 2-3, 3-4

W1 = diag(Qx1);
NMPC_Wmat1 = reshape(W1.',1,[]);

% Terminal cost: avanzamento terminale dei quattro veicoli.
QN1 = [100 100 100 100];
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
% Griglia temporale NMPC non uniforme (identica alla versione 3 veicoli)
%==========================================================================
% 44 intervalli, 45 nodi:
%   nodi 1..15  : 0:1:14 s
%   nodi 16..45 : 15.2:1.2:50 s
%
% Ts resta pari a 1 s perche' rappresenta il sample time del controller
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

timepoints = [t_near, t_far];   % 1 x 45
time_grid  = timepoints(:);     % 45 x 1

if numel(timepoints) ~= N+1
    error('Errore griglia NMPC: numel(timepoints) deve essere N+1.');
end

if abs(timepoints(end) - 50.0) > 1e-12
    error('Errore griglia NMPC: orizzonte finale diverso da 50 s.');
end

Vmax   = 50;
Vmax_2 = 50;
Vmax_3 = 50;
Vmax_4 = 50;

% Stato iniziale ACADO: [pos vel acc pos2 vel2 acc2 pos3 vel3 acc3 pos4 vel4 acc4]
% Spaziatura iniziale di 10 m tra veicoli consecutivi (in linea con d_gap +
% L_platoon usati nella versione a 3 veicoli).
xInit = [0 0 0 -9 0 0 -19 0 0 -29 0 0];
uInit = [0 0 0 0];

%% ========================================================================
%  3. Traffic light data
%  ========================================================================

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

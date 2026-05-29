%==========================================================================
% NMPC INIT - B-GLOSA 1 veicolo rigido lungo L_platoon - 4 TL attivi
%==========================================================================
% Script da eseguire dopo la generazione ACADO:
%   ACADO_BGLOSA_1veh_K4_Lplatoon_aligned.m
% e prima della simulazione Simulink.
%
% La configurazione e' allineata alla versione 3-vehicle alleggerita:
% - OnlineData: 21 parametri/nodo;
% - 4 semafori attivi mobili;
% - una sola fermata attiva;
% - k_road precomputata e passata come lookup/horizon;
% - nessuna slack su Vmax e Ay.
%==========================================================================

%% ========================================================================
%  1. Soft Constraints / Cost Function Weights
%  ========================================================================
% Ordine dei termini h in ACADO_BGLOSA_1veh_K4_Lplatoon_aligned.m:
%  1  cost_dist          avanzamento lungo il percorso
%  2  cost_jerk          jerk longitudinale
%  3  cost_Ax            accelerazione longitudinale
%  4  cost_stop_target   attrazione verso fermata attiva
%  5  cost_dwell_pos     mantenimento posizione durante dwell
%  6  cost_dwell_vel     velocita' nulla durante dwell
%  7  cost_dwell_acc     accelerazione nulla durante dwell

Qx1 = [ ...
    30, ...    % distance progress, coerente con veicolo 1 della versione 3-veh
     2, ...    % jerk
     0.5, ...  % longitudinal acceleration
    50, ...    % stop target attraction
   200, ...    % dwell position
   500, ...    % dwell velocity
   100];       % dwell acceleration

W1 = diag(Qx1);
NMPC_Wmat1 = reshape(W1.',1,[]);

% Terminal cost: avanzamento terminale del veicolo rigido.
QN1 = 30;
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

% ATTENZIONE UNITA': mantenuto pari alla versione 3-vehicle.
% Se il segnale velocita' del modello e' in m/s, usare 50/3.6.
% Se il tuo modello Simulink gestisce gia' la conversione, lascia 50.
Vmax = 50;

% Stato iniziale ACADO: [pos vel acc]
xInit = [0 2 0];

% Nuova formulazione: un solo controllo, il jerk.
% Le vecchie componenti s_Vmax e s_Ay sono state eliminate.
uInit = 0;

%% ========================================================================
%  3. Traffic light data
%  ========================================================================

load('TL_data_piola-tonale_lookupTable_1500s_12TL.mat')

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
%  4. Road curvature lookup table precomputata
%  ========================================================================
% La lookup viene caricata una volta in init. OnlineDataArray deve poi
% ricevere k_road_hor gia' estratta/interpolata sull'orizzonte, evitando
% calcoli pesanti dentro la MATLAB Function di Simulink.

load('k_road_PL_jrl_chs_precomputed.mat','s_kroad_map','k_road_map','ds','s_start')
s_kroad_map = s_kroad_map(:);
k_road_map  = k_road_map(:);

addpath(fullfile('..','chs_gen'))

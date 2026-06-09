function [OnlineData, reached_TL, reached_TL_2, reached_stop, reached_stop_2, t_piola, t_loreto, debug_TL, debug_TL_2, stop_dwell_count, stop_dwell_count_2, red_stopline_hold, s_hold, red_stopline_hold2, s_hold2] = OnlineDataArray( ...
    x_TL, stop_enabled, Vmax, Vmax_2, state_pred_old, t_dwell, time_cur, ...
    reached_TL_old, reached_TL_old_2, reached_stop_old, reached_stop_old_2, ...
    t_piola_old, t_loreto_old, stop_dwell_count_old, stop_dwell_count_old_2, ...
    k_road_hor, k_road_hor_2, timepoints)
%==========================================================================
% ONLINE DATA ARRAY - NMPC B-GLOSA 2 VEHICLES
% Derivato dalla versione 3 veicoli: rimosso tutto il veicolo 3.
%
% OnlineData per nodo: 42 = 21 dati/veicolo
%   VEICOLO 1: indici  1 - 21
%   VEICOLO 2: indici 22 - 42
%
% state_pred_old deve essere (N+1)x6 = 41x6, colonne:
%   [pos vel acc pos2 vel2 acc2]
%==========================================================================

N_od    = 42;
K_tl    = 4;
N_tl    = 30;
N_stops = 7;
N       = 40;
Ts      = 1.0;

if nargin < 18 || isempty(timepoints)
    timepoints = (0:N) * Ts;
end
t_hor = timepoints(:);

%---------------------------- vehicle 1 -----------------------------------
idx_head0         = 0;    % 1  - 4
idx_sstop_active  = 5;
idx_xstop_active  = 6;
idx_xdwell_active = 7;
idx_wstop_active  = 8;
idx_kroad         = 9;
idx_vmax          = 10;
idx_dtschd        = 11;
idx_sst           = 12;
idx_shor          = 13;
idx_tail0         = 13;   % 14 - 17
idx_sTL0          = 17;   % 18 - 21

%---------------------------- vehicle 2 -----------------------------------
idx_head0_2         = 21; % 22 - 25
idx_sstop_active_2  = 26;
idx_xstop_active_2  = 27;
idx_xdwell_active_2 = 28;
idx_wstop_active_2  = 29;
idx_kroad_2         = 30;
idx_vmax_2          = 31;
idx_dtschd_2        = 32;
idx_sst_2           = 33;
idx_shor_2          = 34;
idx_tail0_2         = 34; % 35 - 38
idx_sTL0_2          = 38; % 39 - 42

L_platoon = 8;  % [m]

s_TL   = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, ...
           1823.0, 1948.9, 2046.6, 2287.9, 2421.5, 2635.8, 2683.8, 2773.9, 2800.2, 2828.4, ...
           2981.4, 3232.6, 3420.6, 3600.4, 3764.9, 4051.8, 4215.6, 4434.6, 4648.9, 5107.8];
s_stop = [80, 447, 756, 1084, 1304, 1507, 1822];
s_max  = 5200;

dt_schd_default = 180;
s_st_default    = 1e5;
s_hor_default   = 300;

Ts_fine_local = timepoints(2) - timepoints(1);
N_dwell_steps = max(1, round(t_dwell / Ts_fine_local));

eps_stop_dwell = 0.8;   % [m]
eps_vel_dwell  = 0.05;  % [m/s]
eps_TL_hold    = 0.20;  % [m]

%=========================================================================
% OUTPUT INITIALIZATION
%=========================================================================

OnlineData = zeros(N_od*(N+1),1);

reached_TL   = reached_TL_old;
reached_TL_2 = reached_TL_old_2;

reached_stop   = reached_stop_old;
reached_stop_2 = reached_stop_old_2;

t_piola  = t_piola_old;
t_loreto = t_loreto_old;

stop_dwell_count   = stop_dwell_count_old;
stop_dwell_count_2 = stop_dwell_count_old_2;

red_stopline_hold  = 0.0;
red_stopline_hold2 = 0.0;
s_hold             = 0.0;
s_hold2            = 0.0;

debug_TL   = zeros(N_tl,26);
debug_TL_2 = zeros(N_tl,26);

%=========================================================================
% PREDICTED STATES
%=========================================================================

pos_hor      = state_pred_old(:,1);
vel_hor      = state_pred_old(:,2);
pos_hor_tail = pos_hor - L_platoon;
s_now        = pos_hor(1);
v_now        = vel_hor(1);

pos_hor_2      = state_pred_old(:,4);
vel_hor_2      = state_pred_old(:,5);
pos_hor_tail_2 = pos_hor_2 - L_platoon;
s_now_2        = pos_hor_2(1);
v_now_2        = vel_hor_2(1);

%=========================================================================
% BASE ONLINE DATA NODE
%=========================================================================

base_node = zeros(N_od,1);

%---------------------------- vehicle 1 -----------------------------------
base_node(idx_head0 + (1:K_tl)) = 1;
base_node(idx_sstop_active)  = s_max;
base_node(idx_xstop_active)  = 1;
base_node(idx_xdwell_active) = 0;
base_node(idx_wstop_active)  = 0;
base_node(idx_kroad)         = 0;
base_node(idx_vmax)          = Vmax;
base_node(idx_dtschd)        = dt_schd_default;
base_node(idx_sst)           = s_st_default;
base_node(idx_shor)          = s_hor_default;
base_node(idx_tail0 + (1:K_tl)) = 1;
base_node(idx_sTL0  + (1:K_tl)) = s_max;

%---------------------------- vehicle 2 -----------------------------------
base_node(idx_head0_2 + (1:K_tl)) = 1;
base_node(idx_sstop_active_2)  = s_max;
base_node(idx_xstop_active_2)  = 1;
base_node(idx_xdwell_active_2) = 0;
base_node(idx_wstop_active_2)  = 0;
base_node(idx_kroad_2)         = 0;
base_node(idx_vmax_2)          = Vmax_2;
base_node(idx_dtschd_2)        = dt_schd_default;
base_node(idx_sst_2)           = s_st_default;
base_node(idx_shor_2)          = s_hor_default;
base_node(idx_tail0_2 + (1:K_tl)) = 1;
base_node(idx_sTL0_2  + (1:K_tl)) = s_max;

for kk = 1:N+1
    idx0 = N_od*(kk-1);
    OnlineData(idx0+1:idx0+N_od) = base_node;
end

%=========================================================================
% ROAD CURVATURE ONLINE DATA
%=========================================================================

if numel(k_road_hor) == 1
    for kk = 1:N+1
        OnlineData(idx_kroad + N_od*(kk-1)) = abs(k_road_hor);
    end
else
    for kk = 1:N+1
        OnlineData(idx_kroad + N_od*(kk-1)) = abs(k_road_hor(kk));
    end
end

if numel(k_road_hor_2) == 1
    for kk = 1:N+1
        OnlineData(idx_kroad_2 + N_od*(kk-1)) = abs(k_road_hor_2);
    end
else
    for kk = 1:N+1
        OnlineData(idx_kroad_2 + N_od*(kk-1)) = abs(k_road_hor_2(kk));
    end
end

%=========================================================================
% BUS STOP ONLINE DATA
%=========================================================================

[OnlineData, reached_stop, stop_dwell_count] = local_update_stop_online_data( ...
    OnlineData, reached_stop, reached_stop_old, stop_dwell_count, stop_enabled, ...
    s_stop, s_now, v_now, N_stops, N_dwell_steps, eps_stop_dwell, eps_vel_dwell, ...
    N, N_od, idx_sstop_active, idx_xstop_active, idx_xdwell_active, idx_wstop_active);

[OnlineData, reached_stop_2, stop_dwell_count_2] = local_update_stop_online_data( ...
    OnlineData, reached_stop_2, reached_stop_old_2, stop_dwell_count_2, stop_enabled, ...
    s_stop, s_now_2, v_now_2, N_stops, N_dwell_steps, eps_stop_dwell, eps_vel_dwell, ...
    N, N_od, idx_sstop_active_2, idx_xstop_active_2, idx_xdwell_active_2, idx_wstop_active_2);

%=========================================================================
% TRAFFIC LIGHT SLOT SELECTION - INDIPENDENTE PER OGNI VEICOLO
%=========================================================================

[active_TL_idx_1, s_TL_active_1] = local_select_active_tl_slots(pos_hor_tail(1),   s_TL, N_tl, K_tl, s_max);
[active_TL_idx_2, s_TL_active_2] = local_select_active_tl_slots(pos_hor_tail_2(1), s_TL, N_tl, K_tl, s_max);

for kk = 1:N+1
    idx0 = N_od*(kk-1);
    for jj = 1:K_tl
        OnlineData(idx0 + idx_sTL0   + jj) = s_TL_active_1(jj);
        OnlineData(idx0 + idx_sTL0_2 + jj) = s_TL_active_2(jj);
    end
end

%=========================================================================
% MEMORY SAFETY UPDATE
%=========================================================================

reached_TL   = local_update_reached_memory_only(reached_TL_old,   s_TL, pos_hor(1),   pos_hor_tail(1),   vel_hor(1),   x_TL, N_tl);
reached_TL_2 = local_update_reached_memory_only(reached_TL_old_2, s_TL, pos_hor_2(1), pos_hor_tail_2(1), vel_hor_2(1), x_TL, N_tl);

%=========================================================================
% TRAFFIC LIGHT ONLINE DATA
%=========================================================================

[OnlineData, reached_TL, debug_TL, red_stopline_hold, s_hold] = local_update_tl_slots_for_vehicle( ...
    OnlineData, x_TL, s_TL, active_TL_idx_1, s_TL_active_1, ...
    pos_hor, pos_hor_tail, vel_hor, Vmax, reached_TL_old, reached_TL, debug_TL, ...
    time_cur, N, N_od, K_tl, idx_tail0, idx_head0, eps_TL_hold);

[OnlineData, reached_TL_2, debug_TL_2, red_stopline_hold2, s_hold2] = local_update_tl_slots_for_vehicle( ...
    OnlineData, x_TL, s_TL, active_TL_idx_2, s_TL_active_2, ...
    pos_hor_2, pos_hor_tail_2, vel_hor_2, Vmax_2, reached_TL_old_2, reached_TL_2, debug_TL_2, ...
    time_cur, N, N_od, K_tl, idx_tail0_2, idx_head0_2, eps_TL_hold);

end

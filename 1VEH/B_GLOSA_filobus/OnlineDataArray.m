function [OnlineData, reached_TL, reached_stop, t_piola, t_loreto, debug_TL, stop_dwell_count, red_stopline_hold, s_hold] = OnlineDataArray( ...
    x_TL, stop_enabled, Vmax, state_pred_old, t_dwell, time_cur, ...
    reached_TL_old, reached_stop_old, t_piola_old, t_loreto_old, ...
    stop_dwell_count_old, k_road_hor)
%==========================================================================
% ONLINE DATA ARRAY - NMPC B-GLOSA 1 VEICOLO RIGIDO LUNGO L_PLATOON
% Versione K4 allineata alla logica 3-vehicle con 4 TL attivi mobili.
%==========================================================================
%
% Convenzione semaforica:
%   x_TL = 1  -> verde / vincolo rilasciato
%   x_TL = 0  -> rosso / vincolo attivo
%
% Convenzione fermata:
%   x_stop_active  = 0 -> vincolo approccio attivo: pos <= s_stop_active
%   x_stop_active  = 1 -> vincolo approccio rilasciato
%   x_dwell_active = 1 -> dwell box attiva
%   x_dwell_active = 0 -> dwell non attivo
%   w_stop_active  = 1 -> costo attrazione fermata attivo
%   w_stop_active  = 0 -> costo attrazione fermata spento
%
% Struttura OnlineData per nodo, dimensione 21:
%   1  - 4    x_TL_head, slot 1...4
%   5         s_stop_active
%   6         x_stop_active
%   7         x_dwell_active
%   8         w_stop_active
%   9         k_road
%   10        Vmax
%   11        dt_schd       placeholder compatibilita'
%   12        s_st          placeholder compatibilita'
%   13        s_hor         placeholder compatibilita'
%   14 - 17   x_TL_tail, slot 1...4
%   18 - 21   s_TL_active, coordinate fisiche degli slot TL
%
% state_pred_old deve essere 51x3, con colonne:
%   [pos vel acc]
%==========================================================================

%=========================================================================
% CONSTANTS AND INDICES
%=========================================================================

N_od    = 21;
K_tl    = 4;
N_tl    = 30;
N_stops = 7;
N       = 50;
Ts      = 1.0;

idx_head0        = 0;    % 1  - 4
idx_sstop_active = 5;
idx_xstop_active = 6;
idx_xdwell_active = 7;
idx_wstop_active = 8;
idx_kroad        = 9;
idx_vmax         = 10;
idx_dtschd       = 11;
idx_sst          = 12;
idx_shor         = 13;
idx_tail0        = 13;   % 14 - 17
idx_sTL0         = 17;   % 18 - 21

% Lunghezza rigida equivalente. Deve coincidere con il valore usato nello
% script ACADO. Default: equivalente a 3 veicoli da 8 m con due gap da 3 m.
L_platoon = 18;  % [m]

s_TL   = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, ...
           1823.0, 1948.9, 2046.6, 2287.9, 2421.5, 2635.8, 2683.8, 2773.9, 2800.2, 2828.4, ...
           2981.4, 3232.6, 3420.6, 3600.4, 3764.9, 4051.8, 4215.6, 4434.6, 4648.9, 5107.8];
s_stop = [80, 447, 756, 1084, 1304, 1507, 1822];
s_max  = 5200;

dt_schd_default = 180;
s_st_default    = 1e5;
s_hor_default   = 300;

N_dwell_steps = max(1, round(t_dwell/Ts));

eps_stop_dwell = 0.8;   % [m]
eps_vel_dwell  = 0.05;  % [m/s]
eps_TL_hold    = 0.20;  % [m]

%=========================================================================
% OUTPUT INITIALIZATION
%=========================================================================

OnlineData = zeros(N_od*(N+1),1);

reached_TL = reached_TL_old;
reached_stop = reached_stop_old;

t_piola  = t_piola_old;
t_loreto = t_loreto_old;

stop_dwell_count = stop_dwell_count_old;

red_stopline_hold = 0.0;
s_hold            = 0.0;

debug_TL = zeros(N_tl,26);

%=========================================================================
% PREDICTED STATES
%=========================================================================

pos_hor      = state_pred_old(:,1);
vel_hor      = state_pred_old(:,2);
pos_hor_tail = pos_hor - L_platoon;

s_now = pos_hor(1);
v_now = vel_hor(1);

%=========================================================================
% BASE ONLINE DATA NODE
%=========================================================================

base_node = zeros(N_od,1);

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

for kk = 1:N+1
    idx0 = N_od*(kk-1);
    OnlineData(idx0+1:idx0+N_od) = base_node;
end

%=========================================================================
% ROAD CURVATURE ONLINE DATA
%=========================================================================
% k_road_hor deve essere gia' precomputata/interpolata fuori da questa
% funzione. Qui viene solo copiata negli OnlineData.

if numel(k_road_hor) == 1
    for kk = 1:N+1
        OnlineData(idx_kroad + N_od*(kk-1)) = abs(k_road_hor);
    end
else
    for kk = 1:N+1
        OnlineData(idx_kroad + N_od*(kk-1)) = abs(k_road_hor(kk));
    end
end

%=========================================================================
% BUS STOP ONLINE DATA - ONE ACTIVE STOP
%=========================================================================

[OnlineData, reached_stop, stop_dwell_count] = local_update_stop_online_data( ...
    OnlineData, reached_stop, reached_stop_old, stop_dwell_count, stop_enabled, ...
    s_stop, s_now, v_now, N_stops, N_dwell_steps, eps_stop_dwell, eps_vel_dwell, ...
    N, N_od, idx_sstop_active, idx_xstop_active, idx_xdwell_active, idx_wstop_active);

%=========================================================================
% TRAFFIC LIGHT SLOT SELECTION
%=========================================================================
% La selezione usa la coda del veicolo rigido. Quindi un semaforo rimane
% nello slot fino a quando la coda non ha liberato la stop-line.

[active_TL_idx, s_TL_active] = local_select_active_tl_slots(pos_hor_tail(1), s_TL, N_tl, K_tl, s_max);

for kk = 1:N+1
    idx0 = N_od*(kk-1);
    for jj = 1:K_tl
        OnlineData(idx0 + idx_sTL0 + jj) = s_TL_active(jj);
    end
end

%=========================================================================
% MEMORY SAFETY UPDATE FOR NON-SLOTTED TRAFFIC LIGHTS
%=========================================================================
% Aggiorna la memoria globale sugli 11 semafori anche se solo 4 vengono
% passati al solver. Questo evita incoerenze quando la finestra mobile cambia.

reached_TL = local_update_reached_memory_only(reached_TL_old, s_TL, pos_hor(1), pos_hor_tail(1), vel_hor(1), x_TL, N_tl);

%=========================================================================
% TRAFFIC LIGHT ONLINE DATA
%=========================================================================

[OnlineData, reached_TL, debug_TL, red_stopline_hold, s_hold] = local_update_tl_slots_for_vehicle( ...
    OnlineData, x_TL, s_TL, active_TL_idx, s_TL_active, ...
    pos_hor, pos_hor_tail, vel_hor, Vmax, reached_TL_old, reached_TL, debug_TL, ...
    time_cur, N, N_od, K_tl, idx_tail0, idx_head0, eps_TL_hold);

end

%==========================================================================
% LOCAL FUNCTION: BUS STOP ONLINE DATA
%==========================================================================
function [OnlineData, reached_stop, stop_dwell_count] = local_update_stop_online_data( ...
    OnlineData, reached_stop, reached_stop_old, stop_dwell_count, stop_enabled, ...
    s_stop, s_now, v_now, N_stops, N_dwell_steps, eps_stop_dwell, eps_vel_dwell, ...
    N, N_od, idx_sstop_active, idx_xstop_active, idx_xdwell_active, idx_wstop_active)

stop_in_dwell_now = zeros(1,N_stops);
reached_stop = reached_stop_old;

for jj = 1:N_stops
    if reached_stop(jj) == 1
        stop_dwell_count(jj) = N_dwell_steps;
        stop_in_dwell_now(jj) = 0;
    end
end

active_stop_idx = 0;
for jj = 1:N_stops
    if reached_stop(jj) == 0 && stop_enabled(jj) == 1
        active_stop_idx = jj;
        break
    end
end

if active_stop_idx > 0
    jj = active_stop_idx;
    s_bus_stop = s_stop(jj);

    bus_at_stop = abs(s_now - s_bus_stop) <= eps_stop_dwell && abs(v_now) <= eps_vel_dwell;
    bus_passed_stop = s_now >= s_bus_stop + eps_stop_dwell;
    dwell_latched = stop_dwell_count(jj) > 0 && stop_dwell_count(jj) < N_dwell_steps && reached_stop(jj) == 0;

    if dwell_latched
        stop_in_dwell_now(jj) = 1;
        stop_dwell_count(jj) = stop_dwell_count(jj) + 1;

        if stop_dwell_count(jj) >= N_dwell_steps
            reached_stop(jj) = 1;
            stop_dwell_count(jj) = N_dwell_steps;
            stop_in_dwell_now(jj) = 0;
        end

    elseif bus_at_stop
        stop_in_dwell_now(jj) = 1;
        stop_dwell_count(jj) = 1;

        if stop_dwell_count(jj) >= N_dwell_steps
            reached_stop(jj) = 1;
            stop_dwell_count(jj) = N_dwell_steps;
            stop_in_dwell_now(jj) = 0;
        end

    elseif bus_passed_stop
        reached_stop(jj) = 1;
        stop_dwell_count(jj) = N_dwell_steps;
        stop_in_dwell_now(jj) = 0;

    else
        stop_in_dwell_now(jj) = 0;
    end
end

next_stop_idx = 0;
for jj = 1:N_stops
    if reached_stop(jj) == 0 && stop_enabled(jj) == 1
        next_stop_idx = jj;
        break
    end
end

if next_stop_idx > 0
    jj = next_stop_idx;
    s_active = s_stop(jj);

    dwell_remaining = N_dwell_steps - stop_dwell_count(jj);
    if dwell_remaining < 0
        dwell_remaining = 0;
    end

    for kk = 2:N+1
        idx0 = N_od*(kk-1);
        OnlineData(idx0 + idx_sstop_active) = s_active;

        if stop_in_dwell_now(jj) == 1
            OnlineData(idx0 + idx_xstop_active) = 1;
            OnlineData(idx0 + idx_wstop_active) = 0;

            if kk <= dwell_remaining + 1
                OnlineData(idx0 + idx_xdwell_active) = 1;
            else
                OnlineData(idx0 + idx_xdwell_active) = 0;
            end
        else
            OnlineData(idx0 + idx_xstop_active)  = 0;
            OnlineData(idx0 + idx_xdwell_active) = 0;
            OnlineData(idx0 + idx_wstop_active)  = 1;
        end
    end
end

end

%==========================================================================
% LOCAL FUNCTION: SELECT 4 ACTIVE TRAFFIC LIGHT SLOTS
%==========================================================================
function [active_idx, active_s] = local_select_active_tl_slots(s_tail_now, s_TL, N_tl, K_tl, s_max)

active_idx = zeros(1,K_tl);
active_s   = s_max*ones(1,K_tl);

eps_select_TL = 0.30; % [m]
first_TL = 0;

for ii = 1:N_tl
    if s_TL(ii) > s_tail_now - eps_select_TL
        first_TL = ii;
        break
    end
end

if first_TL > 0
    for jj = 1:K_tl
        ii = first_TL + jj - 1;
        if ii <= N_tl
            active_idx(jj) = ii;
            active_s(jj)   = s_TL(ii);
        else
            active_idx(jj) = 0;
            active_s(jj)   = s_max;
        end
    end
end

end

%==========================================================================
% LOCAL FUNCTION: MEMORY-ONLY UPDATE FOR ALL PHYSICAL TLS
%==========================================================================
function reached_TL = local_update_reached_memory_only(reached_TL_old, s_TL, pos_now, pos_tail_now, vel_now, x_TL, N_tl)

reached_TL = reached_TL_old;

eps_cross_head      = 0.05; % [m]
eps_cross_tail      = 0.20; % [m]
d_stopline_deadband = 0.30; % [m]
v_deadband          = 0.02; % [m/s]

for ii = 1:N_tl
    if reached_TL_old(ii) == 1
        reached_TL(ii) = 1;
    else
        d_avail_now      = s_TL(ii) - pos_now;
        d_avail_tail_now = s_TL(ii) - pos_tail_now;

        head_crossed_now = d_avail_now      < -eps_cross_head;
        tail_crossed_now = d_avail_tail_now < -eps_cross_tail;

        head_stopped_at_light = abs(d_avail_now) <= d_stopline_deadband && abs(vel_now) <= v_deadband && ~head_crossed_now;
        head_stopped_at_red   = x_TL(1,ii) == 0 && head_stopped_at_light;

        if tail_crossed_now || head_crossed_now
            reached_TL(ii) = 1;
        elseif head_stopped_at_red
            reached_TL(ii) = 0;
        elseif head_stopped_at_light
            reached_TL(ii) = 0;
        else
            reached_TL(ii) = 0;
        end
    end
end

end

%==========================================================================
% LOCAL FUNCTION: TRAFFIC LIGHT LOGIC FOR ONE VEHICLE, 4 SLOTS
%==========================================================================
function [OnlineData, reached_TL, debug_TL, red_stopline_hold, s_hold] = local_update_tl_slots_for_vehicle( ...
    OnlineData, x_TL, s_TL, active_TL_idx, s_TL_active, ...
    pos_hor, pos_hor_tail, vel_hor, Vmax_local, reached_TL_old, reached_TL, debug_TL, ...
    time_cur, N, N_od, K_tl, idx_tail0, idx_head0, eps_TL_hold)

red_stopline_hold = 0.0;
s_hold            = 0.0;

for jj_slot = 1:K_tl

    ii = active_TL_idx(jj_slot);

    if ii == 0
        continue
    end

    main_case = 0;
    mem_case  = 0;

    current_green_end = 0;
    i_head_cross      = 0;
    i_tail_cross      = 0;
    green_window_ok   = 0;
    tail_at_green_end = -1e6;

    s_light = s_TL_active(jj_slot);

    eps_cross_head      = 0.05;   % [m]
    eps_cross_tail      = 0.20;   % [m]
    eps_clear_tail      = 0.20;   % [m]
    eps_cross_now       = eps_cross_head;

    d_stopline_deadband = 0.30;   % [m]
    v_deadband          = 0.02;   % [m/s]

    a_brake_guard       = 1.5;    % [m/s^2]
    d_margin_guard      = 0.20;   % [m]
    v_stop_thr_guard    = 0.20;   % [m/s]

    eps_line            = 0.30;   % [m]

    d_avail_now      = s_light - pos_hor(1);
    d_avail_tail_now = s_light - pos_hor_tail(1);

    head_crossed_now = d_avail_now      < -eps_cross_head;
    tail_crossed_now = d_avail_tail_now < -eps_cross_tail;

    head_stopped_at_light = abs(d_avail_now) <= d_stopline_deadband && abs(vel_hor(1)) <= v_deadband && ~head_crossed_now;
    head_stopped_at_red   = x_TL(1,ii) == 0 && head_stopped_at_light;

    reached_TL(ii) = reached_TL_old(ii);

    %======================================================================
    % CASE 1 FROM MEMORY
    %======================================================================

    if reached_TL_old(ii) == 1
        for kk = 1:N+1
            idx_head = idx_head0 + jj_slot + N_od*(kk-1);
            idx_tail = idx_tail0 + jj_slot + N_od*(kk-1);
            OnlineData(idx_head) = 1;
            OnlineData(idx_tail) = 1;
        end

        reached_TL(ii) = 1;
        main_case = 1;

        debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
            reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
            head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
            mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
        continue
    end

    %======================================================================
    % MEMORY UPDATE
    %======================================================================

    if tail_crossed_now
        reached_TL(ii) = 1;
        mem_case = 1;
    elseif head_crossed_now
        reached_TL(ii) = 1;
        mem_case = 2;
    elseif head_stopped_at_light
        reached_TL(ii) = 0;
        mem_case = 3;
    else
        reached_TL(ii) = 0;
        mem_case = 0;
    end

    %======================================================================
    % CASE 1 CURRENTLY PASSED
    %======================================================================

    if head_crossed_now || tail_crossed_now
        for kk = 1:N+1
            idx_head = idx_head0 + jj_slot + N_od*(kk-1);
            idx_tail = idx_tail0 + jj_slot + N_od*(kk-1);
            OnlineData(idx_head) = 1;
            OnlineData(idx_tail) = 1;
        end

        reached_TL(ii) = 1;
        main_case = 1;

        debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
            reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
            head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
            mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
        continue
    end

    %======================================================================
    % CASE 2: COPY BASE SPAT ON HORIZON
    %======================================================================

    OnlineData(idx_head0 + jj_slot) = 1;
    OnlineData(idx_tail0 + jj_slot) = 1;

    for kk = 2:N+1
        idx_head = idx_head0 + jj_slot + N_od*(kk-1);
        idx_tail = idx_tail0 + jj_slot + N_od*(kk-1);
        OnlineData(idx_head) = x_TL(kk,ii);
        OnlineData(idx_tail) = x_TL(kk,ii);
    end

    main_case = 2;

    %======================================================================
    % CASE 2B: STOP-LINE RED HOLD
    %======================================================================

    if head_stopped_at_red
        red_stopline_hold = 1;
        s_hold_guard = s_light - eps_TL_hold;
        s_hold = max(min(pos_hor(1), s_light), s_hold_guard);

        reached_TL(ii) = 0;
        mem_case = 3;

        for kk = 2:N+1
            idx_head = idx_head0 + jj_slot + N_od*(kk-1);
            idx_tail = idx_tail0 + jj_slot + N_od*(kk-1);
            OnlineData(idx_head) = 0;
            OnlineData(idx_tail) = 0;
        end

        main_case = 5.5;

        debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
            reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
            head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
            mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
        continue
    end

    %======================================================================
    % CASE 3: CURRENT GREEN CLEARABLE BY TAIL
    %======================================================================

    if x_TL(1,ii) == 1
        current_green_end = local_current_green_end(x_TL, ii, N);

        if current_green_end > 0
            tail_at_green_end = pos_hor_tail(current_green_end);

            if tail_at_green_end > s_light + eps_clear_tail
                for kk = 1:N+1
                    idx_head = idx_head0 + jj_slot + N_od*(kk-1);
                    idx_tail = idx_tail0 + jj_slot + N_od*(kk-1);
                    OnlineData(idx_head) = 1;
                    OnlineData(idx_tail) = 1;
                end

                main_case = 3;

                debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
                    reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
                    head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
                    mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
                continue
            end
        end
    end

    %======================================================================
    % CASE 4: HEAD DOES NOT REACH TL WITHIN HORIZON
    %======================================================================

    i_head_cross = 0;
    i_tail_cross = 0;

    for kk = 2:N+1
        if pos_hor(kk) >= s_light
            i_head_cross = kk;
            break
        end
    end

    if i_head_cross == 0
        main_case = 4;

        debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
            reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
            head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
            mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
        continue
    end

    %======================================================================
    % CASE 5 / 8 / 9: HEAD REACHES, TAIL MAY NOT CLEAR IN HORIZON
    %======================================================================

    for kk = 2:N+1
        if pos_hor_tail(kk) >= s_light
            i_tail_cross = kk;
            break
        end
    end

    if i_tail_cross == 0
        main_case = 5;

        if x_TL(1,ii) == 1
            current_green_end = local_current_green_end(x_TL, ii, N);

            if current_green_end > 0
                tail_at_green_end = pos_hor_tail(current_green_end);

                dist_tail_to_TL = s_light - pos_hor_tail(1);
                if dist_tail_to_TL < 0
                    dist_tail_to_TL = 0;
                end

                v0_clear = vel_hor(1);
                if v0_clear < 0
                    v0_clear = 0;
                end

                a_clear     = 0.7;  % [m/s^2]
                v_max_clear = Vmax_local;

                if dist_tail_to_TL <= 0
                    t_clear_tail = 0;
                else
                    if v0_clear >= v_max_clear || a_clear <= 0
                        t_clear_tail = dist_tail_to_TL / max(v0_clear, 0.1);
                    else
                        t_to_vmax = (v_max_clear - v0_clear) / a_clear;
                        d_to_vmax = v0_clear*t_to_vmax + 0.5*a_clear*t_to_vmax^2;

                        if dist_tail_to_TL <= d_to_vmax
                            t_clear_tail = (-v0_clear + sqrt(v0_clear^2 + 2*a_clear*dist_tail_to_TL)) / a_clear;
                        else
                            t_clear_tail = t_to_vmax + (dist_tail_to_TL - d_to_vmax) / v_max_clear;
                        end
                    end
                end

                kk_tail_clear_est = ceil(t_clear_tail/1.0) + 1;
                if kk_tail_clear_est < 2
                    kk_tail_clear_est = 2;
                end

                tail_margin_nodes = 1;

                %==========================================================
                % CASE 8: POSITIVE KINEMATIC FALLBACK
                %==========================================================

                if kk_tail_clear_est + tail_margin_nodes <= current_green_end
                    for kk = 2:N+1
                        idx_head = idx_head0 + jj_slot + N_od*(kk-1);
                        idx_tail = idx_tail0 + jj_slot + N_od*(kk-1);
                        OnlineData(idx_head) = 1;

                        if kk < kk_tail_clear_est
                            OnlineData(idx_tail) = 0;
                        else
                            OnlineData(idx_tail) = 1;
                        end
                    end

                    main_case       = 8;
                    green_window_ok = 1;

                    debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
                        reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
                        head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
                        mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
                    continue

                else
                    d_avail     = s_light - pos_hor(1);
                    d_stop_phys = vel_hor(1)^2/(2*a_brake_guard);

                    already_stopped_before_line = (vel_hor(1) <= v_stop_thr_guard) && (d_avail >= 0);
                    can_still_stop = already_stopped_before_line || (d_avail >= d_stop_phys + d_margin_guard);

                    stopline_green_release = x_TL(1,ii) == 1 && d_avail >= 0 && d_avail <= eps_line && vel_hor(1) <= v_stop_thr_guard;

                    if stopline_green_release
                        for kk_case9 = 1:N+1
                            idx_head = idx_head0 + jj_slot + N_od*(kk_case9-1);
                            idx_tail = idx_tail0 + jj_slot + N_od*(kk_case9-1);
                            OnlineData(idx_head) = 1;
                            OnlineData(idx_tail) = 1;
                        end

                        reached_TL(ii)  = 1;
                        mem_case        = 4;
                        main_case       = 9;
                        green_window_ok = 1;

                        debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
                            reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
                            head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
                            mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
                        continue

                    elseif can_still_stop || d_avail >= eps_line
                        for kk_case9 = 2:current_green_end
                            idx_head = idx_head0 + jj_slot + N_od*(kk_case9-1);
                            idx_tail = idx_tail0 + jj_slot + N_od*(kk_case9-1);
                            OnlineData(idx_head) = 0;
                            OnlineData(idx_tail) = 0;
                        end

                        main_case = 9.3;
                    end

                    green_window_ok = 0;

                    debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
                        reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
                        head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
                        mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
                    continue
                end
            end
        end

        %==================================================================
        % CASE 5 DEFAULT
        %==================================================================

        d_avail = s_light - pos_hor(1);
        case5_crossed = d_avail < -eps_cross_now;

        if case5_crossed
            for kk_case5 = 1:N+1
                idx_head = idx_head0 + jj_slot + N_od*(kk_case5-1);
                idx_tail = idx_tail0 + jj_slot + N_od*(kk_case5-1);
                OnlineData(idx_head) = 1;
                OnlineData(idx_tail) = 1;
            end

            reached_TL(ii) = 1;
            main_case = 10;
            mem_case  = 2;
        end

        debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
            reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
            head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
            mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
        continue
    end

    %======================================================================
    % CASE 6 / 7: COMMON GREEN WINDOW FOR HEAD AND TAIL
    %======================================================================

    green_info = local_find_green_window_vec(x_TL, ii, N, i_head_cross, i_tail_cross);

    green_window_ok = green_info(1);
    green_start     = green_info(2);
    green_end       = green_info(3);

    if green_window_ok == 1
        release_head_from = i_head_cross;
        release_tail_from = i_tail_cross;

        if green_start > 1
            if x_TL(green_start-1,ii) == 0 && x_TL(green_start,ii) == 1
                release_head_from = max(release_head_from, green_start);
                release_tail_from = max(release_tail_from, green_start);
            end
        end

        if release_head_from > green_end || release_tail_from > green_end
            main_case       = 7;
            green_window_ok = 0;
        else
            for kk = 2:N+1
                idx_head = idx_head0 + jj_slot + N_od*(kk-1);
                idx_tail = idx_tail0 + jj_slot + N_od*(kk-1);

                if kk < release_head_from
                    OnlineData(idx_head) = 0;
                else
                    OnlineData(idx_head) = 1;
                end

                if kk < release_tail_from
                    OnlineData(idx_tail) = 0;
                else
                    OnlineData(idx_tail) = 1;
                end
            end

            main_case = 6;
        end
    else
        main_case = 7;
    end

    %======================================================================
    % CASE 7 PROTECTION ON CURRENT GREEN
    %======================================================================

    if main_case == 7 && x_TL(1,ii) == 1
        current_green_end = local_current_green_end(x_TL, ii, N);

        if current_green_end > 0
            d_avail     = s_light - pos_hor(1);
            d_stop_phys = vel_hor(1)^2/(2*a_brake_guard);

            already_stopped_before_line = vel_hor(1) <= v_stop_thr_guard && d_avail >= 0;
            can_still_stop = already_stopped_before_line || d_avail >= d_stop_phys + d_margin_guard;

            stopline_green_release = x_TL(1,ii) == 1 && d_avail >= 0 && d_avail <= eps_line && vel_hor(1) <= v_stop_thr_guard;

            if stopline_green_release
                for kk_case7 = 1:N+1
                    idx_head = idx_head0 + jj_slot + N_od*(kk_case7-1);
                    idx_tail = idx_tail0 + jj_slot + N_od*(kk_case7-1);
                    OnlineData(idx_head) = 1;
                    OnlineData(idx_tail) = 1;
                end

                reached_TL(ii)  = 1;
                mem_case        = 4;
                main_case       = 7.3;
                green_window_ok = 1;

            elseif can_still_stop || d_avail >= eps_line
                for kk_case7 = 2:current_green_end
                    idx_head = idx_head0 + jj_slot + N_od*(kk_case7-1);
                    idx_tail = idx_tail0 + jj_slot + N_od*(kk_case7-1);
                    OnlineData(idx_head) = 0;
                    OnlineData(idx_tail) = 0;
                end

                green_window_ok = 0;
                main_case = 7.6;
            end
        end
    end

    debug_TL(ii,:) = local_debug_row_slot(time_cur, ii, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
        reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
        head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
        mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, green_window_ok, tail_at_green_end);
end

end

%==========================================================================
% LOCAL FUNCTION: DEBUG ROW FOR SLOT-BASED TL LOGIC
%==========================================================================
function row = local_debug_row_slot(time_cur, ii_phys, jj_slot, s_light, pos_hor, pos_hor_tail, vel_hor, ...
    reached_TL_old, reached_TL, x_TL, OnlineData, N_od, idx_head0, idx_tail0, ...
    head_crossed_now, tail_crossed_now, head_stopped_at_light, ...
    mem_case, main_case, current_green_end, i_head_cross, i_tail_cross, ...
    green_window_ok, tail_at_green_end)

row = zeros(1,26);

idx_head_k1 = idx_head0 + jj_slot;
idx_tail_k1 = idx_tail0 + jj_slot;
idx_head_k2 = idx_head0 + jj_slot + N_od;
idx_tail_k2 = idx_tail0 + jj_slot + N_od;

row(1)  = time_cur;
row(2)  = ii_phys;
row(3)  = s_light;
row(4)  = pos_hor(1);
row(5)  = vel_hor(1);
row(6)  = pos_hor_tail(1);

row(7)  = reached_TL_old(ii_phys);
row(8)  = reached_TL(ii_phys);

row(9)  = x_TL(1,ii_phys);
row(10) = x_TL(2,ii_phys);
row(11) = x_TL(3,ii_phys);

row(12) = double(head_crossed_now);
row(13) = double(tail_crossed_now);
row(14) = double(head_stopped_at_light);

row(15) = mem_case;
row(16) = main_case;

row(17) = current_green_end;
row(18) = i_head_cross;
row(19) = i_tail_cross;
row(20) = green_window_ok;
row(21) = tail_at_green_end;

row(22) = OnlineData(idx_head_k1);
row(23) = OnlineData(idx_tail_k1);
row(24) = OnlineData(idx_head_k2);
row(25) = OnlineData(idx_tail_k2);

row(26) = s_light - pos_hor(1);

end

%==========================================================================
% LOCAL FUNCTION: FIND GREEN WINDOW
%==========================================================================
function green_info = local_find_green_window_vec(x_TL, ii, N, i_head_cross, i_tail_cross)

green_window_ok = 0;
green_start = 0;
green_end = 0;

kk = 1;

while kk <= N + 1
    if x_TL(kk,ii) == 1
        run_start = kk;

        while kk <= N + 1 && x_TL(kk,ii) == 1
            kk = kk + 1;
        end

        run_end = kk - 1;

        if i_head_cross >= run_start && i_tail_cross <= run_end
            green_window_ok = 1;
            green_start = run_start;
            green_end = run_end;
            break
        end
    else
        kk = kk + 1;
    end
end

green_info = zeros(1,3);
green_info(1) = green_window_ok;
green_info(2) = green_start;
green_info(3) = green_end;

end

%==========================================================================
% LOCAL FUNCTION: CURRENT GREEN END
%==========================================================================
function green_end = local_current_green_end(x_TL, ii, N)

green_end = 0;

if x_TL(1,ii) == 0
    return
end

kk = 1;
while kk <= N+1 && x_TL(kk,ii) == 1
    green_end = kk;
    kk = kk + 1;
end

end

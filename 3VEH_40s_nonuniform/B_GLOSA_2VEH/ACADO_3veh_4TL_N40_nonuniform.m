%==========================================================================
% ACADO NMPC - B-GLOSA 3 veicoli indipendenti - 4 TL attivi
%==========================================================================
% Script per generazione/export solver NMPC ACADO.
%
% Estensione della versione 2 veicoli:
% - aggiunge stati [pos3, vel3, acc3];
% - aggiunge controllo jerk3;
% - estende OnlineData da 42 a 63 elementi per nodo;
% - replica vincoli semaforici, fermate, velocita', accelerazione e jerk
%   per il terzo veicolo;
% - aggiunge vincolo e costo di spacing tra veicolo 2 e veicolo 3.
%=========================================================================
clc;
clear all;
close all;

%% ========================================================================
%  1. DEFINIZIONE DEL PROBLEMA DI CONTROLLO
%  ========================================================================

Ts_ctrl = 1;      % [s] intervallo di aggiornamento NMPC/Simulink, non necessariamente uguale alla griglia predittiva
EXPORT  = 1;
COMPILE = 1;

% Stati del modello predittivo:
% veicolo 1: pos,  vel,  acc
% veicolo 2: pos2, vel2, acc2
% veicolo 3: pos3, vel3, acc3
DifferentialState pos vel acc pos2 vel2 acc2 pos3 vel3 acc3;

% Ingressi di controllo: jerk longitudinale di ciascun veicolo
Control jerk jerk2 jerk3;

% OnlineData per nodo: 63 = 21 dati per veicolo.
% Per ciascun veicolo:
%   4 flag TL testa, stop active, flag stop/dwell/target, k_road, Vmax,
%   schedule placeholders, 4 flag TL coda, 4 coordinate TL attive.
OnlineData x_TL1 x_TL2 x_TL3 x_TL4 ...
    s_stop_active x_stop_active x_dwell_active w_stop_active k_road Vmax dt_schd s_st s_hor ...
    x_TL1_tail x_TL2_tail x_TL3_tail x_TL4_tail ...
    s_TL1_active s_TL2_active s_TL3_active s_TL4_active ...
    x_TL1_2 x_TL2_2 x_TL3_2 x_TL4_2 ...
    s_stop_active_2 x_stop_active_2 x_dwell_active_2 w_stop_active_2 k_road_2 Vmax_2 dt_schd_2 s_st_2 s_hor_2 ...
    x_TL1_tail_2 x_TL2_tail_2 x_TL3_tail_2 x_TL4_tail_2 ...
    s_TL1_active_2 s_TL2_active_2 s_TL3_active_2 s_TL4_active_2 ...
    x_TL1_3 x_TL2_3 x_TL3_3 x_TL4_3 ...
    s_stop_active_3 x_stop_active_3 x_dwell_active_3 w_stop_active_3 k_road_3 Vmax_3 dt_schd_3 s_st_3 s_hor_3 ...
    x_TL1_tail_3 x_TL2_tail_3 x_TL3_tail_3 x_TL4_tail_3 ...
    s_TL1_active_3 s_TL2_active_3 s_TL3_active_3 s_TL4_active_3;

%% ========================================================================
%  2. PARAMETRI PRINCIPALI DELLO SCENARIO E DEL MODELLO
%  ========================================================================

s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0]; 

s_max   = 5200;        % lunghezza path [m]
t_dwell = 10;          % dwell time [s]
V_max   = 50/3.6;      % limite statico per vincoli dwell [m/s]
Ax_min  = -1.5;        % limite inferiore accelerazione longitudinale [m/s^2]
Ax_max  =  1.5;        % limite superiore accelerazione longitudinale [m/s^2]
Ay_max  =  2.0;        % limite accelerazione laterale [m/s^2]
jerk_min = -0.5;       % limite inferiore jerk [m/s^3]
jerk_max =  0.5;       % limite superiore jerk [m/s^3]

L_platoon = 7.8;        % lunghezza veicolo / ingombro longitudinale [m]

% Gap netti tra veicoli adiacenti.
gap12 = pos  - pos2 - L_platoon;
gap23 = pos2 - pos3 - L_platoon;

%% ========================================================================
%  3. PARAMETRI FERMATE BUS
%  ========================================================================

eps_stop_back  = 0.5;  % tolleranza dietro fermata [m]
eps_stop_front = 0.5;  % tolleranza davanti fermata [m]
eps_stop_v     = 0.1;  % velocita' massima durante dwell [m/s]
eps_stop_a     = 0.3;  % accelerazione ammessa durante dwell [m/s^2]

%% ========================================================================
%  4. EQUAZIONI DI STATO
%  ========================================================================

pos_dot  = is(vel);
vel_dot  = is(acc);
acc_dot  = is(jerk);

pos_dot2 = is(vel2);
vel_dot2 = is(acc2);
acc_dot2 = is(jerk2);

pos_dot3 = is(vel3);
vel_dot3 = is(acc3);
acc_dot3 = is(jerk3);

f = [dot(pos);  dot(vel);  dot(acc); ...
     dot(pos2); dot(vel2); dot(acc2); ...
     dot(pos3); dot(vel3); dot(acc3)] == ...
    [pos_dot;   vel_dot;   acc_dot; ...
     pos_dot2;  vel_dot2;  acc_dot2; ...
     pos_dot3;  vel_dot3;  acc_dot3];

%% ========================================================================
%  5. TERMINI DELLA COST FUNCTION
%  ========================================================================

%--------------------------------------------------------------------------
% 5.1 Avanzamento, accelerazione longitudinale e jerk
%--------------------------------------------------------------------------

cost_dist  = is(s_max - pos)  / s_max;
cost_dist2 = is(s_max - pos2) / s_max;
cost_dist3 = is(s_max - pos3) / s_max;

cost_jerk  = is(jerk)  / jerk_max;
cost_jerk2 = is(jerk2) / jerk_max;
cost_jerk3 = is(jerk3) / jerk_max;

cost_Ax  = is(acc)  / Ax_max;
cost_Ax2 = is(acc2) / Ax_max;
cost_Ax3 = is(acc3) / Ax_max;

%--------------------------------------------------------------------------
% 5.2 Costo spacing / beneficio aerodinamico tra veicoli adiacenti
%--------------------------------------------------------------------------
% Il costo agisce solo quando il gap e' inferiore a gap_on. Oltre gap_on
% si spegne in modo C1 tramite atan, evitando di penalizzare differenze tra
% veicoli lontani.
%--------------------------------------------------------------------------

d_gap     = 3;       % gap desiderato netto [m]
gap_on    = 6;      % distanza oltre cui il costo si spegne [m]
sigma_gap = 1;    % morbidezza transizione [m]

aero_activation12 = 0.5 - atan((gap12 - gap_on)/sigma_gap)/pi;
aero_activation23 = 0.5 - atan((gap23 - gap_on)/sigma_gap)/pi;

cost_aero12 = is(aero_activation12 * (gap12 - d_gap)/d_gap);
cost_aero23 = is(aero_activation23 * (gap23 - d_gap)/d_gap);

gap_min = 0.5;        % [m] oppure 1.0 per più margine numerico
%gap_slack_scale = 0.1;

% cost_slack_gap12 = is(s_gap12 / gap_slack_scale);
% cost_slack_gap23 = is(s_gap23 / gap_slack_scale);
%--------------------------------------------------------------------------
% 5.3 Termini associati alle fermate bus
%--------------------------------------------------------------------------

cost_stop_target  = is(w_stop_active   * (pos  - s_stop_active)   / s_max);
cost_stop_target2 = is(w_stop_active_2 * (pos2 - s_stop_active_2) / s_max);
cost_stop_target3 = is(w_stop_active_3 * (pos3 - s_stop_active_3) / s_max);

cost_dwell_pos  = is(x_dwell_active   * (pos  - s_stop_active)   / eps_stop_front);
cost_dwell_pos2 = is(x_dwell_active_2 * (pos2 - s_stop_active_2) / eps_stop_front);
cost_dwell_pos3 = is(x_dwell_active_3 * (pos3 - s_stop_active_3) / eps_stop_front);

cost_dwell_vel  = is(x_dwell_active   * vel  / eps_stop_v);
cost_dwell_vel2 = is(x_dwell_active_2 * vel2 / eps_stop_v);
cost_dwell_vel3 = is(x_dwell_active_3 * vel3 / eps_stop_v);

cost_dwell_acc  = is(x_dwell_active   * acc  / eps_stop_a);
cost_dwell_acc2 = is(x_dwell_active_2 * acc2 / eps_stop_a);
cost_dwell_acc3 = is(x_dwell_active_3 * acc3 / eps_stop_a);

%--------------------------------------------------------------------------
% 5.4 Assemblaggio stage cost e terminal cost
%--------------------------------------------------------------------------
% Ordine dei pesi nel file init:
%  1-3   dist 1,2,3
%  4-6   jerk 1,2,3
%  7-9   Ax 1,2,3
%  10-12 stop target 1,2,3
%  13-15 dwell position 1,2,3
%  16-18 dwell velocity 1,2,3
%  19-21 dwell acceleration 1,2,3
%  22-23 spacing/aero 1-2, 2-3
%--------------------------------------------------------------------------

h = {cost_dist; cost_dist2; cost_dist3; ...
     cost_jerk; cost_jerk2; cost_jerk3; ...
     cost_Ax; cost_Ax2; cost_Ax3; ...
     cost_stop_target; cost_stop_target2; cost_stop_target3; ...
     cost_dwell_pos; cost_dwell_pos2; cost_dwell_pos3; ...
     cost_dwell_vel; cost_dwell_vel2; cost_dwell_vel3; ...
     cost_dwell_acc; cost_dwell_acc2; cost_dwell_acc3; ...
     cost_aero12; cost_aero23}; ...
     %cost_slack_gap12; cost_slack_gap23};

hN = {cost_dist; cost_dist2; cost_dist3};

%% ========================================================================
%  6. DEFINIZIONE ED EXPORT DEL PROBLEMA MPC
%  ========================================================================

fprintf('----------------------------\n         NMPC-export         \n----------------------------\n');
acadoSet('problemname', 'NMPC');

% Griglia predittiva non uniforme: 40 intervalli, orizzonte totale 50 s
% - intervalli 1..20 : 1.0 s  -> [0, 20] s
% - intervalli 21..40: 1.5 s  -> [20, 50] s
N_near  = 20;
N_far   = 20;
Ts_near = 1.0;
Ts_far  = 1.5;

T1 = N_near*Ts_near;
timepoints = [0:Ts_near:T1, T1 + Ts_far*(1:N_far)];

N = numel(timepoints) - 1;
T_hor = timepoints(end);

if N ~= 40 || abs(T_hor - 50.0) > 1e-12
    error('Griglia NMPC non coerente: N=%d, T_hor=%.6f s.', N, T_hor);
end

ocp = acado.OCP(timepoints);
W  = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));

ocp.minimizeLSQ(W, h);
ocp.minimizeLSQEndTerm(WN, hN);

%% ========================================================================
%  7. VINCOLI DELL'OCP
%  ========================================================================

%--------------------------------------------------------------------------
% 7.1 Vincoli comfort: accelerazione longitudinale, jerk, accelerazione laterale
%--------------------------------------------------------------------------

ocp.subjectTo(acc  <= Ax_max);
ocp.subjectTo(acc  >= Ax_min);
ocp.subjectTo(acc2 <= Ax_max);
ocp.subjectTo(acc2 >= Ax_min);
ocp.subjectTo(acc3 <= Ax_max);
ocp.subjectTo(acc3 >= Ax_min);

ocp.subjectTo(jerk  <= jerk_max);
ocp.subjectTo(jerk  >= jerk_min);
ocp.subjectTo(jerk2 <= jerk_max);
ocp.subjectTo(jerk2 >= jerk_min);
ocp.subjectTo(jerk3 <= jerk_max);
ocp.subjectTo(jerk3 >= jerk_min);

ocp.subjectTo(is(vel  * vel  * k_road   - Ay_max) <= 0);
ocp.subjectTo(is(vel2 * vel2 * k_road_2 - Ay_max) <= 0);
ocp.subjectTo(is(vel3 * vel3 * k_road_3 - Ay_max) <= 0);

%--------------------------------------------------------------------------
% 7.2 Vincoli di velocita' stradale e non-negativita'
%--------------------------------------------------------------------------

ocp.subjectTo(is(vel  - Vmax)   <= 0);
ocp.subjectTo(is(vel2 - Vmax_2) <= 0);
ocp.subjectTo(is(vel3 - Vmax_3) <= 0);

v_eps = 0.01;
ocp.subjectTo(vel  >= -v_eps);
ocp.subjectTo(vel2 >= -v_eps);
ocp.subjectTo(vel3 >= -v_eps);

%--------------------------------------------------------------------------
% 7.3 Vincoli minimi sui gap tra veicoli adiacenti
%--------------------------------------------------------------------------

% ocp.subjectTo(s_gap12 >= 0);
% ocp.subjectTo(s_gap23 >= 0);
% eps_gap_max = 0.5;   % limite massimo di rilassamento accettabile [m]
% ocp.subjectTo(s_gap12 <= eps_gap_max);
% ocp.subjectTo(s_gap23 <= eps_gap_max);

ocp.subjectTo(gap12 >= gap_min);
ocp.subjectTo(gap23 >= gap_min);

%--------------------------------------------------------------------------
% 7.4 Vincoli fermate bus
%--------------------------------------------------------------------------
eps_stop_num = 0.0;   % [m] tolleranza fermata: 5 cm
% Bus stop - vincolo di approccio
ocp.subjectTo(is(pos  - ((s_stop_active   + eps_stop_num)*(1-x_stop_active)   + s_max*x_stop_active))   <= 0);
ocp.subjectTo(is(pos2 - ((s_stop_active_2 + eps_stop_num)*(1-x_stop_active_2) + s_max*x_stop_active_2)) <= 0);
ocp.subjectTo(is(pos3 - ((s_stop_active_3 + eps_stop_num)*(1-x_stop_active_3) + s_max*x_stop_active_3)) <= 0);

% Bus stop - dwell box
ocp.subjectTo(is(pos  - ((s_stop_active   + eps_stop_front)*x_dwell_active   + s_max*(1-x_dwell_active)))   <= 0);
ocp.subjectTo(is((s_stop_active   - eps_stop_back)*x_dwell_active   - s_max*(1-x_dwell_active)   - pos)  <= 0);

ocp.subjectTo(is(pos2 - ((s_stop_active_2 + eps_stop_front)*x_dwell_active_2 + s_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is((s_stop_active_2 - eps_stop_back)*x_dwell_active_2 - s_max*(1-x_dwell_active_2) - pos2) <= 0);

ocp.subjectTo(is(pos3 - ((s_stop_active_3 + eps_stop_front)*x_dwell_active_3 + s_max*(1-x_dwell_active_3))) <= 0);
ocp.subjectTo(is((s_stop_active_3 - eps_stop_back)*x_dwell_active_3 - s_max*(1-x_dwell_active_3) - pos3) <= 0);

% Bus stop - dwell velocity and acceleration
ocp.subjectTo(is(vel  - (eps_stop_v*x_dwell_active   + V_max*(1-x_dwell_active)))   <= 0);
ocp.subjectTo(is(acc  - (eps_stop_a*x_dwell_active   + Ax_max*(1-x_dwell_active)))  <= 0);
ocp.subjectTo(is(acc  + (eps_stop_a*x_dwell_active   + abs(Ax_min)*(1-x_dwell_active)))   >= 0);

ocp.subjectTo(is(vel2 - (eps_stop_v*x_dwell_active_2 + V_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is(acc2 - (eps_stop_a*x_dwell_active_2 + Ax_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is(acc2 + (eps_stop_a*x_dwell_active_2 + abs(Ax_min)*(1-x_dwell_active_2))) >= 0);

ocp.subjectTo(is(vel3 - (eps_stop_v*x_dwell_active_3 + V_max*(1-x_dwell_active_3))) <= 0);
ocp.subjectTo(is(acc3 - (eps_stop_a*x_dwell_active_3 + Ax_max*(1-x_dwell_active_3))) <= 0);
ocp.subjectTo(is(acc3 + (eps_stop_a*x_dwell_active_3 + abs(Ax_min)*(1-x_dwell_active_3))) >= 0);

%--------------------------------------------------------------------------
% 7.5 Vincoli semaforici con 4 slot mobili per ciascun veicolo
%--------------------------------------------------------------------------

d_safe_TL = 0.0;

% Testa veicolo 1
ocp.subjectTo(is(pos - ((s_TL1_active-d_safe_TL)*(1-x_TL1) + s_max*x_TL1)) <= 0);
ocp.subjectTo(is(pos - ((s_TL2_active-d_safe_TL)*(1-x_TL2) + s_max*x_TL2)) <= 0);
ocp.subjectTo(is(pos - ((s_TL3_active-d_safe_TL)*(1-x_TL3) + s_max*x_TL3)) <= 0);
ocp.subjectTo(is(pos - ((s_TL4_active-d_safe_TL)*(1-x_TL4) + s_max*x_TL4)) <= 0);

% Coda veicolo 1
ocp.subjectTo(is((pos-L_platoon) - ((s_TL1_active-d_safe_TL)*(1-x_TL1_tail) + s_max*x_TL1_tail)) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL2_active-d_safe_TL)*(1-x_TL2_tail) + s_max*x_TL2_tail)) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL3_active-d_safe_TL)*(1-x_TL3_tail) + s_max*x_TL3_tail)) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL4_active-d_safe_TL)*(1-x_TL4_tail) + s_max*x_TL4_tail)) <= 0);

% Testa veicolo 2
ocp.subjectTo(is(pos2 - ((s_TL1_active_2-d_safe_TL)*(1-x_TL1_2) + s_max*x_TL1_2)) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL2_active_2-d_safe_TL)*(1-x_TL2_2) + s_max*x_TL2_2)) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL3_active_2-d_safe_TL)*(1-x_TL3_2) + s_max*x_TL3_2)) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL4_active_2-d_safe_TL)*(1-x_TL4_2) + s_max*x_TL4_2)) <= 0);

% Coda veicolo 2
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL1_active_2-d_safe_TL)*(1-x_TL1_tail_2) + s_max*x_TL1_tail_2)) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL2_active_2-d_safe_TL)*(1-x_TL2_tail_2) + s_max*x_TL2_tail_2)) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL3_active_2-d_safe_TL)*(1-x_TL3_tail_2) + s_max*x_TL3_tail_2)) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL4_active_2-d_safe_TL)*(1-x_TL4_tail_2) + s_max*x_TL4_tail_2)) <= 0);

% Testa veicolo 3
ocp.subjectTo(is(pos3 - ((s_TL1_active_3-d_safe_TL)*(1-x_TL1_3) + s_max*x_TL1_3)) <= 0);
ocp.subjectTo(is(pos3 - ((s_TL2_active_3-d_safe_TL)*(1-x_TL2_3) + s_max*x_TL2_3)) <= 0);
ocp.subjectTo(is(pos3 - ((s_TL3_active_3-d_safe_TL)*(1-x_TL3_3) + s_max*x_TL3_3)) <= 0);
ocp.subjectTo(is(pos3 - ((s_TL4_active_3-d_safe_TL)*(1-x_TL4_3) + s_max*x_TL4_3)) <= 0);

% Coda veicolo 3
ocp.subjectTo(is((pos3-L_platoon) - ((s_TL1_active_3-d_safe_TL)*(1-x_TL1_tail_3) + s_max*x_TL1_tail_3)) <= 0);
ocp.subjectTo(is((pos3-L_platoon) - ((s_TL2_active_3-d_safe_TL)*(1-x_TL2_tail_3) + s_max*x_TL2_tail_3)) <= 0);
ocp.subjectTo(is((pos3-L_platoon) - ((s_TL3_active_3-d_safe_TL)*(1-x_TL3_tail_3) + s_max*x_TL3_tail_3)) <= 0);
ocp.subjectTo(is((pos3-L_platoon) - ((s_TL4_active_3-d_safe_TL)*(1-x_TL4_tail_3) + s_max*x_TL4_tail_3)) <= 0);

%% ========================================================================
%  8. SETTAGGI ACADO, EXPORT E COMPILAZIONE
%  ========================================================================

tic;
ocp.setModel(f);

mpc = acado.OCPexport(ocp);
mpc.set('HESSIAN_APPROXIMATION',       'GAUSS_NEWTON');
mpc.set('DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING');
mpc.set('SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set('INTEGRATOR_TYPE',             'INT_RK4');
%mpc.set('NUM_INTEGRATOR_STEPS',        N);
mpc.set('NUM_INTEGRATOR_STEPS',        40);
mpc.set('QP_SOLVER',                   'QP_QPOASES3');
mpc.set('GENERATE_SIMULINK_INTERFACE', 'YES');
mpc.set('LEVENBERG_MARQUARDT',         1e-4);
mpc.set('MAX_NUM_QP_ITERATIONS',       1400);   % limita iterazioni QP: bound sul tempo di esecuzione

if EXPORT
    mpc.exportCode('export_NMPC_R_1');
end

if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_NMPC_R_1/qpoases3');
    make_custom_solver_sfunction_R_1;
end

toc

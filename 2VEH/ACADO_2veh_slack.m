%==========================================================================
% ACADO NMPC - B-GLOSA 2 veicoli indipendenti - 4 TL attivi

%==========================================================================
clc;
clear all;
close all;

%% ========================================================================
%%  1. DEFINIZIONE DEL PROBLEMA DI CONTROLLO
%%  ========================================================================

Ts_ctrl = 1;      % [s] intervallo di aggiornamento NMPC/Simulink
EXPORT  = 1;
COMPILE = 1;

% Stati del modello predittivo:
% veicolo 1: pos,  vel,  acc
% veicolo 2: pos2, vel2, acc2
DifferentialState pos vel acc pos2 vel2 acc2;

% Ingressi di controllo: jerk longitudinale di ciascun veicolo
Control jerk jerk2 ;%s_Jmin1 s_Jmin2;

% OnlineData per nodo: 42 = 21 dati per veicolo.
OnlineData x_TL1 x_TL2 x_TL3 x_TL4 ...
    s_stop_active x_stop_active x_dwell_active w_stop_active k_road Vmax dt_schd s_st s_hor ...
    x_TL1_tail x_TL2_tail x_TL3_tail x_TL4_tail ...
    s_TL1_active s_TL2_active s_TL3_active s_TL4_active ...
    x_TL1_2 x_TL2_2 x_TL3_2 x_TL4_2 ...
    s_stop_active_2 x_stop_active_2 x_dwell_active_2 w_stop_active_2 k_road_2 Vmax_2 dt_schd_2 s_st_2 s_hor_2 ...
    x_TL1_tail_2 x_TL2_tail_2 x_TL3_tail_2 x_TL4_tail_2 ...
    s_TL1_active_2 s_TL2_active_2 s_TL3_active_2 s_TL4_active_2;

%% ========================================================================
%%  2. PARAMETRI PRINCIPALI
%%  ========================================================================

s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0];

s_max   = 5200;
t_dwell = 10;
V_max   = 50/3.6;
Ax_min  = -1.5;
Ax_max  =  1.5;
Ay_max  =  2.0;
jerk_min = -0.5;
jerk_max =  0.5;

L_platoon = 7.8;

gap12 = pos  - pos2 - L_platoon;

%% ========================================================================
%%  3. PARAMETRI FERMATE BUS
%%  ========================================================================

%==========================================================================
% PARAMETRI NUMERICI / TOLLERANZE VINCOLI
%==========================================================================

eps_stop_back  = 0.5;   
% [m] tolleranza posteriore della box di dwell alla fermata.
% Quando x_dwell_active = 1, il veicolo è ammesso fino a 0.5 m prima della
% coordinata nominale della fermata: pos >= s_stop_active - eps_stop_back.
% Serve a evitare che il solver debba fermarsi esattamente in un punto
% geometrico infinitesimo.

eps_stop_front = 0.5;   
% [m] tolleranza anteriore della box di dwell alla fermata.
% Quando x_dwell_active = 1, il veicolo è ammesso fino a 0.5 m oltre la
% coordinata nominale della fermata: pos <= s_stop_active + eps_stop_front.
% Definisce insieme a eps_stop_back la finestra spaziale accettabile della fermata.

eps_stop_v     = 0.1;   
% [m/s] velocità massima ammessa durante il dwell.
% Quando x_dwell_active = 1, il vincolo diventa vel <= eps_stop_v.
% Non impone velocità esattamente nulla, ma permette una piccola tolleranza
% numerica pari a 0.1 m/s, cioè circa 0.36 km/h.

eps_stop_a     = 0.3;   
% [m/s^2] accelerazione massima in valore assoluto ammessa durante il dwell.
% Quando x_dwell_active = 1, il vincolo diventa:
% -eps_stop_a <= acc <= eps_stop_a.
% Serve a mantenere il veicolo quasi fermo anche dinamicamente, senza imporre
% acc = 0 come vincolo hard esatto.

v_eps = 0.02;   
% [m/s] tolleranza numerica sulla velocità minima.
% Il vincolo reale desiderato è vel >= 0, cioè niente retromarcia.
% In ACADO viene imposto vel >= -v_eps per evitare infeasibility dovute a
% piccole oscillazioni numeriche vicino a velocità nulla.
% Con v_eps = 0.05 si ammette una velocità negativa massima di 0.18 km/h,
% fisicamente trascurabile.

%gap_min = 1;   
% [m] distanza minima hard tra veicolo 1 e veicolo 2.
% Il vincolo è gap12 = pos - pos2 - L_platoon >= gap_min.
% Impedisce la collisione geometrica tra i veicoli lasciando almeno 0.5 m
% tra la coda del veicolo davanti e la testa del veicolo dietro.

%d_gap = 3;   
% [m] distanza desiderata usata nel costo di spacing/aerodinamica.
% Non è un vincolo hard: entra nella cost function tramite cost_aero12.
% Il costo tende a favorire gap12 vicino a d_gap quando il termine è attivo.
% Deve essere maggiore di gap_min, perché gap_min è il limite di sicurezza,
% mentre d_gap è il target desiderato.

%gap_on = 5;   
% [m] soglia di attivazione progressiva del costo di spacing/aerodinamica.
% Quando gap12 è molto maggiore di gap_on, il costo di spacing è quasi spento.
% Quando gap12 scende sotto circa gap_on, il costo inizia ad attivarsi.
% Serve a evitare che il secondo veicolo venga sempre attratto verso il primo
% anche quando è ancora lontano.

%sigma_gap = 1;   
% [m] parametro di smoothing della funzione di attivazione del costo gap.
% Regola quanto è graduale il passaggio tra costo spento e costo attivo.
% Valori piccoli rendono l'attivazione più brusca; valori grandi la rendono
% più morbida. Con sigma_gap = 1 la transizione avviene su circa pochi metri.

sJmin_max = 1.0;   
% [m/s^3] valore massimo ammesso per lo slack sul vincolo inferiore del jerk.
% Il vincolo soft è:
% jerk - jerk_min + s_Jmin >= 0, con 0 <= s_Jmin <= sJmin_max.
% Serve a rilassare temporaneamente il limite jerk >= jerk_min nei casi in cui
% il problema sarebbe altrimenti infeasible.
% Con jerk_min = -0.5 e sJmin_max = 1.0, il jerk minimo effettivo può arrivare
% al massimo a -1.5 m/s^3 solo se lo slack viene usato.


%% ========================================================================
%%  4. EQUAZIONI DI STATO
%%  ========================================================================

pos_dot  = is(vel);
vel_dot  = is(acc);
acc_dot  = is(jerk);

pos_dot2 = is(vel2);
vel_dot2 = is(acc2);
acc_dot2 = is(jerk2);

f = [dot(pos);  dot(vel);  dot(acc); ...
     dot(pos2); dot(vel2); dot(acc2)] == ...
    [pos_dot;   vel_dot;   acc_dot; ...
     pos_dot2;  vel_dot2;  acc_dot2];

%% ========================================================================
%%  5. TERMINI DELLA COST FUNCTION
%%  ========================================================================

cost_dist  = is(s_max - pos)  / s_max;
cost_dist2 = is(s_max - pos2) / s_max;

cost_jerk  = is(jerk)  / jerk_max;
cost_jerk2 = is(jerk2) / jerk_max;

cost_Ax  = is(acc)  / Ax_max;
cost_Ax2 = is(acc2) / Ax_max;

%cost_Jmin1 = is(s_Jmin1)/abs(jerk_min);
%cost_Jmin2 = is(s_Jmin2)/abs(jerk_min);



% aero_activation12 = 0.5 - atan((gap12 - gap_on)/sigma_gap)/pi;
% cost_aero12 = is(aero_activation12 * (gap12 - d_gap)/d_gap);
d_gap    = 3;    % gap netto desiderato [m]
eps_aero = 0.1;  % [m^2] smoothing transizione in gap=d_gap (errore <0.01 a 1m di distanza)
gap_min  = 1;  % [m] gap minimo assoluto (vincolo hard)
% smooth_neg: approssimazione smooth di min(0, gap - d_gap)
%   negativo per gap < d_gap, decadimento ~eps/(2*(gap-d_gap)) per gap > d_gap
smooth_neg12 = (is(gap12 - d_gap) - sqrt(is((gap12 - d_gap)*(gap12 - d_gap)) + eps_aero)) / 2;
cost_aero12 = is(smooth_neg12 / d_gap);




cost_stop_target  = is(w_stop_active   * (pos  - s_stop_active)   / s_max);
cost_stop_target2 = is(w_stop_active_2 * (pos2 - s_stop_active_2) / s_max);

cost_dwell_pos  = is(x_dwell_active   * (pos  - s_stop_active)   / eps_stop_front);
cost_dwell_pos2 = is(x_dwell_active_2 * (pos2 - s_stop_active_2) / eps_stop_front);

cost_dwell_vel  = is(x_dwell_active   * vel  / eps_stop_v);
cost_dwell_vel2 = is(x_dwell_active_2 * vel2 / eps_stop_v);

cost_dwell_acc  = is(x_dwell_active   * acc  / eps_stop_a);
cost_dwell_acc2 = is(x_dwell_active_2 * acc2 / eps_stop_a);

% Assemblaggio stage cost e terminal cost
% Ordine dei pesi nel file init:
%  1-2   dist 1,2
%  3-4   jerk 1,2
%  5-6   Ax 1,2
%  7-8   stop target 1,2
%  9-10  dwell position 1,2
% 11-12  dwell velocity 1,2
% 13-14  dwell acceleration 1,2
%    15  spacing/aero 1-2
% 16-17  slack Jmin 1,2

h = {cost_dist; cost_dist2; ...
     cost_jerk; cost_jerk2; ...
     cost_Ax; cost_Ax2; ...
     cost_stop_target; cost_stop_target2; ...
     cost_dwell_pos; cost_dwell_pos2; ...
     cost_dwell_vel; cost_dwell_vel2; ...
     cost_dwell_acc; cost_dwell_acc2; ...
     cost_aero12; ...
     };%cost_Jmin1; cost_Jmin2};

hN = {cost_dist; cost_dist2};

%% ========================================================================
%%  6. DEFINIZIONE ED EXPORT DEL PROBLEMA MPC
%%  ========================================================================

fprintf('----------------------------\n         NMPC-export         \n----------------------------\n');
acadoSet('problemname', 'NMPC');

N_near  = 14;
N_far   = 30;
Ts_near = 1.0;
Ts_far  = 1.2;

T1 = N_near*Ts_near;
timepoints = [0:Ts_near:T1, T1 + Ts_far*(1:N_far)];

N = numel(timepoints) - 1;
T_hor = timepoints(end);

if N ~= 44 || abs(T_hor - 50.0) > 1e-12
    error('Griglia NMPC non coerente: N=%d, T_hor=%.6f s.', N, T_hor);
end

ocp = acado.OCP(timepoints);
W  = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));

ocp.minimizeLSQ(W, h);
ocp.minimizeLSQEndTerm(WN, hN);

%% ========================================================================
%%  7. VINCOLI DELL'OCP
%%  ========================================================================

%--------------------------------------------------------------------------
% 7.1 Accelerazione longitudinale
%--------------------------------------------------------------------------
ocp.subjectTo(acc  <= Ax_max);
ocp.subjectTo(acc  >= Ax_min);
ocp.subjectTo(acc2 <= Ax_max);
ocp.subjectTo(acc2 >= Ax_min);

% Vincoli slack jerk min
ocp.subjectTo(is(jerk  - jerk_min ) >= 0);
ocp.subjectTo(is(jerk2 - jerk_min ) >= 0);
% ocp.subjectTo(s_Jmin1 >= 0);
% ocp.subjectTo(s_Jmin2 >= 0);


% ocp.subjectTo(s_Jmin1 <= sJmin_max);
% ocp.subjectTo(s_Jmin2 <= sJmin_max);

ocp.subjectTo(jerk  <= jerk_max);
ocp.subjectTo(jerk2 <= jerk_max);

%--------------------------------------------------------------------------
% 7.2 Accelerazione laterale e velocita'
%--------------------------------------------------------------------------
ocp.subjectTo(is(vel  * vel  * k_road   - Ay_max) <= 0);
ocp.subjectTo(is(vel2 * vel2 * k_road_2 - Ay_max) <= 0);

ocp.subjectTo(is(vel  - Vmax)   <= 0);
ocp.subjectTo(is(vel2 - Vmax_2) <= 0);


ocp.subjectTo(vel  >= -v_eps);
ocp.subjectTo(vel2 >= -v_eps);

%--------------------------------------------------------------------------
% 7.3 Gap minimo tra veicoli
%--------------------------------------------------------------------------
ocp.subjectTo(gap12 >= gap_min);

%--------------------------------------------------------------------------
% 7.4 Vincoli fermate bus
%--------------------------------------------------------------------------
eps_stop_num = 0.0;

% Vincolo di approccio
ocp.subjectTo(is(pos  - ((s_stop_active   + eps_stop_num)*(1-x_stop_active)   + s_max*x_stop_active))   <= 0);
ocp.subjectTo(is(pos2 - ((s_stop_active_2 + eps_stop_num)*(1-x_stop_active_2) + s_max*x_stop_active_2)) <= 0);

% Dwell box
ocp.subjectTo(is(pos  - ((s_stop_active   + eps_stop_front)*x_dwell_active   + s_max*(1-x_dwell_active)))   <= 0);
ocp.subjectTo(is((s_stop_active   - eps_stop_back)*x_dwell_active   - s_max*(1-x_dwell_active)   - pos)  <= 0);

ocp.subjectTo(is(pos2 - ((s_stop_active_2 + eps_stop_front)*x_dwell_active_2 + s_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is((s_stop_active_2 - eps_stop_back)*x_dwell_active_2 - s_max*(1-x_dwell_active_2) - pos2) <= 0);

% Dwell velocity e acceleration
ocp.subjectTo(is(vel  - (eps_stop_v*x_dwell_active   + V_max*(1-x_dwell_active)))   <= 0);
ocp.subjectTo(is(acc  - (eps_stop_a*x_dwell_active   + Ax_max*(1-x_dwell_active)))  <= 0);
ocp.subjectTo(is(acc  + (eps_stop_a*x_dwell_active   + abs(Ax_min)*(1-x_dwell_active)))   >= 0);

ocp.subjectTo(is(vel2 - (eps_stop_v*x_dwell_active_2 + V_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is(acc2 - (eps_stop_a*x_dwell_active_2 + Ax_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is(acc2 + (eps_stop_a*x_dwell_active_2 + abs(Ax_min)*(1-x_dwell_active_2))) >= 0);

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

%% ========================================================================
%%  8. SETTAGGI ACADO, EXPORT E COMPILAZIONE
%%  ========================================================================

tic;
ocp.setModel(f);

mpc = acado.OCPexport(ocp);
mpc.set('HESSIAN_APPROXIMATION',       'GAUSS_NEWTON');
mpc.set('DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING');
mpc.set('SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set('INTEGRATOR_TYPE',             'INT_RK4');
mpc.set('NUM_INTEGRATOR_STEPS',        N);
mpc.set('QP_SOLVER',                   'QP_QPOASES3');
mpc.set('GENERATE_SIMULINK_INTERFACE', 'YES');
mpc.set('LEVENBERG_MARQUARDT',         1e-4);
mpc.set('MAX_NUM_QP_ITERATIONS',       1400);

if EXPORT
    mpc.exportCode('export_NMPC_R_1');
end

if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_NMPC_R_1/qpoases3');
    make_custom_solver_sfunction_R_1;
end

toc

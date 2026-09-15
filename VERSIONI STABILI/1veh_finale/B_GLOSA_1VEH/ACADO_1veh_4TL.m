%==========================================================================
% ACADO NMPC - B-GLOSA 1 veicolo - 4 TL attivi
%==========================================================================
% Script per generazione/export solver NMPC ACADO.
%
% Versione a 1 veicolo (ridotta dalla versione a 2 veicoli):
% - stati [pos, vel, acc];
% - controllo jerk;
% - OnlineData a 21 elementi per nodo;
% - vincoli semaforici, fermate, velocita', accelerazione e jerk
%   per il singolo veicolo.
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
DifferentialState pos vel acc;

% Ingresso di controllo: jerk longitudinale del veicolo
Control jerk;

% OnlineData per nodo: 21 dati.
% Contenuto:
%   4 flag TL testa, stop active, flag stop/dwell/target, k_road, Vmax,
%   schedule placeholders, 4 flag TL coda, 4 coordinate TL attive.
OnlineData x_TL1 x_TL2 x_TL3 x_TL4 ...
    s_stop_active x_stop_active x_dwell_active w_stop_active k_road Vmax dt_schd s_st s_hor ...
    x_TL1_tail x_TL2_tail x_TL3_tail x_TL4_tail ...
    s_TL1_active s_TL2_active s_TL3_active s_TL4_active;

%% ========================================================================
%  2. PARAMETRI PRINCIPALI DELLO SCENARIO E DEL MODELLO
%  ========================================================================

s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0]; 

s_max   = 5200;        % lunghezza path [m]
t_dwell = 7;          % dwell time [s]
V_max   = 50/3.6;      % limite statico per vincoli dwell [m/s]
Ax_min  = -1.5;        % limite inferiore accelerazione longitudinale [m/s^2]
Ax_max  =  1.5;        % limite superiore accelerazione longitudinale [m/s^2]
Ay_max  =  2.0;        % limite accelerazione laterale [m/s^2]
jerk_min = -0.5;       % limite inferiore jerk [m/s^3]
jerk_max =  0.5;       % limite superiore jerk [m/s^3]

L_platoon = 7.8;        % lunghezza veicolo / ingombro longitudinale [m]

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

f = [dot(pos); dot(vel); dot(acc)] == [pos_dot; vel_dot; acc_dot];

%% ========================================================================
%  5. TERMINI DELLA COST FUNCTION
%  ========================================================================

%--------------------------------------------------------------------------
% 5.1 Avanzamento, accelerazione longitudinale e jerk
%--------------------------------------------------------------------------

cost_dist  = is(s_max - pos)  / s_max;

cost_jerk  = is(jerk)  / jerk_max;

cost_Ax  = is(acc)  / Ax_max;

%--------------------------------------------------------------------------
% 5.2 Termini associati alle fermate bus
%--------------------------------------------------------------------------

cost_stop_target  = is(w_stop_active   * (pos  - s_stop_active)   / s_max);

cost_dwell_pos  = is(x_dwell_active   * (pos  - s_stop_active)   / eps_stop_front);

cost_dwell_vel  = is(x_dwell_active   * vel  / eps_stop_v);

cost_dwell_acc  = is(x_dwell_active   * acc  / eps_stop_a);

%--------------------------------------------------------------------------
% 5.3 Assemblaggio stage cost e terminal cost
%--------------------------------------------------------------------------
% Ordine dei pesi nel file init:
%  1   dist
%  2   jerk
%  3   Ax
%  4   stop target
%  5   dwell position
%  6   dwell velocity
%  7   dwell acceleration
%--------------------------------------------------------------------------

h = {cost_dist; ...
     cost_jerk; ...
     cost_Ax; ...
     cost_stop_target; ...
     cost_dwell_pos; ...
     cost_dwell_vel; ...
     cost_dwell_acc};

hN = {cost_dist};

%% ========================================================================
%  6. DEFINIZIONE ED EXPORT DEL PROBLEMA MPC
%  ========================================================================

fprintf('----------------------------\n         NMPC-export         \n----------------------------\n');
acadoSet('problemname', 'NMPC');

%% ========================================================================
%  GRIGLIA CHEBYSHEV-LOBATTO ATTENUATA
%  ========================================================================

N       = 35;       % numero di intervalli
T_hor   = 50.0;     % orizzonte totale [s]
Ts_ctrl = 1.0;      % periodo reale di aggiornamento NMPC [s]

[timepoints,dt_grid,beta_cheb] = ...
    NMPC_chebyshev_grid(N,T_hor,Ts_ctrl);

% Controlli di coerenza
if numel(timepoints) ~= N+1
    error('Numero di nodi errato.');
end

if abs(timepoints(1)) > 1e-12
    error('Il primo nodo deve essere zero.');
end

if abs(timepoints(end)-T_hor) > 1e-12
    error('Orizzonte temporale non corretto.');
end

if abs(dt_grid(1)-Ts_ctrl) > 1e-10
    error('Il primo intervallo non coincide con Ts_ctrl.');
end

fprintf('\nGriglia predittiva NMPC:\n');
fprintf('  N             = %d\n',N);
fprintf('  nodi          = %d\n',N+1);
fprintf('  T_hor         = %.3f s\n',T_hor);
fprintf('  beta Cheb.    = %.6f\n',beta_cheb);
fprintf('  dt iniziale   = %.6f s\n',dt_grid(1));
fprintf('  dt massimo    = %.6f s\n',max(dt_grid));
fprintf('  dt finale     = %.6f s\n\n',dt_grid(end));


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

ocp.subjectTo(jerk  <= jerk_max);
ocp.subjectTo(jerk  >= jerk_min);

ocp.subjectTo(is(vel  * vel  * k_road   - Ay_max) <= 0);

%--------------------------------------------------------------------------
% 7.2 Vincoli di velocita' stradale e non-negativita'
%--------------------------------------------------------------------------

ocp.subjectTo(is(vel  - Vmax)   <= 0);

v_eps = 0.02;
ocp.subjectTo(vel  >= -v_eps);

%--------------------------------------------------------------------------
% 7.3 Vincoli fermate bus
%--------------------------------------------------------------------------
eps_stop_num = 0.1;   % [m] tolleranza fermata: 10 cm
% Bus stop - vincolo di approccio
ocp.subjectTo(is(pos  - ((s_stop_active   + eps_stop_num)*(1-x_stop_active)   + s_max*x_stop_active))   <= 0);

% Bus stop - dwell box
ocp.subjectTo(is(pos  - ((s_stop_active   + eps_stop_front)*x_dwell_active   + s_max*(1-x_dwell_active)))   <= 0);
ocp.subjectTo(is((s_stop_active   - eps_stop_back)*x_dwell_active   - s_max*(1-x_dwell_active)   - pos)  <= 0);

% Bus stop - dwell velocity and acceleration
ocp.subjectTo(is(vel  - (eps_stop_v*x_dwell_active   + V_max*(1-x_dwell_active)))   <= 0);
ocp.subjectTo(is(acc  - (eps_stop_a*x_dwell_active   + Ax_max*(1-x_dwell_active)))  <= 0);
ocp.subjectTo(is(acc  + (eps_stop_a*x_dwell_active   + abs(Ax_min)*(1-x_dwell_active)))   >= 0);

%--------------------------------------------------------------------------
% 7.4 Vincoli semaforici con 4 slot mobili
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
mpc.set('NUM_INTEGRATOR_STEPS',         N);
mpc.set('QP_SOLVER',                   'QP_QPOASES3');
mpc.set('GENERATE_SIMULINK_INTERFACE', 'YES');
mpc.set('LEVENBERG_MARQUARDT',         1e-4);

if EXPORT
    mpc.exportCode('export_NMPC_R_1');
end

if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_NMPC_R_1/qpoases3');
    make_custom_solver_sfunction_R_1;
end

toc

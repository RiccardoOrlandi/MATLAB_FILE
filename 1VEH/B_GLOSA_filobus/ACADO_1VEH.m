%==========================================================================
% ACADO NMPC - B-GLOSA 1 veicolo rigido lungo L_platoon - 4 TL attivi
%==========================================================================
% Script per generazione/export solver NMPC ACADO.
%
% - un solo veicolo modellato come corpo rigido di lunghezza
%   L_platoon, con vincoli semaforici applicati sia alla testa sia alla coda;
% - OnlineData a 21 parametri per nodo:
%       4 flag TL testa,
%       1 fermata attiva,
%       k_road precomputata,
%       Vmax,
%       placeholders schedule,
%       4 flag TL coda,
%       4 coordinate fisiche dei TL attivi;
% - No le slack su velocita' e accelerazione laterale, coerentemente
%   con la versione 3-vehicle alleggerita;
% - mantenere una sola fermata attiva alla volta.
%==========================================================================

clc;
clear all;
close all;

%% ========================================================================
%  1. DEFINIZIONE DEL PROBLEMA DI CONTROLLO
%  ========================================================================
Ts = 1;
EXPORT  = 1;
COMPILE = 1;

% Stati del modello predittivo:
%   pos : coordinata longitudinale della testa del veicolo/platoon rigido [m]
%   vel : velocita' longitudinale [m/s]
%   acc : accelerazione longitudinale [m/s^2]
DifferentialState pos vel acc;

% Ingresso di controllo: jerk longitudinale.
Control jerk;

% OnlineData per nodo: 21 parametri.
%   1  - 4    x_TL_head, slot TL mobili 1...4
%   5         s_stop_active
%   6         x_stop_active
%   7         x_dwell_active
%   8         w_stop_active
%   9         k_road
%   10        Vmax
%   11        dt_schd       placeholder compatibilita'
%   12        s_st          placeholder compatibilita'
%   13        s_hor         placeholder compatibilita'
%   14 - 17   x_TL_tail, slot TL mobili 1...4
%   18 - 21   s_TL_active, coordinate fisiche degli slot TL mobili
OnlineData x_TL1 x_TL2 x_TL3 x_TL4 ...
    s_stop_active x_stop_active x_dwell_active w_stop_active ...
    k_road Vmax dt_schd s_st s_hor ...
    x_TL1_tail x_TL2_tail x_TL3_tail x_TL4_tail ...
    s_TL1_active s_TL2_active s_TL3_active s_TL4_active;

%% ========================================================================
%  2. PARAMETRI PRINCIPALI DELLO SCENARIO E DEL MODELLO
%  ========================================================================

s_max = 5200;          % lunghezza path [m]
t_dwell = 10;         % dwell time nominale [s], gestito online
V_max = 50/3.6;       % limite statico per vincoli dwell [m/s]

Ax_min = -1.5;        % limite inferiore accelerazione longitudinale [m/s^2]
Ax_max =  1.5;        % limite superiore accelerazione longitudinale [m/s^2]
Ay_max =  2.0;        % limite accelerazione laterale [m/s^2]

jerk_min = -0.5;      % limite inferiore jerk [m/s^3]
jerk_max =  0.5;      % limite superiore jerk [m/s^3]

% Lunghezza rigida equivalente usata per i vincoli sulla coda.
L_platoon = 18;       % ingombro longitudinale rigido equivalente [m]

%% ========================================================================
%  3. PARAMETRI FERMATE BUS
%  ========================================================================

% x_stop_active  = 0 -> vincolo di approccio attivo: pos <= s_stop_active
% x_stop_active  = 1 -> vincolo di approccio rilasciato
% x_dwell_active = 1 -> dwell attivo: stop-box attiva
% x_dwell_active = 0 -> dwell non attivo
% w_stop_active  = 1 -> costo di attrazione verso fermata attivo
% w_stop_active  = 0 -> costo di attrazione spento

eps_stop_back  = 0.5;  % tolleranza dietro fermata [m]
eps_stop_front = 0.5;  % tolleranza davanti fermata [m]
eps_stop_v     = 0.1;  % velocita' massima durante dwell [m/s]
eps_stop_a     = 0.3;  % accelerazione ammessa durante dwell [m/s^2]

%% ========================================================================
%  4. EQUAZIONI DI STATO
%  ========================================================================

pos_dot = is(vel);
vel_dot = is(acc);
acc_dot = is(jerk);

f = [dot(pos); dot(vel); dot(acc)] == [pos_dot; vel_dot; acc_dot];

%% ========================================================================
%  5. TERMINI DELLA COST FUNCTION
%  ========================================================================

%--------------------------------------------------------------------------
% 5.1 Avanzamento, jerk e accelerazione longitudinale
%--------------------------------------------------------------------------

cost_dist = is(s_max - pos) / s_max;
cost_jerk = is(jerk) / jerk_max;
cost_Ax   = is(acc)  / Ax_max;

%--------------------------------------------------------------------------
% 5.2 Termini associati alla fermata attiva
%--------------------------------------------------------------------------
% La formulazione e' coerente con la versione 3-vehicle: viene passato al
% solver un solo target di fermata alla volta, riducendo il numero di
% OnlineData e di vincoli rispetto alla vecchia formulazione esplicita.
%--------------------------------------------------------------------------

cost_stop_target = is(w_stop_active * (pos - s_stop_active) / s_max);
cost_dwell_pos   = is(x_dwell_active * (pos - s_stop_active) / eps_stop_front);
cost_dwell_vel   = is(x_dwell_active * vel / eps_stop_v);
cost_dwell_acc   = is(x_dwell_active * acc / eps_stop_a);

%--------------------------------------------------------------------------
% 5.3 Assemblaggio stage cost e terminal cost
%--------------------------------------------------------------------------
% Ordine dei pesi nel file NMPC_init_1veh_K4_Lplatoon_aligned.m:
%  1  cost_dist          avanzamento lungo il percorso
%  2  cost_jerk          jerk longitudinale
%  3  cost_Ax            accelerazione longitudinale
%  4  cost_stop_target   attrazione verso fermata attiva
%  5  cost_dwell_pos     mantenimento posizione durante dwell
%  6  cost_dwell_vel     velocita' nulla durante dwell
%  7  cost_dwell_acc     accelerazione nulla durante dwell
%--------------------------------------------------------------------------

h  = {cost_dist; cost_jerk; cost_Ax; ...
      cost_stop_target; cost_dwell_pos; cost_dwell_vel; cost_dwell_acc};
hN = {cost_dist};

%% ========================================================================
%  6. DEFINIZIONE ED EXPORT DEL PROBLEMA MPC
%  ========================================================================

fprintf('----------------------------\n         NMPC-export         \n----------------------------\n');
acadoSet('problemname', 'NMPC');

N = 50;
ocp = acado.OCP(0.0, N*Ts, N);

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

ocp.subjectTo(acc <= Ax_max);
ocp.subjectTo(acc >= Ax_min);

ocp.subjectTo(jerk <= jerk_max);
ocp.subjectTo(jerk >= jerk_min);

% k_road e' fornita da lookup/precomputazione esterna e passata come OnlineData.
ocp.subjectTo(is(vel * vel * k_road - Ay_max) <= 0);

%--------------------------------------------------------------------------
% 7.2 Vincoli di velocita' stradale e non-negativita'
%--------------------------------------------------------------------------

ocp.subjectTo(is(vel - Vmax) <= 0);

v_eps = 0.01;
ocp.subjectTo(vel >= -v_eps);

%--------------------------------------------------------------------------
% 7.3 Vincoli fermata bus: approccio, dwell box, velocita'/accelerazione dwell
%--------------------------------------------------------------------------

% Vincolo di approccio alla fermata attiva.
ocp.subjectTo(is(pos - (s_stop_active*(1-x_stop_active) + s_max*x_stop_active)) <= 0);

% Stop-box durante dwell.
ocp.subjectTo(is(pos - ((s_stop_active + eps_stop_front)*x_dwell_active + s_max*(1-x_dwell_active))) <= 0);
ocp.subjectTo(is((s_stop_active - eps_stop_back)*x_dwell_active - s_max*(1-x_dwell_active) - pos) <= 0);

% Vincoli dinamici durante dwell.
ocp.subjectTo(is(vel - (eps_stop_v*x_dwell_active + V_max*(1-x_dwell_active))) <= 0);
ocp.subjectTo(is(acc - (eps_stop_a*x_dwell_active + Ax_max*(1-x_dwell_active))) <= 0);
ocp.subjectTo(is(acc + (eps_stop_a*x_dwell_active + abs(Ax_min)*(1-x_dwell_active))) >= 0);

%--------------------------------------------------------------------------
% 7.4 Vincoli semaforici con 4 slot mobili
%--------------------------------------------------------------------------
% x_TL = 1 -> verde / vincolo rilasciato.
% x_TL = 0 -> rosso / vincolo attivo.
% Le coordinate s_TL*_active sono aggiornate da OnlineDataArray e
% rappresentano i primi 4 semafori ancora rilevanti rispetto alla coda.
%--------------------------------------------------------------------------

d_safe_TL = 0;

% Testa del veicolo/platoon rigido.
ocp.subjectTo(is(pos - ((s_TL1_active - d_safe_TL)*(1-x_TL1) + s_max*x_TL1)) <= 0);
ocp.subjectTo(is(pos - ((s_TL2_active - d_safe_TL)*(1-x_TL2) + s_max*x_TL2)) <= 0);
ocp.subjectTo(is(pos - ((s_TL3_active - d_safe_TL)*(1-x_TL3) + s_max*x_TL3)) <= 0);
ocp.subjectTo(is(pos - ((s_TL4_active - d_safe_TL)*(1-x_TL4) + s_max*x_TL4)) <= 0);

% Coda del veicolo/platoon rigido.
ocp.subjectTo(is((pos - L_platoon) - ((s_TL1_active - d_safe_TL)*(1-x_TL1_tail) + s_max*x_TL1_tail)) <= 0);
ocp.subjectTo(is((pos - L_platoon) - ((s_TL2_active - d_safe_TL)*(1-x_TL2_tail) + s_max*x_TL2_tail)) <= 0);
ocp.subjectTo(is((pos - L_platoon) - ((s_TL3_active - d_safe_TL)*(1-x_TL3_tail) + s_max*x_TL3_tail)) <= 0);
ocp.subjectTo(is((pos - L_platoon) - ((s_TL4_active - d_safe_TL)*(1-x_TL4_tail) + s_max*x_TL4_tail)) <= 0);

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
mpc.set('NUM_INTEGRATOR_STEPS',        2);
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

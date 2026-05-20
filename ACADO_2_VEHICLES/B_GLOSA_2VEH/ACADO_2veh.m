clc;
clear all;
close all;
%% Control problem definition
Ts = 1;   
EXPORT = 1;
COMPILE= 1;

%Define your states
DifferentialState pos vel acc pos2 vel2 acc2; 

%Define your control inputs
Control jerk jerk2;
%Extra inputs needed
OnlineData x_TL1 x_TL2 x_TL3 x_TL4 x_TL5 x_TL6 x_TL7 x_TL8 x_TL9 x_TL10 x_TL11 s_stop_active x_stop_active x_dwell_active w_stop_active k_road Vmax dt_schd s_st s_hor x_TL1_tail x_TL2_tail x_TL3_tail x_TL4_tail x_TL5_tail x_TL6_tail x_TL7_tail x_TL8_tail x_TL9_tail x_TL10_tail x_TL11_tail ...
x_TL1_2 x_TL2_2 x_TL3_2 x_TL4_2 x_TL5_2 x_TL6_2 x_TL7_2 x_TL8_2 x_TL9_2 x_TL10_2 x_TL11_2 s_stop_active_2 x_stop_active_2 x_dwell_active_2 w_stop_active_2 k_road_2 Vmax_2 dt_schd_2 s_st_2 s_hor_2 x_TL1_tail_2 x_TL2_tail_2 x_TL3_tail_2 x_TL4_tail_2 x_TL5_tail_2 x_TL6_tail_2 x_TL7_tail_2 x_TL8_tail_2 x_TL9_tail_2 x_TL10_tail_2 x_TL11_tail_2;
%=====================================
%  PARAMETRI PRINCIPALI:
%=====================================
s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0];% TL coordinate [m]

s_max = 5200;           % path lenth
t_dwell = 10;           % dwelling time [s]
V_max = 50/3.6;         % limite statico diverso invece da Vmax
Ax_min = -1.5;          % lower limit long. acc. [m/s^2]
Ax_max = 1.5;           % upper limit long. acc. [m/s^2]
Ay_max = 2;             % upper limit lat. acc.  [m/s^2]
jerk_min = -0.5;        % lower limit long. jerk [m/s^3]
jerk_max = 0.5;         % upper limit long. jerk [m/s^3]
L_platoon=10;   %lunghezza platoon rigido [m]

%Ax_max_mot = 1.15; %massima accelerazione motre [m/s^2]

%=====================================
% PARAMETRI FERMATE BUS
%=====================================
% x_stop_i  = 0 -> vincolo di approccio attivo: pos <= s_stop(i)
% x_stop_i  = 1 -> vincolo di approccio rilasciato
% x_dwell_i = 1 -> dwell attivo: il bus deve restare nella stop-box
% x_dwell_i = 0 -> dwell non attivo
% w_stop_i  = 1 -> costo di attrazione verso la fermata attivo
% w_stop_i  = 0 -> costo di attrazione spento
%======================================
eps_stop_back  = 0.5;   % tolleranza dietro fermata [m]
eps_stop_front = 0.5;   % tolleranza davanti fermata [m]
eps_stop_v     = 0.1;  % velocita massima durante dwell [m/s]
eps_stop_a     = 0.3;  % accelerazione ammessa durante dwell [m/s^2]
epsilon = 1e-5; 
alpha = 100; 
min_v = is((0.5 + vel)/2 + sqrt((0.5 - vel)^2 + epsilon)/2);

%=================
% STATE EQUATIONS
%=================
pos_dot = is(vel);
vel_dot = is(acc);
acc_dot = is(jerk);
pos_dot2 = is(vel2);
vel_dot2 = is(acc2);
acc_dot2 = is(jerk2);
f = [dot(pos);dot(vel);dot(acc);dot(pos2);dot(vel2);dot(acc2)]==[pos_dot;vel_dot;acc_dot;pos_dot2;vel_dot2;acc_dot2];

%======================================
%% TERMINI DELLA COST FUNCTION
%======================================
cost_dist = is(s_max - pos)/s_max; 
cost_Ax = is(acc)/Ax_max;  
cost_jerk = is(jerk)/jerk_max;

cost_dist2 = is(s_max - pos2)/s_max; 
cost_Ax2 = is(acc2)/Ax_max;  
cost_jerk2 = is(jerk2)/jerk_max;

%====================================
%cost_Jmax = is(s_Jmax)/jerk_max; %7.jerk slack max
%cost_Jmin = is(s_Jmin)/abs(jerk_min);%8.jerk slack min
%=====================================
cost_stop_target = is(w_stop_active * (pos - s_stop_active) / s_max);
cost_dwell_pos = is(x_dwell_active * (pos - s_stop_active) / eps_stop_front);
cost_dwell_vel = is(x_dwell_active * vel / eps_stop_v);
cost_dwell_acc = is(x_dwell_active * acc / eps_stop_a);

cost_stop_target2 = is(w_stop_active_2 * (pos2 - s_stop_active_2) / s_max);
cost_dwell_pos2 = is(x_dwell_active_2 * (pos2 - s_stop_active_2) / eps_stop_front);
cost_dwell_vel2 = is(x_dwell_active_2 * vel2 / eps_stop_v);
cost_dwell_acc2 = is(x_dwell_active_2 * acc2 / eps_stop_a);
%======================================
% AERODYNAMIC / PLATOON GAP COST
%======================================
d_gap     = 5;   % distanza desiderata [m]
gap_on    = 20;   % distanza oltre cui il termine satura [m]
sigma_gap = 5;    % morbidezza della transizione [m]

% gap bumper-to-bumper
gap = is((pos - L_platoon) - pos2);
aero_activation = is(0.5 - atan((gap - gap_on)/sigma_gap)/pi);
cost_aero = is(aero_activation * (gap - d_gap)/d_gap);
%==========================================
% COST FUNCTION
%==========================================
h = {cost_dist; cost_dist2; cost_jerk; cost_jerk2; cost_Ax; cost_Ax2; cost_stop_target; cost_stop_target2; cost_dwell_pos; cost_dwell_pos2 ; cost_dwell_vel; cost_dwell_vel2 ;cost_dwell_acc; cost_dwell_acc2;cost_aero};
hN = {cost_dist;cost_dist2};

% h = [diffStates; controls];
% hN = [diffStates];

%% MPCexport
fprintf('----------------------------\n         NMPC-export         \n----------------------------\n');
acadoSet('problemname', 'NMPC');

N = 50; %Steps
ocp = acado.OCP( 0.0, N*Ts, N );

%These two variables are just for the compilation. The ones implemented in
%the Simulink Model are used in the simulation
W = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));

ocp.minimizeLSQ(W, h);
ocp.minimizeLSQEndTerm(WN, hN);
%=========================================
%% Constraints
%=========================================

% ============================================================
% Comfort constraints: longitudinal acceleration, jerk, lateral acceleration
% ============================================================

ocp.subjectTo(acc <= Ax_max);
ocp.subjectTo(acc >= Ax_min);
ocp.subjectTo(acc2 <= Ax_max);
ocp.subjectTo(acc2 >= Ax_min);

ocp.subjectTo(jerk <= jerk_max);
ocp.subjectTo(jerk >= jerk_min);
ocp.subjectTo(jerk2 <= jerk_max);
ocp.subjectTo(jerk2 >= jerk_min);

ocp.subjectTo(is(vel*vel*k_road - Ay_max) <= 0);
ocp.subjectTo(is(vel2*vel2*k_road_2 - Ay_max) <= 0);

% ============================================================
% Road speed constraint
% ============================================================

ocp.subjectTo(is(vel - Vmax) <= 0);
ocp.subjectTo(vel >= 0);
ocp.subjectTo(is(vel2 - Vmax_2) <= 0);
ocp.subjectTo(vel2 >= 0);

ocp.subjectTo(gap>=2);

% ============================================================
% Bus stop constraints
% ============================================================

% Bus stop - vincolo di approccio
ocp.subjectTo(is(pos - (s_stop_active*(1-x_stop_active) + s_max*x_stop_active)) <= 0);
ocp.subjectTo(is(pos2 - (s_stop_active_2*(1-x_stop_active_2) + s_max*x_stop_active_2)) <= 0);

% Bus stop - dwell box
ocp.subjectTo(is(pos - ((s_stop_active+eps_stop_front)*x_dwell_active + s_max*(1-x_dwell_active))) <= 0);
ocp.subjectTo(is((s_stop_active-eps_stop_back)*x_dwell_active - s_max*(1-x_dwell_active) - pos) <= 0);
ocp.subjectTo(is(pos2 - ((s_stop_active_2+eps_stop_front)*x_dwell_active_2 + s_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is((s_stop_active_2-eps_stop_back)*x_dwell_active_2 - s_max*(1-x_dwell_active_2) - pos2) <= 0);

% Bus stop - dwell velocity and acceleration
ocp.subjectTo(is(vel - (eps_stop_v*x_dwell_active + V_max*(1-x_dwell_active))) <= 0);
ocp.subjectTo(is(acc - (eps_stop_a*x_dwell_active + Ax_max*(1-x_dwell_active))) <= 0);
ocp.subjectTo(is(acc + (eps_stop_a*x_dwell_active + abs(Ax_min)*(1-x_dwell_active))) >= 0);
ocp.subjectTo(is(vel2 - (eps_stop_v*x_dwell_active_2 + V_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is(acc2 - (eps_stop_a*x_dwell_active_2 + Ax_max*(1-x_dwell_active_2))) <= 0);
ocp.subjectTo(is(acc2 + (eps_stop_a*x_dwell_active_2 + abs(Ax_min)*(1-x_dwell_active_2))) >= 0);

% Traffic light - head constraint
d_safe_TL = 0;

ocp.subjectTo(is(pos - ((s_TL(1)-d_safe_TL)*(1-x_TL1)  + s_max*x_TL1 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(2)-d_safe_TL)*(1-x_TL2)  + s_max*x_TL2 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(3)-d_safe_TL)*(1-x_TL3)  + s_max*x_TL3 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(4)-d_safe_TL)*(1-x_TL4)  + s_max*x_TL4 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(5)-d_safe_TL)*(1-x_TL5)  + s_max*x_TL5 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(6)-d_safe_TL)*(1-x_TL6)  + s_max*x_TL6 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(7)-d_safe_TL)*(1-x_TL7)  + s_max*x_TL7 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(8)-d_safe_TL)*(1-x_TL8)  + s_max*x_TL8 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(9)-d_safe_TL)*(1-x_TL9)  + s_max*x_TL9 )) <= 0);
ocp.subjectTo(is(pos - ((s_TL(10)-d_safe_TL)*(1-x_TL10) + s_max*x_TL10)) <= 0);
ocp.subjectTo(is(pos - ((s_TL(11)-d_safe_TL)*(1-x_TL11) + s_max*x_TL11)) <= 0);

ocp.subjectTo(is(pos2 - ((s_TL(1)-d_safe_TL)*(1-x_TL1_2)  + s_max*x_TL1_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(2)-d_safe_TL)*(1-x_TL2_2)  + s_max*x_TL2_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(3)-d_safe_TL)*(1-x_TL3_2)  + s_max*x_TL3_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(4)-d_safe_TL)*(1-x_TL4_2)  + s_max*x_TL4_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(5)-d_safe_TL)*(1-x_TL5_2)  + s_max*x_TL5_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(6)-d_safe_TL)*(1-x_TL6_2)  + s_max*x_TL6_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(7)-d_safe_TL)*(1-x_TL7_2)  + s_max*x_TL7_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(8)-d_safe_TL)*(1-x_TL8_2)  + s_max*x_TL8_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(9)-d_safe_TL)*(1-x_TL9_2)  + s_max*x_TL9_2 )) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(10)-d_safe_TL)*(1-x_TL10_2) + s_max*x_TL10_2)) <= 0);
ocp.subjectTo(is(pos2 - ((s_TL(11)-d_safe_TL)*(1-x_TL11_2) + s_max*x_TL11_2)) <= 0);

% Traffic light - tail constraint
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(1)-d_safe_TL)*(1-x_TL1_tail)  + s_max*x_TL1_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(2)-d_safe_TL)*(1-x_TL2_tail)  + s_max*x_TL2_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(3)-d_safe_TL)*(1-x_TL3_tail)  + s_max*x_TL3_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(4)-d_safe_TL)*(1-x_TL4_tail)  + s_max*x_TL4_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(5)-d_safe_TL)*(1-x_TL5_tail)  + s_max*x_TL5_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(6)-d_safe_TL)*(1-x_TL6_tail)  + s_max*x_TL6_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(7)-d_safe_TL)*(1-x_TL7_tail)  + s_max*x_TL7_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(8)-d_safe_TL)*(1-x_TL8_tail)  + s_max*x_TL8_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(9)-d_safe_TL)*(1-x_TL9_tail)  + s_max*x_TL9_tail )) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(10)-d_safe_TL)*(1-x_TL10_tail) + s_max*x_TL10_tail)) <= 0);
ocp.subjectTo(is((pos-L_platoon) - ((s_TL(11)-d_safe_TL)*(1-x_TL11_tail) + s_max*x_TL11_tail)) <= 0);

ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(1)-d_safe_TL)*(1-x_TL1_tail_2)  + s_max*x_TL1_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(2)-d_safe_TL)*(1-x_TL2_tail_2)  + s_max*x_TL2_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(3)-d_safe_TL)*(1-x_TL3_tail_2)  + s_max*x_TL3_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(4)-d_safe_TL)*(1-x_TL4_tail_2)  + s_max*x_TL4_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(5)-d_safe_TL)*(1-x_TL5_tail_2)  + s_max*x_TL5_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(6)-d_safe_TL)*(1-x_TL6_tail_2)  + s_max*x_TL6_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(7)-d_safe_TL)*(1-x_TL7_tail_2)  + s_max*x_TL7_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(8)-d_safe_TL)*(1-x_TL8_tail_2)  + s_max*x_TL8_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(9)-d_safe_TL)*(1-x_TL9_tail_2)  + s_max*x_TL9_tail_2 )) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(10)-d_safe_TL)*(1-x_TL10_tail_2) + s_max*x_TL10_tail_2)) <= 0);
ocp.subjectTo(is((pos2-L_platoon) - ((s_TL(11)-d_safe_TL)*(1-x_TL11_tail_2) + s_max*x_TL11_tail_2)) <= 0);


%% Other settings
tic;
ocp.setModel( f );

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',      'INT_RK4' );
mpc.set( 'NUM_INTEGRATOR_STEPS', N );
% mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
% mpc.set( 'NUM_INTEGRATOR_STEPS',        1e1*Ts*N            ); 
mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );
mpc.set( 'LEVENBERG_MARQUARDT',         1e-4                );

if EXPORT
    mpc.exportCode( 'export_NMPC_R_1' );
end
if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_NMPC_R_1/qpoases3')
    
    make_custom_solver_sfunction_R_1
    
end
toc
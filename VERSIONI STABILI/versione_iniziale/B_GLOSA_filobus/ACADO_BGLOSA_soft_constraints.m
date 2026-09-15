clc;
clear all;
close all;
%% Control problem definition
Ts = 1;   
EXPORT = 1;
COMPILE= 1;

%Define your states
DifferentialState pos vel acc; 

%Define your control inputs
Control jerk s_Vmax s_Ay s_Jmin s_Jmax;% s_Vmin;% s_Amin s_Amax s_Ay s_Jmax s_Jmin;  

%Extra inputs needed
OnlineData x_TL1 x_TL2 x_TL3 x_TL4 x_TL5 x_TL6 x_TL7 x_TL8 x_TL9 x_TL10 x_TL11 x_stop1 x_stop2 x_stop3 x_stop4 x_stop5 x_stop6 x_stop7 k_road Vmin Vmax Ax_lim_mot dt_schd s_st s_hor;

%  DEFINE THE PARAMETERS:
s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0];    % TL coordinate [m]
s_stop = [80, 447, 756, 1084, 1304, 1507, 1822];                                % stop coordinate [m]
s_max = 5200;           % path lenth
t_dwell = 10;           % dwelling time [s]
Ax_min = -1.5;          % lower limit long. acc. [m/s^2]
Ax_max = 1.5;           % upper limit long. acc. [m/s^2]
Ay_max = 2;             % upper limit lat. acc.  [m/s^2]
jerk_min = -0.5;        % lower limit long. jerk [m/s^3]
jerk_max = 0.5;         % upper limit long. jerk [m/s^3]
a = 38.62;
b = -0.849;
c = -0.465;
Ax_max_mot = 1.15;

% STATE EQUATIONS
pos_dot = is(vel);
vel_dot = is(acc);
acc_dot = is(jerk);

f = [dot(pos);dot(vel);dot(acc)]==[pos_dot;vel_dot;acc_dot];

%% COMPUTING TERMS TO DEFINE COST FUNCTION
V_max = 50/3.6;
cost_dist = is(s_max - pos)/s_max;
cost_Ax = is(acc)/Ax_max;
cost_jerk = is(jerk)/jerk_max;

epsilon = 1e-5; alpha = 100; 
min_v = is((0.5 + vel)/2 + sqrt((0.5 - vel)^2 + epsilon)/2);
cost_schd = is((1/pi*atan(alpha*(pos-(s_st-s_hor))) + 0.5) * (dt_schd - (s_st-pos)/min_v)/180);

cost_vmax = is(s_Vmax)/V_max;
cost_Ay = is(s_Ay)/Ay_max;
cost_Jmax = is(s_Jmax)/jerk_max;
cost_Jmin = is(s_Jmin)/jerk_min;

h = {cost_dist; cost_jerk; cost_Ax; cost_schd; cost_vmax; cost_Ay; cost_Jmax; cost_Jmin};   % s_Amin; s_Amax; s_Ay; s_Jmin; s_Jmax};
hN = {cost_dist};
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

%% Constraints
% Comfort
ocp.subjectTo(acc <= Ax_max);
ocp.subjectTo(acc >= Ax_min);
ocp.subjectTo(is(jerk - jerk_max - s_Jmax) <= 0);
ocp.subjectTo(s_Jmax >= 0);
ocp.subjectTo(is(jerk - jerk_min - s_Jmin) >= 0);
ocp.subjectTo(s_Jmin >= 0);

ocp.subjectTo(is(vel^2/(1/k_road) - Ay_max - s_Ay) <= 0);
ocp.subjectTo(s_Ay >= 0);
% Motor
% ocp.subjectTo(acc <= Ax_max_mot);
ocp.subjectTo(is(acc - Ax_lim_mot) <= 0);
% Road
ocp.subjectTo(is(vel - Vmax - s_Vmax) <= 0);
ocp.subjectTo(s_Vmax >= 0);
ocp.subjectTo(vel >= 0);
% Traffic light
ocp.subjectTo(is(pos - (s_TL(1)*(1-x_TL1) + s_max*x_TL1)) <= 0);
ocp.subjectTo(is(pos - (s_TL(2)*(1-x_TL2) + s_max*x_TL2)) <= 0);
ocp.subjectTo(is(pos - (s_TL(3)*(1-x_TL3) + s_max*x_TL3)) <= 0);
ocp.subjectTo(is(pos - (s_TL(4)*(1-x_TL4) + s_max*x_TL4)) <= 0);
ocp.subjectTo(is(pos - (s_TL(5)*(1-x_TL5) + s_max*x_TL5)) <= 0);
ocp.subjectTo(is(pos - (s_TL(6)*(1-x_TL6) + s_max*x_TL6)) <= 0);
ocp.subjectTo(is(pos - (s_TL(7)*(1-x_TL7) + s_max*x_TL7)) <= 0);
ocp.subjectTo(is(pos - (s_TL(8)*(1-x_TL8) + s_max*x_TL8)) <= 0);
ocp.subjectTo(is(pos - (s_TL(9)*(1-x_TL9) + s_max*x_TL9)) <= 0);
ocp.subjectTo(is(pos - (s_TL(10)*(1-x_TL10) + s_max*x_TL10)) <= 0);
ocp.subjectTo(is(pos - (s_TL(11)*(1-x_TL11) + s_max*x_TL11)) <= 0);

% Bus stop
ocp.subjectTo(is(pos - (s_stop(1)*(1-x_stop1) + s_max*x_stop1)) <= 0);
ocp.subjectTo(is(pos - (s_stop(2)*(1-x_stop2) + s_max*x_stop2)) <= 0);
ocp.subjectTo(is(pos - (s_stop(3)*(1-x_stop3) + s_max*x_stop3)) <= 0);
ocp.subjectTo(is(pos - (s_stop(4)*(1-x_stop4) + s_max*x_stop4)) <= 0);
ocp.subjectTo(is(pos - (s_stop(5)*(1-x_stop5) + s_max*x_stop5)) <= 0);
ocp.subjectTo(is(pos - (s_stop(6)*(1-x_stop6) + s_max*x_stop6)) <= 0);
ocp.subjectTo(is(pos - (s_stop(7)*(1-x_stop7) + s_max*x_stop7)) <= 0);


%% Other settings
tic;
ocp.setModel( f );

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        1e1*Ts*N            ); 
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
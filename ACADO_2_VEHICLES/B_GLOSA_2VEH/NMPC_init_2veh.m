%==========================================
% script da eseguire una volta runnato ACADO_BGLOSA_soft_constraints_platoon_stop_dwell.m
% una volta eseguito puoi avviare la simulazione su Simulink
%==========================================

%% Soft Constraints / Cost Function Weights

% Ordine dei termini:
% 1  cost_dist          avanzamento lungo il percorso
% 2  cost_dist2         avanzamento lungo il percorso 2veh
% 3  cost_jerk          jerk
% 4  cost_jerk2         jerk2
% 5  cost_Ax            accelerazione longitudinale
% 6  cost_Ax2           accelerazione longitudinale 2veh
% 7  cost_stop_target   attrazione verso fermata attiva
% 8  cost_stop_target   attrazione verso fermata attiva 2 veh
% 9  cost_dwell_pos     mantenimento posizione durante dwell
% 10 cost_dwell_pos     mantenimento posizione durante dwell 2 veh
% 11 cost_dwell_vel     velocità nulla durante dwell
% 12 cost_dwell_vel     velocità nulla durante dwell 2 veh
% 13 cost_dwell_acc     accelerazione nulla durante dwell
% 14 cost_dwell_acc     accelerazione nulla durante dwell 2 veh
% 15 cost_aero          costo aerodinamicità

Qx1 = [10, 10, 2, 2, 0.5, 0.5, 50, 50, 200, 200, 500, 500, 100, 100, 1];
W1 = diag(Qx1);
NMPC_Wmat1 = [W1(1,:) W1(2,:) W1(3,:) W1(4,:) W1(5,:) W1(6,:) W1(7,:) W1(8,:) W1(9,:) W1(10,:) W1(11,:) W1(12,:) W1(13,:) W1(14,:) W1(15,:)];

% Terminal cost
QN1 = [10 10];
WN1 = diag(QN1);
NMPC_WNmat1 = reshape(WN1.',1,[]);
N_Jterms = length(Qx1);


t_dwell = 10;
t_offset = 0;
%%
zInit = [];   
bValues = [];

N_iter = 3;
Ts = 1;
N = 50;
Vmax = 50;
Vmax_2 = 50;
xInit = [100 2 0 0 2 0];
uInit = [0 0];

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
%% Road curvature lookup table

load('k_road_PL_jrl_chs_precomputed.mat','s_kroad_map','k_road_map','ds','s_start')
s_kroad_map = s_kroad_map(:);
k_road_map  = k_road_map(:);

addpath('..\chs_gen\')
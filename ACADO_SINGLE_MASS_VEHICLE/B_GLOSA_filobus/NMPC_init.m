clear all; close all; clc;

%% Soft Constraints
Qx1= [10 2 2.5 40 10000 10000 10000 10000];
W1 = diag(Qx1); 
NMPC_Wmat1=[W1(1,:) W1(2,:) W1(3,:) W1(4,:) W1(5,:) W1(6,:) W1(7,:) W1(8,:)];
QN1=[1];
WN1=diag(QN1);
NMPC_WNmat1=[WN1(1,:)];

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
xInit = [0 2 0];
uInit = [0 0 0];

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

addpath('..\chs_gen\')

%%
s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, 1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0];
s_stop = [80, 447, 756, 1084, 1304, 1507, 1822]; 

i_fig = 10;

h = plot_covDist(out,i_fig,s_TL,s_stop);
N = 35;
T_hor = 50;
Ts_ctrl = 1;

[timepoints,dt_grid,beta_cheb] = NMPC_chebyshev_grid(N,T_hor,Ts_ctrl);

figure;
plot(timepoints,zeros(size(timepoints)),'o','LineWidth',1.5,'MarkerSize',7);
hold on;
for i = 1:length(timepoints)
    text(timepoints(i),0.02,num2str(i-1), ...
        'HorizontalAlignment','center','FontSize',8);
end
grid on;
xlabel('Tempo [s]');
yticks([]);
title(sprintf('Distribuzione dei nodi con indice - N = %d',N));
xlim([0 T_hor]);
ylim([-0.05 0.08]);
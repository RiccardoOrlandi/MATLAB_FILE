% ============================================================
% PLOT ACADO SOLVER STATUS
% ============================================================

t_exec = out.status.time;
T_exec = squeeze(out.status.signals.values);

figure
plot(t_exec, T_exec, 'LineWidth', 1.2)
grid on
xlabel('Simulation time [s]')
ylabel('Execution time [s]')
title('NMPC execution time')
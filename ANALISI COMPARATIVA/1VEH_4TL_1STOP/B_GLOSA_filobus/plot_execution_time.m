% ============================================================
% PLOT EXECUTION TIME NMPC
% ============================================================

t_exec = out.executionTime.time;
T_exec = squeeze(out.executionTime.signals.values);

figure
plot(t_exec, T_exec, 'LineWidth', 1.2)
grid on
xlabel('Simulation time [s]')
ylabel('Execution time [s]')
title('NMPC execution time')
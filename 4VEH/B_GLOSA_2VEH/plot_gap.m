% ============================================================
% PLOT GAP
% ============================================================

t_gap = out.states.time;
gap12 = squeeze(out.states.signals.values(:,1)-out.states.signals.values(:,4)-L_platoon);
gap23 = squeeze(out.states.signals.values(:,4)-out.states.signals.values(:,7)-L_platoon);


figure
plot(t_gap, gap12, 'LineWidth', 1.2)
grid on
xlabel('time [s]')
ylabel('gap [m]')
title('GAP 1-2')

figure
plot(t_gap, gap23, 'LineWidth', 1.2)
grid on
xlabel('time [s]')
ylabel('gap [m]')
title('GAP 1-2')
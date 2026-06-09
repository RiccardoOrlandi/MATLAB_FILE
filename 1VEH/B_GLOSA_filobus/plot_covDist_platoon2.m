function h = plot_covDist_platoon2(out,i_fig,s_TL,s_stop,L_platoon)
% ============================================================
% PLOT NMPC-BGLOSA 1 VEHICLE WITH FINITE LENGTH L_PLATOON
%
% Figure i_fig   : space-time trajectory + TL phases + stops
% Figure i_fig+1 : speed and longitudinal acceleration
%                  jerk (control input)
% Figure i_fig+2 : objective function, solver status,
%                  QP iterations, execution time
% ============================================================

%% ============================================================
% Extract states
%% ============================================================

t_states = out.states.time(:);
x_states = squeeze(out.states.signals.values);

if size(x_states,2) < 3
    error('out.states must have at least 3 columns: [s v a].');
end

s1_head = x_states(:,1);
v1_head = x_states(:,2);
a1_head = x_states(:,3);
s1_tail = s1_head - L_platoon;

%% ============================================================
% Extract control input
%% ============================================================

t_u   = out.u.time(:);
u_mat = local_signal_to_matrix(out.u.signals.values, length(t_u));
u1    = u_mat(:,1);

%% ============================================================
% Extract objective function
%% ============================================================

J  = squeeze(out.Objective_Value.signals.values);
J  = J(:);
if isprop(out.Objective_Value,'time') || isfield(out.Objective_Value,'time')
    t_J = out.Objective_Value.time(:);
else
    t_J = t_states;
end

%% ============================================================
% Extract solver status
%% ============================================================

solver_status = squeeze(out.status.signals.values);
solver_status = solver_status(:);
if isprop(out.status,'time') || isfield(out.status,'time')
    t_status = out.status.time(:);
else
    t_status = t_states;
end

%% ============================================================
% Extract QP iterations
%% ============================================================

nIter = squeeze(out.nIteration.signals.values);
nIter = nIter(:);
if isprop(out.nIteration,'time') || isfield(out.nIteration,'time')
    t_nIter = out.nIteration.time(:);
else
    t_nIter = t_status;
end

%% ============================================================
% Extract execution time
%% ============================================================

execT = squeeze(out.executionTime.signals.values);
execT = execT(:) * 1e3;   % [s] -> [ms]
if isprop(out.executionTime,'time') || isfield(out.executionTime,'time')
    t_execT = out.executionTime.time(:);
else
    t_execT = t_status;
end

%% ============================================================
%% FIGURE 1 — SPACE-TIME TRAJECTORY + TRAFFIC LIGHTS + STOPS
%% ============================================================

h.fig_traj = figure(i_fig);
clf(h.fig_traj)
hold on

% --- Traffic-light phases (vectorized) ---
t_tl = out.x_tl_hor.time(:);
nT   = length(t_tl);
nTL  = length(s_TL);

tl_state = squeeze(out.x_tl_hor.signals.values(1,1:nTL,1:nT));
if size(tl_state,1) == nTL
    tl_state = tl_state.';
end
nT_eff  = min(size(tl_state,1), nT);
nTL_eff = min(size(tl_state,2), nTL);
tl_state = tl_state(1:nT_eff, 1:nTL_eff);
t_tl     = t_tl(1:nT_eff);
s_TL_plt = s_TL(1:nTL_eff);

[T_grid, S_grid] = ndgrid(t_tl, s_TL_plt(:));
idx_green = (tl_state == 1);
hp_g = plot(T_grid( idx_green), S_grid( idx_green), '.g', 'MarkerSize', 6);
hp_r = plot(T_grid(~idx_green), S_grid(~idx_green), '.r', 'MarkerSize', 6);
hp_g.Annotation.LegendInformation.IconDisplayStyle = 'off';
hp_r.Annotation.LegendInformation.IconDisplayStyle = 'off';

% --- Bus stops ---
for jj = 1:length(s_stop)
    hp = plot([t_states(1) t_states(end)], [s_stop(jj) s_stop(jj)], '--k');
    hp.Annotation.LegendInformation.IconDisplayStyle = 'off';
end

% --- Vehicle head / tail ---
h_head = plot(t_states, s1_head, 'LineWidth', 2);
h_tail = plot(t_states, s1_tail, '--',  'LineWidth', 2);

grid on
xlabel('Time [s]')
ylabel('Covered distance [m]')
legend([h_head h_tail], {'Head','Tail'}, 'Location','best')
title('Space-time trajectory — head/tail and traffic-light phases')

%% ============================================================
%% FIGURE 2 — SPEED / ACCELERATION / JERK
%% ============================================================

h.fig_kin = figure(i_fig+1);
clf(h.fig_kin)

tl2 = tiledlayout(h.fig_kin, 3, 1, 'TileSpacing','compact','Padding','compact');

% --- Speed ---
ax1 = nexttile(tl2);
hold(ax1,'on')
g = plot(ax1, [t_states(1) t_states(end)], [15 15], '--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';
g = plot(ax1, [t_states(1) t_states(end)], [50 50], '--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';
plot(ax1, t_states, v1_head.*3.6, 'LineWidth', 2)
grid(ax1,'on')
ylabel(ax1, 'V  [km/h]')
ylim(ax1, [0 55])
title(ax1, 'Speed')

% --- Acceleration ---
ax2 = nexttile(tl2);
hold(ax2,'on')
yline(ax2,  1.5, '--k', 'HandleVisibility','off');
yline(ax2, -1.5, '--k', 'HandleVisibility','off');
plot(ax2, t_states, a1_head, 'LineWidth', 2)
grid(ax2,'on')
ylabel(ax2, 'a  [m/s^2]')
title(ax2, 'Longitudinal acceleration')

% --- Jerk ---
ax3 = nexttile(tl2);
hold(ax3,'on')
yline(ax3,  0.5, '--k', 'HandleVisibility','off');
yline(ax3, -0.5, '--k', 'HandleVisibility','off');
plot(ax3, t_u, u1, 'LineWidth', 2)
grid(ax3,'on')
ylabel(ax3, 'jerk  [m/s^3]')
xlabel(ax3, 'Time [s]')
title(ax3, 'Control input / jerk')

linkaxes([ax1 ax2 ax3], 'x')
sgtitle(tl2, 'Kinematics — speed, acceleration, jerk')

%% ============================================================
%% FIGURE 3 — SOLVER DIAGNOSTICS
%% ============================================================

h.fig_diag = figure(i_fig+2);
clf(h.fig_diag)

tl3 = tiledlayout(h.fig_diag, 4, 1, 'TileSpacing','compact','Padding','compact');

% --- Objective function ---
ax4 = nexttile(tl3);
plot(ax4, t_J, J, 'LineWidth', 2)
grid(ax4,'on')
ylabel(ax4, 'J  [-]')
title(ax4, 'NMPC objective function')

% --- Solver status ---
ax5 = nexttile(tl3);
stairs(ax5, t_status, solver_status, 'LineWidth', 2)
hold(ax5,'on')
yline(ax5, 0, '--k', 'HandleVisibility','off')
grid(ax5,'on')
ylabel(ax5, 'status  [-]')
title(ax5, 'Solver status')
pad5 = max(1, 0.15 * (max(solver_status) - min(solver_status)));
ylim(ax5, [min(solver_status)-pad5,  max(solver_status)+pad5])

% --- QP iterations ---
ax6 = nexttile(tl3);
stairs(ax6, t_nIter, nIter, 'LineWidth', 2)
grid(ax6,'on')
ylabel(ax6, 'iter  [-]')
title(ax6, 'QP iterations')
ylim(ax6, [0, max(nIter(:))*1.15 + 1])

% --- Execution time ---
ax7 = nexttile(tl3);
plot(ax7, t_execT, execT, 'LineWidth', 2)
hold(ax7,'on')
yline(ax7, mean(execT), '--r', 'Mean', 'LabelHorizontalAlignment','left', 'HandleVisibility','off')
grid(ax7,'on')
ylabel(ax7, 't_{exec}  [ms]')
xlabel(ax7, 'Time [s]')
title(ax7, 'Execution time')

linkaxes([ax4 ax5 ax6 ax7], 'x')
sgtitle(tl3, 'Solver diagnostics — objective, status, iterations, execution time')

%% ============================================================
% Console diagnostics
%% ============================================================

bad_idx = find(solver_status(:) < 0);
fprintf('\n=== NMPC diagnostics ===\n')
if isempty(bad_idx)
    fprintf('No negative solver status detected.\n')
else
    fprintf('Negative solver status at t = '); disp(t_status(bad_idx).')
end

fprintf('s_head  = [%.1f  %.1f] m\n',    min(s1_head), max(s1_head));
fprintf('v       = [%.2f  %.2f] km/h\n', min(v1_head)*3.6, max(v1_head)*3.6);
fprintf('a       = [%.3f  %.3f] m/s^2\n',min(a1_head), max(a1_head));
fprintf('jerk    = [%.3f  %.3f] m/s^3\n',min(u1), max(u1));
fprintf('t_exec  = mean %.2f ms  max %.2f ms\n', mean(execT), max(execT));
fprintf('nIter   = mean %.1f  max %d\n', mean(nIter), max(nIter));

end

%% ============================================================
% LOCAL: convert Simulink signal to [time x channels]
%% ============================================================

function M = local_signal_to_matrix(values, n_time)
values = squeeze(values);
if isvector(values)
    M = values(:);
elseif size(values,1) == n_time
    M = values;
elseif size(values,2) == n_time
    M = values.';
else
    M = reshape(values,[],n_time).';
end
end

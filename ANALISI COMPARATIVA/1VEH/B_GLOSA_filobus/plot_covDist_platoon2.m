function h = plot_covDist_platoon2(out,i_fig,s_TL,s_stop,L_platoon)
% ============================================================
% PLOT NMPC-BGLOSA RIGID PLATOON RESULTS
% Figure 1: trajectory + traffic lights + stops
% Figure 2: all diagnostics in one figure
% ============================================================

%% =======================
% Extract main signals
% =======================

t_states = out.states.time(:);
x_states = out.states.signals.values;

t_u = out.u.time(:);
u = squeeze(out.u.signals.values);
u = u(:);

s_head = x_states(:,1);
v_head = x_states(:,2);
a_head = x_states(:,3);

s_tail = s_head - L_platoon;

J = squeeze(out.Objective_Value.signals.values);
J = J(:);

if isfield(out.Objective_Value,'time')
    t_J = out.Objective_Value.time(:);
else
    t_J = t_states;
end

solver_status = squeeze(out.status.signals.values);
solver_status = solver_status(:);

if isfield(out.status,'time')
    t_status = out.status.time(:);
else
    t_status = t_states;
end

%% ============================================================
% FIGURE 1: SPACE-TIME TRAJECTORY + TRAFFIC LIGHTS + STOPS
% ============================================================

h.fig_traj = figure(i_fig);
clf(h.fig_traj)
hold on

for kk = 1:length(out.x_tl_hor.time)
    for ii = 1:length(s_TL)

        if out.x_tl_hor.signals.values(1,ii,kk) == 1
            hp = plot(out.x_tl_hor.time(kk),s_TL(ii),'.g');
        else
            hp = plot(out.x_tl_hor.time(kk),s_TL(ii),'.r');
        end

        hp.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end

for jj = 1:length(s_stop)
    hp = plot([t_states(1) t_states(end)], ...
              [s_stop(jj) s_stop(jj)],'--k');

    hp.Annotation.LegendInformation.IconDisplayStyle = 'off';
end

h_head = plot(t_states, s_head, 'LineWidth', 2);
h_tail = plot(t_states, s_tail, '--', 'LineWidth', 2);

grid on
xlabel('Time [s]')
ylabel('Covered distance [m]')
legend([h_head h_tail],{'Head','Tail'},'Location','best')
title('Platoon head/tail trajectory and traffic-light phases')

%% ============================================================
% FIGURE 2: ALL OTHER PLOTS IN ONE FIGURE
% ============================================================

h.fig_diag = figure(i_fig+1);
clf(h.fig_diag)

tl = tiledlayout(h.fig_diag,5,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% =======================
% Speed
% =======================

ax1 = nexttile(tl);
hold(ax1,'on')

g = plot(ax1,[t_states(1) t_states(end)],[15 15],'--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';

g = plot(ax1,[t_states(1) t_states(end)],[50 50],'--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';

plot(ax1,t_states, v_head.*3.6, 'LineWidth', 2)

grid(ax1,'on')
xlabel(ax1,'Time [s]')
ylabel(ax1,'V [km/h]')
ylim(ax1,[0 55])
title(ax1,'Speed')

%% =======================
% Acceleration
% =======================

ax2 = nexttile(tl);
plot(ax2,t_states, a_head, 'LineWidth', 2)

grid(ax2,'on')
xlabel(ax2,'Time [s]')
ylabel(ax2,'A_x [m/s^2]')
title(ax2,'Longitudinal acceleration')

%% =======================
% Control input
% =======================

ax3 = nexttile(tl);
plot(ax3,t_u, u, 'LineWidth', 2)

grid(ax3,'on')
xlabel(ax3,'Time [s]')
ylabel(ax3,'u [m/s^3]')
title(ax3,'Control input')

%% =======================
% Objective function
% =======================

ax4 = nexttile(tl);
plot(ax4,t_J, J, 'LineWidth', 2)

grid(ax4,'on')
xlabel(ax4,'Time [s]')
ylabel(ax4,'J [-]')
title(ax4,'NMPC objective function')

%% =======================
% Solver status
% =======================

ax5 = nexttile(tl);
stairs(ax5,t_status, solver_status, 'LineWidth', 2)
hold(ax5,'on')
yline(ax5,0,'--k')

grid(ax5,'on')
xlabel(ax5,'Time [s]')
ylabel(ax5,'status [-]')
title(ax5,'Solver status')

status_min = min(solver_status(:));
status_max = max(solver_status(:));

if status_min == status_max
    ylim(ax5,[status_min-1 status_max+1])
else
    ylim(ax5,[status_min-1 status_max+1])
end

linkaxes([ax1 ax2 ax3 ax4 ax5],'x')

sgtitle(tl,'NMPC-BGLOSA diagnostics')

%% ============================================================
% Print critical solver events
% ============================================================

bad_idx = find(solver_status(:) < 0);

fprintf('\n=== NMPC diagnostics ===\n')

if isempty(bad_idx)
    fprintf('No negative solver status detected.\n')
else
    fprintf('Negative solver status detected at:\n')
    disp(t_status(bad_idx))
end

end
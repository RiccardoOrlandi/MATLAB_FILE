function h = plot_covDist_2veh_platoon(out,i_fig,s_TL,s_stop,L_platoon)
% ============================================================
% PLOT NMPC-BGLOSA 2 VEHICLES WITH FINITE LENGTH L_PLATOON
%
% State convention:
%   out.states(:,1) = s1
%   out.states(:,2) = v1
%   out.states(:,3) = a1
%   out.states(:,4) = s2
%   out.states(:,5) = v2
%   out.states(:,6) = a2
%
% Vehicle length:
%   s_tail_i = s_head_i - L_platoon
%
% Figure i_fig:
%   space-time trajectory of both vehicles + TL phases + stops
%
% Figure i_fig+1:
%   diagnostics: speed, acceleration, control input, objective, solver status
% ============================================================

%% ============================================================
% Extract main signals
% ============================================================

t_states = out.states.time(:);
x_states = out.states.signals.values;

s1_head = x_states(:,1);
v1_head = x_states(:,2);
a1_head = x_states(:,3);

s2_head = x_states(:,4);
v2_head = x_states(:,5);
a2_head = x_states(:,6);

s1_tail = s1_head - L_platoon;
s2_tail = s2_head - L_platoon;

%% ============================================================
% Extract control input
% ============================================================

t_u = out.u.time(:);
u_raw = out.u.signals.values;
u_mat = local_signal_to_matrix(u_raw,length(t_u));

u1 = u_mat(:,1);

if size(u_mat,2) >= 2
    u2 = u_mat(:,2);
else
    u2 = nan(size(u1));
end

%% ============================================================
% Extract objective function
% ============================================================

J = squeeze(out.Objective_Value.signals.values);
J = J(:);

if isfield(out.Objective_Value,'time')
    t_J = out.Objective_Value.time(:);
else
    t_J = t_states;
end

%% ============================================================
% Extract solver status
% ============================================================

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

% ------------------------------------------------------------
% Traffic-light phases - vectorized plotting
% ------------------------------------------------------------

t_tl = out.x_tl_hor.time(:);
nT  = length(t_tl);
nTL = length(s_TL);

tl_state = squeeze(out.x_tl_hor.signals.values(1,1:nTL,1:nT));

% Dopo squeeze, normalmente tl_state è [nTL x nT].
% Lo porto sempre nel formato [nT x nTL].
if size(tl_state,1) == nTL
    tl_state = tl_state.';
end

% Griglie tempo-posizione semaforo
[T_grid,S_grid] = ndgrid(t_tl,s_TL(:));

idx_green = (tl_state == 1);
idx_red   = ~idx_green;

hp_g = plot(T_grid(idx_green),S_grid(idx_green),'.g','MarkerSize',6);
hp_r = plot(T_grid(idx_red),  S_grid(idx_red),  '.r','MarkerSize',6);

hp_g.Annotation.LegendInformation.IconDisplayStyle = 'off';
hp_r.Annotation.LegendInformation.IconDisplayStyle = 'off';

% ------------------------------------------------------------
% Bus stops
% ------------------------------------------------------------

for jj = 1:length(s_stop)

    hp = plot([t_states(1) t_states(end)], ...
              [s_stop(jj) s_stop(jj)],'--k');

    hp.Annotation.LegendInformation.IconDisplayStyle = 'off';

end

% ------------------------------------------------------------
% Vehicle trajectories
% ------------------------------------------------------------

h_v1_head = plot(t_states,s1_head,'LineWidth',2);
h_v1_tail = plot(t_states,s1_tail,'--','LineWidth',2);

h_v2_head = plot(t_states,s2_head,'LineWidth',2);
h_v2_tail = plot(t_states,s2_tail,'--','LineWidth',2);

grid on
xlabel('Time [s]')
ylabel('Covered distance [m]')

legend([h_v1_head h_v1_tail h_v2_head h_v2_tail], ...
       {'Vehicle 1 head','Vehicle 1 tail', ...
        'Vehicle 2 head','Vehicle 2 tail'}, ...
       'Location','best')

title('Two finite-length vehicles: head/tail trajectories and traffic-light phases')

%% ============================================================
% FIGURE 2: DIAGNOSTICS
% ============================================================

h.fig_diag = figure(i_fig+1);
clf(h.fig_diag)

tl = tiledlayout(h.fig_diag,5,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% ============================================================
% Speed
% ============================================================

ax1 = nexttile(tl);
hold(ax1,'on')

g = plot(ax1,[t_states(1) t_states(end)],[15 15],'--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';

g = plot(ax1,[t_states(1) t_states(end)],[50 50],'--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';

plot(ax1,t_states,v1_head.*3.6,'LineWidth',2)
plot(ax1,t_states,v2_head.*3.6,'LineWidth',2)

grid(ax1,'on')
xlabel(ax1,'Time [s]')
ylabel(ax1,'V [km/h]')
ylim(ax1,[0 55])
legend(ax1,{'Vehicle 1','Vehicle 2'},'Location','best')
title(ax1,'Speed')

%% ============================================================
% Acceleration
% ============================================================

ax2 = nexttile(tl);
hold(ax2,'on')

plot(ax2,t_states,a1_head,'LineWidth',2)
plot(ax2,t_states,a2_head,'LineWidth',2)

grid(ax2,'on')
xlabel(ax2,'Time [s]')
ylabel(ax2,'A_x [m/s^2]')
legend(ax2,{'Vehicle 1','Vehicle 2'},'Location','best')
title(ax2,'Longitudinal acceleration')

%% ============================================================
% Control input
% ============================================================

ax3 = nexttile(tl);
hold(ax3,'on')

plot(ax3,t_u,u1,'LineWidth',2)

if all(isfinite(u2))
    plot(ax3,t_u,u2,'LineWidth',2)
    legend(ax3,{'Vehicle 1','Vehicle 2'},'Location','best')
else
    legend(ax3,{'Vehicle 1'},'Location','best')
end

grid(ax3,'on')
xlabel(ax3,'Time [s]')
ylabel(ax3,'u [m/s^3]')
title(ax3,'Control input / jerk')

%% ============================================================
% Objective function
% ============================================================

ax4 = nexttile(tl);

plot(ax4,t_J,J,'LineWidth',2)

grid(ax4,'on')
xlabel(ax4,'Time [s]')
ylabel(ax4,'J [-]')
title(ax4,'NMPC objective function')

%% ============================================================
% Solver status
% ============================================================

ax5 = nexttile(tl);
stairs(ax5,t_status,solver_status,'LineWidth',2)
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

sgtitle(tl,'NMPC-BGLOSA diagnostics - two finite-length vehicles')

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

%% ============================================================
% LOCAL FUNCTION: convert Simulink signal to [time x channels]
% ============================================================

function M = local_signal_to_matrix(values,n_time)

values = squeeze(values);

if isvector(values)

    M = values(:);

else

    if size(values,1) == n_time

        M = values;

    elseif size(values,2) == n_time

        M = values.';

    else

        values = reshape(values,[],n_time).';
        M = values;

    end

end

end

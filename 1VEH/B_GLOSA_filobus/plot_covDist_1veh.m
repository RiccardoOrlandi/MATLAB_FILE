function h = plot_covDist_1veh(out,i_fig,s_TL,s_stop,L_platoon)
%==========================================================================
% PLOT NMPC-BGLOSA 1 VEHICLE WITH FINITE LENGTH L_PLATOON
%==========================================================================
%
% State convention:
%   out.states(:,1) = s1
%   out.states(:,2) = v1
%   out.states(:,3) = a1
%
% Vehicle length:
%   s_tail_1 = s_head_1 - L_platoon
%
% Figure i_fig:
%   space-time trajectory + TL phases + stops
%
% Figure i_fig+1:
%   diagnostics: speed, acceleration, control input, objective, solver status
%==========================================================================

%% ============================================================
% Extract main signals
% ============================================================

t_states = out.states.time(:);
x_states = out.states.signals.values;
x_states = squeeze(x_states);

if size(x_states,2) < 3
    error('out.states must contain at least 3 columns: [s1 v1 a1].');
end

s1_head = x_states(:,1);
v1_head = x_states(:,2);
a1_head = x_states(:,3);

s1_tail = s1_head - L_platoon;

%% ============================================================
% Extract control input
% ============================================================

t_u = out.u.time(:);
u_raw = out.u.signals.values;
u_mat = local_signal_to_matrix(u_raw,length(t_u));

u1 = u_mat(:,1);

%% ============================================================
% Extract objective function
% ============================================================

J = squeeze(out.Objective_Value.signals.values);
J = J(:);

if isprop(out.Objective_Value,'time') || isfield(out.Objective_Value,'time')
    t_J = out.Objective_Value.time(:);
else
    t_J = t_states;
end

%% ============================================================
% Extract solver status
% ============================================================

solver_status = squeeze(out.status.signals.values);
solver_status = solver_status(:);

if isprop(out.status,'time') || isfield(out.status,'time')
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

tl_values = out.x_tl_hor.signals.values;

% Caso nominale originale:
%   out.x_tl_hor.signals.values = [1 x nTL x nT]
%
% Dopo squeeze:
%   tl_state = [nTL x nT]
tl_state = squeeze(tl_values(1,1:nTL,1:nT));

% Porta sempre nel formato [nT x nTL]
if size(tl_state,1) == nTL
    tl_state = tl_state.';
end

% Protezione dimensionale
nT_eff  = min(size(tl_state,1),nT);
nTL_eff = min(size(tl_state,2),nTL);

tl_state = tl_state(1:nT_eff,1:nTL_eff);
t_tl     = t_tl(1:nT_eff);
s_TL     = s_TL(1:nTL_eff);

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
              [s_stop(jj) s_stop(jj)], ...
              '--k');

    hp.Annotation.LegendInformation.IconDisplayStyle = 'off';

end

% ------------------------------------------------------------
% Vehicle trajectory
% ------------------------------------------------------------

h_v1_head = plot(t_states,s1_head,'LineWidth',2);
h_v1_tail = plot(t_states,s1_tail,'--','LineWidth',2);

grid on
xlabel('Time [s]')
ylabel('Covered distance [m]')

legend([h_v1_head h_v1_tail], ...
       {'Vehicle 1 head','Vehicle 1 tail'}, ...
       'Location','best')

title('Single finite-length vehicle: head/tail trajectory and traffic-light phases')

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

grid(ax1,'on')
xlabel(ax1,'Time [s]')
ylabel(ax1,'V [km/h]')
ylim(ax1,[0 55])
legend(ax1,{'Vehicle 1'},'Location','best')
title(ax1,'Speed')

%% ============================================================
% Acceleration
% ============================================================

ax2 = nexttile(tl);
hold(ax2,'on')

plot(ax2,t_states,a1_head,'LineWidth',2)

grid(ax2,'on')
xlabel(ax2,'Time [s]')
ylabel(ax2,'A_x [m/s^2]')
legend(ax2,{'Vehicle 1'},'Location','best')
title(ax2,'Longitudinal acceleration')

%% ============================================================
% Control input
% ============================================================

ax3 = nexttile(tl);
hold(ax3,'on')

plot(ax3,t_u,u1,'LineWidth',2)

grid(ax3,'on')
xlabel(ax3,'Time [s]')
ylabel(ax3,'u [m/s^3]')
legend(ax3,{'Vehicle 1'},'Location','best')
title(ax3,'Control input / jerk')

%% ============================================================
% Objective function
% ============================================================

ax4 = nexttile(tl);
hold(ax4,'on')

plot(ax4,t_J,J,'LineWidth',2)

grid(ax4,'on')
xlabel(ax4,'Time [s]')
ylabel(ax4,'J [-]')
title(ax4,'NMPC objective function')

%% ============================================================
% Solver status
% ============================================================

ax5 = nexttile(tl);
hold(ax5,'on')

stairs(ax5,t_status,solver_status,'LineWidth',2)
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

sgtitle(tl,'NMPC-BGLOSA diagnostics - single finite-length vehicle')

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

%% ============================================================
% Optional vehicle diagnostics
% ============================================================

fprintf('\n=== Vehicle diagnostics ===\n')
fprintf('s_head min/max = %.3f / %.3f m\n',min(s1_head),max(s1_head));
fprintf('s_tail min/max = %.3f / %.3f m\n',min(s1_tail),max(s1_tail));
fprintf('v min/max      = %.3f / %.3f km/h\n',min(v1_head)*3.6,max(v1_head)*3.6);
fprintf('a min/max      = %.3f / %.3f m/s^2\n',min(a1_head),max(a1_head));

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
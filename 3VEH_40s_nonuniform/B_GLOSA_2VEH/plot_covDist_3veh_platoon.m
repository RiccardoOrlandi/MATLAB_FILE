function h = plot_covDist_3veh_platoon(out, i_fig, s_TL, s_stop, L_platoon)
% ============================================================
% PLOT_COVDIST_3VEH_PLATOON
% Genera 3 figure:
%   Fig i_fig   - Traiettoria spazio-tempo + fasi TL + fermate
%   Fig i_fig+1 - Jerk / Velocita' / Accelerazione (3 veicoli sovrapposti)
%   Fig i_fig+2 - Execution time / Objective / nIteration / Solver status
% ============================================================

%% ----------------------------------------------------------
%  1. ESTRAZIONE STATI
%  ----------------------------------------------------------

t_st = out.states.time(:);
xs   = out.states.signals.values;          % [1601 x 9]

s1 = xs(:,1);  v1 = xs(:,2);  a1 = xs(:,3);
s2 = xs(:,4);  v2 = xs(:,5);  a2 = xs(:,6);
s3 = xs(:,7);  v3 = xs(:,8);  a3 = xs(:,9);

s1t = s1 - L_platoon;
s2t = s2 - L_platoon;
s3t = s3 - L_platoon;

%% ----------------------------------------------------------
%  2. ESTRAZIONE CONTROLLI (u, u2, u3 = struct [1x1xN])
%  ----------------------------------------------------------

t_u = out.u.time(:);
u1  = squeeze(out.u.signals.values);        % [1601 x 1] -> [1601]
u2  = squeeze(out.u2.signals.values);
u3  = squeeze(out.u3.signals.values);

%% ----------------------------------------------------------
%  3. ESTRAZIONE SEGNALI DI DIAGNOSTICA
%  ----------------------------------------------------------

t_J  = out.Objective_Value.time(:);
J    = squeeze(out.Objective_Value.signals.values);

t_st2 = out.status.time(:);
stat  = squeeze(out.status.signals.values);

t_ni  = out.nIteration.time(:);
nIter = squeeze(out.nIteration.signals.values);

t_ex  = out.executionTime.time(:);
exT   = squeeze(out.executionTime.signals.values);

%% ----------------------------------------------------------
%  PALETTE COLORI
%  ----------------------------------------------------------

cV1 = [0.00, 0.45, 0.70];   % blu
cV2 = [0.77, 0.29, 0.77];   % viola
cV3 = [0.91, 0.63, 0.00];   % ambra
lw  = 1.8;

%% ==========================================================
%  FIGURA 1: TRAIETTORIA SPAZIO-TEMPO
%% ==========================================================

h.fig_traj = figure(i_fig);
clf(h.fig_traj);
hold on;

% --- Fasi semaforiche ---
t_tl    = out.x_tl_hor.time(:);
nT      = length(t_tl);
nTL     = length(s_TL);
tl_raw  = out.x_tl_hor.signals.values;
tl_st   = squeeze(tl_raw(1, 1:nTL, 1:nT));  % [nTL x nT]
if size(tl_st,1) == nTL && size(tl_st,2) == nT
    tl_st = tl_st.';                          % -> [nT x nTL]
end
[T_g, S_g] = ndgrid(t_tl, s_TL(:));
pg = plot(T_g(tl_st==1), S_g(tl_st==1), '.g', 'MarkerSize', 5);
pr = plot(T_g(tl_st==0), S_g(tl_st==0), '.r', 'MarkerSize', 5);
pg.Annotation.LegendInformation.IconDisplayStyle = 'off';
pr.Annotation.LegendInformation.IconDisplayStyle = 'off';

% --- Fermate bus ---
for jj = 1:length(s_stop)
    hs = plot([t_st(1) t_st(end)], [s_stop(jj) s_stop(jj)], '--k', 'LineWidth', 0.9);
    hs.Annotation.LegendInformation.IconDisplayStyle = 'off';
end

% --- Traiettorie ---
hh(1) = plot(t_st, s1,  '-',  'Color', cV1, 'LineWidth', lw);
hh(2) = plot(t_st, s1t, '--', 'Color', cV1, 'LineWidth', lw);
hh(3) = plot(t_st, s2,  '-',  'Color', cV2, 'LineWidth', lw);
hh(4) = plot(t_st, s2t, '--', 'Color', cV2, 'LineWidth', lw);
hh(5) = plot(t_st, s3,  '-',  'Color', cV3, 'LineWidth', lw);
hh(6) = plot(t_st, s3t, '--', 'Color', cV3, 'LineWidth', lw);

grid on; box on;
xlabel('Tempo [s]');  ylabel('Posizione [m]');
legend(hh, {'Veh 1 testa','Veh 1 coda', ...
            'Veh 2 testa','Veh 2 coda', ...
            'Veh 3 testa','Veh 3 coda'}, 'Location', 'best');
title('Traiettorie spazio-tempo – fasi TL e fermate bus');

%% ==========================================================
%  FIGURA 2: JERK / VELOCITA' / ACCELERAZIONE
%% ==========================================================

h.fig_kinem = figure(i_fig+1);
clf(h.fig_kinem);

tl2 = tiledlayout(h.fig_kinem, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- Jerk ---
ax_j = nexttile(tl2);
hold(ax_j, 'on');
yline(ax_j,  0.5, '--k', 'LineWidth', 0.9);
yline(ax_j, -0.5, '--k', 'LineWidth', 0.9);
plot(ax_j, t_u, u1, '-', 'Color', cV1, 'LineWidth', lw, 'DisplayName', 'Veh 1');
plot(ax_j, t_u, u2, '-', 'Color', cV2, 'LineWidth', lw, 'DisplayName', 'Veh 2');
plot(ax_j, t_u, u3, '-', 'Color', cV3, 'LineWidth', lw, 'DisplayName', 'Veh 3');
grid(ax_j, 'on'); box(ax_j, 'on');
ylabel(ax_j, 'Jerk [m/s^3]');
legend(ax_j, 'Location', 'best');
title(ax_j, 'Jerk (ingresso di controllo)');

% --- Velocita' ---
ax_v = nexttile(tl2);
hold(ax_v, 'on');
yline(ax_v, 50, '--k', 'LineWidth', 0.9);
plot(ax_v, t_st, v1.*3.6, '-', 'Color', cV1, 'LineWidth', lw, 'DisplayName', 'Veh 1');
plot(ax_v, t_st, v2.*3.6, '-', 'Color', cV2, 'LineWidth', lw, 'DisplayName', 'Veh 2');
plot(ax_v, t_st, v3.*3.6, '-', 'Color', cV3, 'LineWidth', lw, 'DisplayName', 'Veh 3');
grid(ax_v, 'on'); box(ax_v, 'on');
ylabel(ax_v, 'V [km/h]');
ylim(ax_v, [0 55]);
legend(ax_v, 'Location', 'best');
title(ax_v, 'Velocita');

% --- Accelerazione ---
ax_a = nexttile(tl2);
hold(ax_a, 'on');
yline(ax_a,  1.5, '--k', 'LineWidth', 0.9);
yline(ax_a, -1.5, '--k', 'LineWidth', 0.9);
plot(ax_a, t_st, a1, '-', 'Color', cV1, 'LineWidth', lw, 'DisplayName', 'Veh 1');
plot(ax_a, t_st, a2, '-', 'Color', cV2, 'LineWidth', lw, 'DisplayName', 'Veh 2');
plot(ax_a, t_st, a3, '-', 'Color', cV3, 'LineWidth', lw, 'DisplayName', 'Veh 3');
grid(ax_a, 'on'); box(ax_a, 'on');
ylabel(ax_a, 'A_x [m/s^2]');
legend(ax_a, 'Location', 'best');
title(ax_a, 'Accelerazione longitudinale');
xlabel(ax_a, 'Tempo [s]');

linkaxes([ax_j ax_v ax_a], 'x');
sgtitle(tl2, 'Cinematica – 3 veicoli sovrapposti');

%% ==========================================================
%  FIGURA 3: DIAGNOSTICA SOLVER
%% ==========================================================

h.fig_solver = figure(i_fig+2);
clf(h.fig_solver);

tl3 = tiledlayout(h.fig_solver, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- Execution time ---
ax_e = nexttile(tl3);
plot(ax_e, t_ex, exT.*1e3, '-', 'Color', [0.2 0.2 0.6], 'LineWidth', 1.2);
hold(ax_e, 'on');
yline(ax_e, mean(exT)*1e3, '--r', 'LineWidth', 1, 'DisplayName', ...
     sprintf('media %.1f ms', mean(exT)*1e3));
grid(ax_e, 'on'); box(ax_e, 'on');
ylabel(ax_e, 't_{exec} [ms]');
legend(ax_e, 'Location', 'best');
title(ax_e, sprintf('Execution time    media=%.1f ms  max=%.1f ms', mean(exT)*1e3, max(exT)*1e3));

% --- Objective function ---
ax_o = nexttile(tl3);
plot(ax_o, t_J, J, '-', 'Color', [0.6 0.1 0.1], 'LineWidth', 1.2);
grid(ax_o, 'on'); box(ax_o, 'on');
ylabel(ax_o, 'J [-]');
title(ax_o, 'Objective function');

% --- nIteration ---
ax_n = nexttile(tl3);
stairs(ax_n, t_ni, nIter, '-', 'Color', [0.1 0.5 0.1], 'LineWidth', 1.2);
hold(ax_n, 'on');
yline(ax_n, mean(nIter), '--r', 'LineWidth', 1, 'DisplayName', ...
     sprintf('media %.0f', mean(nIter)));
grid(ax_n, 'on'); box(ax_n, 'on');
ylabel(ax_n, 'nIter [-]');
legend(ax_n, 'Location', 'best');
title(ax_n, sprintf('Iterazioni QP    media=%.0f  max=%.0f', mean(nIter), max(nIter)));

% --- Solver status ---
ax_s = nexttile(tl3);
hold(ax_s, 'on');
% sfondo rosso per status < 0 (compatibile R2022b)
bad_mask = stat < 0;
bad_t    = t_st2(bad_mask);
y_lo = min(stat)-1.5;  y_hi = max(stat)+1.5;
if y_lo == y_hi; y_lo = y_lo-1; y_hi = y_hi+1; end
for bi = 1:length(bad_t)
    patch(ax_s, [bad_t(bi)-0.5 bad_t(bi)+0.5 bad_t(bi)+0.5 bad_t(bi)-0.5], ...
          [y_lo y_lo y_hi y_hi], [1 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end
stairs(ax_s, t_st2, stat, '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 1.5);
yline(ax_s, 0, '--k', 'LineWidth', 0.9);
grid(ax_s, 'on'); box(ax_s, 'on');
ylabel(ax_s, 'status [-]');
xlabel(ax_s, 'Tempo [s]');
st_lims = [min(stat)-1, max(stat)+1];
if st_lims(1) == st_lims(2), st_lims = st_lims + [-1 1]; end
ylim(ax_s, st_lims);
n_bad = sum(bad_mask);
title(ax_s, sprintf('Solver status    0=OK  -2=infeasible  +1=maxIter   (%d step negativi)', n_bad));

linkaxes([ax_e ax_o ax_n ax_s], 'x');
sgtitle(tl3, 'Diagnostica solver NMPC');

%% ----------------------------------------------------------
%  STAMPA RIEPILOGO
%  ----------------------------------------------------------

gap12 = s1 - s2 - L_platoon;
gap23 = s2 - s3 - L_platoon;

fprintf("\n=== Riepilogo simulazione ===\n");
fprintf("  Gap min 1-2 : %.3f m\n", min(gap12));
fprintf("  Gap min 2-3 : %.3f m\n", min(gap23));
fprintf("  |u1| max    : %.4f m/s3\n", max(abs(u1)));
fprintf("  |u2| max    : %.4f m/s3\n", max(abs(u2)));
fprintf("  |u3| max    : %.4f m/s3\n", max(abs(u3)));
fprintf("  J max       : %.2f\n",     max(J));
fprintf("  exec max    : %.1f ms\n",  max(exT)*1e3);
fprintf("  exec media  : %.1f ms\n",  mean(exT)*1e3);
fprintf("  nIter max   : %.0f\n",     max(nIter));
n_bad = sum(stat < 0);
fprintf("  status < 0  : %d step (%.1f%%)\n", n_bad, 100*n_bad/length(stat));
fprintf("==============================\n");

end

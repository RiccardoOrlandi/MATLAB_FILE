%% ============================================================
%  DEBUG AUTOMATICO RIPARTENZA PREDETTA DOPO FERMATA
%  GLOSA 3 VEHICLES - NMPC ACADO
%
%  Obiettivo:
%  1) Analizzare la predizione prima, durante e dopo l'arrivo reale
%     alla fermata.
%
%  2) Trovare il PRIMO istante in cui la predizione vede:
%
%        approach -> stop/dwell -> restart
%
%     oppure, se il veicolo è già fermo:
%
%        dwell -> restart
%
%  3) Dire se la ripartenza viene vista:
%        - prima dell'arrivo reale;
%        - mentre il veicolo è già lento/fermo alla fermata;
%        - dopo.
%
%  4) Stampare gli intervalli reali in cui il veicolo rimane fermo
%     alla fermata.
%
%  Stati:
%      X = [s1 v1 a1 s2 v2 a2 s3 v3 a3]
%
%  Predizione:
%      state_pred = [N+1 x nx x n_samples_pred]
%      esempio: [51 x 9 x 753]
% ============================================================
 
clc;
 
%% =======================
% Parametri principali
% =======================
 
N  = 50;
Ts = 1;
 
s_stop = [80, 447, 756, 1084, 1304, 1507, 1822];
 
% Veicolo da analizzare
veh_id = 3;
 
% Fermata da analizzare
stop_to_analyze = 756;
 
% Finestra di scansione:
% - prima dell'arrivo reale: quanto indietro guardare
% - dopo l'arrivo reale: quanto avanti continuare a guardare
%
% Se vuoi guardare tutto prima, lascia Inf.
max_scan_before_arrival = Inf;
 
% Continua anche dopo l'arrivo, perché la ripartenza può essere vista
% quando il veicolo è già fermo alla fermata.
max_scan_after_arrival = 80;      % [s]
 
% Tolleranze per trovare l'arrivo reale alla fermata
tol_stop_arrival = 1.0;       % [m]
tol_vel_arrival  = 0.30;      % [m/s]
 
% Tolleranze per classificare la predizione
near_stop_tol_pred = 1.5;     % [m]
v_stop_threshold   = 0.20;    % [m/s]
restart_threshold  = 0.50;    % [m/s]
 
% Tolleranze per individuare la fermata reale effettiva
real_near_stop_tol          = 1.0;    % [m]
real_slow_at_stop_threshold = 0.30;   % [m/s]
real_stationary_threshold   = 0.05;   % [m/s]
 
% Opzioni output
save_csv = true;
csv_folder = 'pred_stop_debug_tables';
 
% Stampa la tabella completa dei 50 nodi al primo istante in cui
% la predizione vede la ripartenza
print_first_restart_nodes = true;
 
%% =======================
% Estrazione stati reali
% =======================
 
[t_real, Xsim] = local_get_signal(out, 'states');
 
if ndims(Xsim) > 2
    Xsim = squeeze(Xsim);
end
 
if size(Xsim,2) < 8 && size(Xsim,1) >= 8
    Xsim = Xsim.';
end
 
if size(Xsim,2) < 9
    error('La matrice stati deve avere 9 colonne: [s1 v1 a1 s2 v2 a2 s3 v3 a3].');
end
 
pos_cols = [1 4 7];
vel_cols = [2 5 8];
acc_cols = [3 6 9];
 
idx_s = pos_cols(veh_id);
idx_v = vel_cols(veh_id);
idx_a = acc_cols(veh_id);
 
pos_real = Xsim(:,idx_s);
vel_real = Xsim(:,idx_v);
acc_real = Xsim(:,idx_a);
 
fprintf('\n============================================================\n');
fprintf('DEBUG AUTOMATICO RIPARTENZA PREDETTA DOPO FERMATA\n');
fprintf('============================================================\n');
fprintf('Veicolo analizzato: %d\n', veh_id);
fprintf('Fermata analizzata: %.2f m\n', stop_to_analyze);
fprintf('Durata simulazione reale: %.2f s\n', t_real(end)-t_real(1));
fprintf('Numero campioni reali: %d\n', length(t_real));
fprintf('Ts medio stati reali: %.3f s\n', mean(diff(t_real)));
 
%% =======================
% Trova arrivo reale alla fermata
% =======================
 
idx_near_stop = find(abs(pos_real - stop_to_analyze) < tol_stop_arrival);
 
if isempty(idx_near_stop)
 
    [~, idx_arrival] = min(abs(pos_real - stop_to_analyze));
    warning('Il veicolo non entra entro %.2f m dalla fermata. Uso il punto più vicino.', tol_stop_arrival);
 
else
 
    idx_candidates = idx_near_stop(abs(vel_real(idx_near_stop)) < tol_vel_arrival);
 
    if isempty(idx_candidates)
        idx_arrival = idx_near_stop(1);
        warning('Il veicolo passa vicino alla fermata ma non ha velocità < %.2f m/s. Uso primo istante vicino.', tol_vel_arrival);
    else
        idx_arrival = idx_candidates(1);
    end
 
end
 
t_arrival = t_real(idx_arrival);
 
fprintf('\nArrivo/stazionamento reale circa:\n');
fprintf('t = %.2f s\n', t_arrival);
fprintf('pos = %.3f m, vel = %.3f m/s, acc = %.3f m/s^2\n', ...
    pos_real(idx_arrival), vel_real(idx_arrival), acc_real(idx_arrival));
 
%% =======================
% Intervalli reali in cui il veicolo è alla fermata
% =======================
 
mask_real_slow_at_stop = abs(pos_real - stop_to_analyze) <= real_near_stop_tol & ...
                         abs(vel_real) <= real_slow_at_stop_threshold;
 
mask_real_stationary_at_stop = abs(pos_real - stop_to_analyze) <= real_near_stop_tol & ...
                               abs(vel_real) <= real_stationary_threshold;
 
T_real_slow_intervals = local_mask_to_intervals( ...
    mask_real_slow_at_stop, t_real, pos_real, vel_real, acc_real);
 
T_real_stationary_intervals = local_mask_to_intervals( ...
    mask_real_stationary_at_stop, t_real, pos_real, vel_real, acc_real);
 
T_real_slow_samples = table( ...
    t_real(mask_real_slow_at_stop), ...
    pos_real(mask_real_slow_at_stop), ...
    vel_real(mask_real_slow_at_stop), ...
    acc_real(mask_real_slow_at_stop), ...
    'VariableNames', {'time_s','pos_m','vel_ms','acc_ms2'});
 
T_real_stationary_samples = table( ...
    t_real(mask_real_stationary_at_stop), ...
    pos_real(mask_real_stationary_at_stop), ...
    vel_real(mask_real_stationary_at_stop), ...
    acc_real(mask_real_stationary_at_stop), ...
    'VariableNames', {'time_s','pos_m','vel_ms','acc_ms2'});
 
fprintf('\n============================================================\n');
fprintf('INTERVALLI REALI: VEICOLO VICINO E LENTO ALLA FERMATA\n');
fprintf('Criterio: |pos - stop| <= %.2f m, |vel| <= %.2f m/s\n', ...
    real_near_stop_tol, real_slow_at_stop_threshold);
fprintf('============================================================\n');
 
if height(T_real_slow_intervals) == 0
    fprintf('Nessun intervallo trovato.\n');
else
    disp(T_real_slow_intervals);
end
 
fprintf('\n============================================================\n');
fprintf('INTERVALLI REALI: VEICOLO EFFETTIVAMENTE FERMO ALLA FERMATA\n');
fprintf('Criterio: |pos - stop| <= %.2f m, |vel| <= %.2f m/s\n', ...
    real_near_stop_tol, real_stationary_threshold);
fprintf('============================================================\n');
 
if height(T_real_stationary_intervals) == 0
    fprintf('Nessun intervallo trovato con soglia vel <= %.2f m/s.\n', real_stationary_threshold);
else
    disp(T_real_stationary_intervals);
end
 
assignin('base','T_real_slow_intervals',T_real_slow_intervals);
assignin('base','T_real_stationary_intervals',T_real_stationary_intervals);
assignin('base','T_real_slow_samples',T_real_slow_samples);
assignin('base','T_real_stationary_samples',T_real_stationary_samples);
 
%% =======================
% Estrazione traiettoria predetta
% =======================
 
[t_pred, Xpred_raw, pred_name] = local_find_prediction_signal(out, t_real);
 
fprintf('\nSegnale predizione trovato: %s\n', pred_name);
fprintf('Numero campioni predizione: %d\n', length(t_pred));
 
if length(t_pred) > 1
    fprintf('Ts medio predizione: %.3f s\n', mean(diff(t_pred)));
end
 
Xpred = local_format_prediction(Xpred_raw, length(t_pred), N, 9);
 
fprintf('Formato interno Xpred: [%d x %d x %d]\n', ...
    size(Xpred,1), size(Xpred,2), size(Xpred,3));
 
tau = (0:N)' * Ts;
node_index = (0:N)';
 
%% =======================
% Cartella CSV
% =======================
 
if save_csv
    if ~exist(csv_folder, 'dir')
        mkdir(csv_folder);
    end
end
 
%% =======================
% Definizione finestra di scansione
% =======================
 
if isinf(max_scan_before_arrival)
    t_scan_start = t_pred(1);
else
    t_scan_start = t_arrival - max_scan_before_arrival;
end
 
t_scan_end = t_arrival + max_scan_after_arrival;
 
% Se ci sono intervalli reali slow/stationary, estendi la scansione
% fino alla fine dell'ultimo intervallo lento + qualche secondo.
if height(T_real_slow_intervals) > 0
    t_last_slow_end = max(T_real_slow_intervals.t_end_s);
    t_scan_end = max(t_scan_end, t_last_slow_end + 20);
end
 
scan_indices = find(t_pred >= t_scan_start & t_pred <= t_scan_end);
 
if isempty(scan_indices)
    error('Nessun campione di predizione nella finestra di scansione.');
end
 
fprintf('\n============================================================\n');
fprintf('SCANSIONE AUTOMATICA PRIMA, DURANTE E DOPO LA FERMATA\n');
fprintf('============================================================\n');
fprintf('Numero istanti predetti analizzati: %d\n', length(scan_indices));
fprintf('Intervallo scansione: %.2f s -> %.2f s\n', ...
    t_pred(scan_indices(1)), t_pred(scan_indices(end)));
fprintf('Arrivo reale circa: %.2f s\n', t_arrival);
 
%% =======================
% Scansione automatica
% =======================
 
ScanRows = [];
 
first_restart_seen_found = false;
first_restart_k_pred = NaN;
first_restart_A = [];
first_restart_T_nodes = table();
first_restart_real_status = '';
 
first_restart_before_arrival_found = false;
first_restart_during_slow_found = false;
first_restart_during_stationary_found = false;
 
first_before_arrival_time = NaN;
first_during_slow_time = NaN;
first_during_stationary_time = NaN;
 
for ss = 1:length(scan_indices)
 
    k_pred = scan_indices(ss);
    t_current = t_pred(k_pred);
    offset_real = t_current - t_arrival;
 
    s_pred = squeeze(Xpred(k_pred,:,idx_s)).';
    v_pred = squeeze(Xpred(k_pred,:,idx_v)).';
    a_pred = squeeze(Xpred(k_pred,:,idx_a)).';
 
    A = local_analyze_prediction_stop( ...
        s_pred, v_pred, a_pred, ...
        t_current, stop_to_analyze, ...
        tau, node_index, ...
        near_stop_tol_pred, v_stop_threshold, restart_threshold, ...
        false);
 
    real_pos_k = interp1(t_real, pos_real, t_current, 'linear', 'extrap');
    real_vel_k = interp1(t_real, vel_real, t_current, 'linear', 'extrap');
    real_acc_k = interp1(t_real, acc_real, t_current, 'linear', 'extrap');
 
    real_near_stop_k = abs(real_pos_k - stop_to_analyze) <= real_near_stop_tol;
    real_slow_k = real_near_stop_k && abs(real_vel_k) <= real_slow_at_stop_threshold;
    real_stationary_k = real_near_stop_k && abs(real_vel_k) <= real_stationary_threshold;
 
    if t_current < t_arrival
        real_status_code = 0;
        real_status_text = 'BEFORE_ARRIVAL';
    elseif real_stationary_k
        real_status_code = 3;
        real_status_text = 'STATIONARY_AT_STOP';
    elseif real_slow_k
        real_status_code = 2;
        real_status_text = 'SLOW_AT_STOP';
    elseif real_near_stop_k
        real_status_code = 1;
        real_status_text = 'NEAR_STOP_NOT_SLOW';
    else
        real_status_code = 4;
        real_status_text = 'AFTER_OR_AWAY_FROM_STOP';
    end
 
    ScanRows = [ScanRows; ...
        ss, ...
        t_current, ...
        offset_real, ...
        real_pos_k, ...
        real_vel_k, ...
        real_acc_k, ...
        real_status_code, ...
        A.pred_min_dist_stop, ...
        A.n_stopped_steps, ...
        A.first_stop_tau, ...
        A.last_stop_tau, ...
        A.dwell_duration, ...
        double(A.restart_seen), ...
        A.restart_tau, ...
        A.restart_abs_time, ...
        A.restart_pos, ...
        A.max_vel_after_stop];
 
    if A.restart_seen && t_current < t_arrival && ~first_restart_before_arrival_found
        first_restart_before_arrival_found = true;
        first_before_arrival_time = t_current;
    end
 
    if A.restart_seen && real_slow_k && ~first_restart_during_slow_found
        first_restart_during_slow_found = true;
        first_during_slow_time = t_current;
    end
 
    if A.restart_seen && real_stationary_k && ~first_restart_during_stationary_found
        first_restart_during_stationary_found = true;
        first_during_stationary_time = t_current;
    end
 
    % Primo istante assoluto della scansione in cui la predizione vede restart.
    % Può essere prima dell'arrivo, durante il dwell reale, o dopo.
    if A.has_stop && A.restart_seen && ~first_restart_seen_found
 
        first_restart_seen_found = true;
        first_restart_k_pred = k_pred;
        first_restart_real_status = real_status_text;
 
        first_restart_A = local_analyze_prediction_stop( ...
            s_pred, v_pred, a_pred, ...
            t_current, stop_to_analyze, ...
            tau, node_index, ...
            near_stop_tol_pred, v_stop_threshold, restart_threshold, ...
            true);
 
        first_restart_T_nodes = first_restart_A.T_nodes;
 
        fprintf('\n>>> PRIMA RIPARTENZA VISTA TROVATA <<<\n');
        fprintf('Istante scansione numero: %d\n', ss);
        fprintf('t_current = %.2f s\n', t_current);
        fprintf('offset rispetto arrivo reale = %.2f s\n', offset_real);
        fprintf('stato reale corrente: %s\n', real_status_text);
        fprintf('stato reale: pos = %.3f m, vel = %.3f m/s, acc = %.3f m/s^2\n', ...
            real_pos_k, real_vel_k, real_acc_k);
        fprintf('first_stop_tau = %.2f s\n', first_restart_A.first_stop_tau);
        fprintf('last_stop_tau  = %.2f s\n', first_restart_A.last_stop_tau);
        fprintf('dwell predetto circa = %.2f s\n', first_restart_A.dwell_duration);
        fprintf('restart_tau = %.2f s\n', first_restart_A.restart_tau);
        fprintf('restart_abs_time = %.2f s\n', first_restart_A.restart_abs_time);
        fprintf('restart_pos = %.3f m\n', first_restart_A.restart_pos);
        fprintf('max_vel_after_stop = %.3f m/s\n', first_restart_A.max_vel_after_stop);
 
        % NON faccio break: continuo comunque la scansione per avere
        % la tabella completa e per vedere se il segnale restart_seen è intermittente.
    end
 
end
 
T_restart_scan = array2table(ScanRows, ...
    'VariableNames', { ...
    'scan_id', ...
    't_current_s', ...
    'offset_from_arrival_s', ...
    'real_pos_m', ...
    'real_vel_ms', ...
    'real_acc_ms2', ...
    'real_status_code', ...
    'pred_min_dist_stop_m', ...
    'pred_stopped_steps', ...
    'pred_first_stop_tau_s', ...
    'pred_last_stop_tau_s', ...
    'pred_dwell_duration_s', ...
    'pred_restart_seen', ...
    'pred_restart_tau_s', ...
    'pred_restart_abs_time_s', ...
    'pred_restart_pos_m', ...
    'pred_max_vel_after_stop_ms'});
 
% Aggiungo colonna testuale dello stato reale
real_status_text_col = cell(height(T_restart_scan),1);
 
for r = 1:height(T_restart_scan)
    switch T_restart_scan.real_status_code(r)
        case 0
            real_status_text_col{r} = 'BEFORE_ARRIVAL';
        case 1
            real_status_text_col{r} = 'NEAR_STOP_NOT_SLOW';
        case 2
            real_status_text_col{r} = 'SLOW_AT_STOP';
        case 3
            real_status_text_col{r} = 'STATIONARY_AT_STOP';
        otherwise
            real_status_text_col{r} = 'AFTER_OR_AWAY_FROM_STOP';
    end
end
 
T_restart_scan.real_status = real_status_text_col;
 
assignin('base','T_restart_scan',T_restart_scan);
 
fprintf('\n============================================================\n');
fprintf('TABELLA COMPLETA SCANSIONE RESTART\n');
fprintf('============================================================\n');
 
disp(T_restart_scan(:, { ...
    't_current_s', ...
    'offset_from_arrival_s', ...
    'real_pos_m', ...
    'real_vel_ms', ...
    'real_status', ...
    'pred_first_stop_tau_s', ...
    'pred_last_stop_tau_s', ...
    'pred_dwell_duration_s', ...
    'pred_restart_seen', ...
    'pred_restart_tau_s', ...
    'pred_restart_abs_time_s', ...
    'pred_restart_pos_m'}));
 
if save_csv
    writetable(T_restart_scan, fullfile(csv_folder, 'restart_scan_full.csv'));
end
 
%% =======================
% Risultato principale
% =======================
 
fprintf('\n============================================================\n');
fprintf('RISULTATO PRINCIPALE\n');
fprintf('============================================================\n');
 
if first_restart_seen_found
 
    t_first = t_pred(first_restart_k_pred);
 
    fprintf('La predizione vede per la prima volta la ripartenza a:\n');
    fprintf('t_current = %.2f s\n', t_first);
    fprintf('offset rispetto arrivo reale = %.2f s\n', t_first - t_arrival);
    fprintf('stato reale in quell''istante = %s\n', first_restart_real_status);
 
    fprintf('\nDentro l''orizzonte predittivo in quell''istante:\n');
    fprintf('FIRST_STOP tau = %.2f s\n', first_restart_A.first_stop_tau);
    fprintf('LAST_STOP  tau = %.2f s\n', first_restart_A.last_stop_tau);
    fprintf('RESTART    tau = %.2f s\n', first_restart_A.restart_tau);
 
    fprintf('\nTempi assoluti previsti:\n');
    fprintf('FIRST_STOP abs time = %.2f s\n', t_first + first_restart_A.first_stop_tau);
    fprintf('LAST_STOP  abs time = %.2f s\n', t_first + first_restart_A.last_stop_tau);
    fprintf('RESTART    abs time = %.2f s\n', first_restart_A.restart_abs_time);
 
    fprintf('\nInterpretazione:\n');
    if t_first < t_arrival
        fprintf('La ripartenza viene vista PRIMA dell''arrivo reale alla fermata.\n');
    else
        fprintf('La ripartenza viene vista quando il veicolo è già arrivato o è già fermo alla fermata.\n');
    end
 
    if print_first_restart_nodes
        fprintf('\n============================================================\n');
        fprintf('TABELLA COMPLETA DEI 50 NODI AL PRIMO ISTANTE CON RESTART\n');
        fprintf('============================================================\n');
        disp(first_restart_T_nodes);
    end
 
    assignin('base','T_first_restart_pred_nodes',first_restart_T_nodes);
    assignin('base','first_restart_A',first_restart_A);
 
    if save_csv
        writetable(first_restart_T_nodes, fullfile(csv_folder, 'first_restart_pred_nodes.csv'));
    end
 
else
 
    fprintf('ATTENZIONE: in nessun istante della finestra analizzata la predizione vede la ripartenza.\n');
    fprintf('La fermata appare come vincolo attivo fino a fine orizzonte.\n');
 
end
 
%% =======================
% Riepilogo: restart prima/durante fermata
% =======================
 
fprintf('\n============================================================\n');
fprintf('RIEPILOGO QUANDO VIENE VISTA LA RIPARTENZA\n');
fprintf('============================================================\n');
 
if first_restart_before_arrival_found
    fprintf('Prima dell''arrivo reale: SI, primo istante t = %.2f s\n', first_before_arrival_time);
else
    fprintf('Prima dell''arrivo reale: NO\n');
end
 
if first_restart_during_slow_found
    fprintf('Durante stato slow-at-stop: SI, primo istante t = %.2f s\n', first_during_slow_time);
else
    fprintf('Durante stato slow-at-stop: NO\n');
end
 
if first_restart_during_stationary_found
    fprintf('Durante stato stationary-at-stop: SI, primo istante t = %.2f s\n', first_during_stationary_time);
else
    fprintf('Durante stato stationary-at-stop: NO\n');
end
 
%% =======================
% Riepilogo istanti reali effettivi alla fermata
% =======================
 
fprintf('\n============================================================\n');
fprintf('ISTANTI REALI IN CUI IL VEICOLO RIMANE FERMO ALLA FERMATA\n');
fprintf('============================================================\n');
 
if height(T_real_stationary_samples) == 0
 
    fprintf('Nessun campione reale con criterio stationary trovato:\n');
    fprintf('|pos - stop| <= %.2f m, |vel| <= %.2f m/s\n', ...
        real_near_stop_tol, real_stationary_threshold);
 
    fprintf('\nUso allora il criterio piu'' largo slow-at-stop:\n');
    fprintf('|pos - stop| <= %.2f m, |vel| <= %.2f m/s\n', ...
        real_near_stop_tol, real_slow_at_stop_threshold);
 
    if height(T_real_slow_samples) == 0
        fprintf('Nessun campione trovato nemmeno con criterio slow-at-stop.\n');
    else
        local_disp_table_limited(T_real_slow_samples, 40);
    end
 
else
 
    fprintf('Criterio stationary:\n');
    fprintf('|pos - stop| <= %.2f m, |vel| <= %.2f m/s\n\n', ...
        real_near_stop_tol, real_stationary_threshold);
 
    local_disp_table_limited(T_real_stationary_samples, 40);
 
end
 
%% =======================
% Plot 1: posizione reale
% =======================
 
figure;
hold on;
grid on;
title(sprintf('Real position near stop - vehicle %d, stop %.1f m', veh_id, stop_to_analyze));
xlabel('time [s]');
ylabel('position [m]');
plot(t_real, pos_real, 'LineWidth', 1.2);
yline(stop_to_analyze, 'k--', 'stop');
 
xlim([max(t_real(1), t_arrival-40), min(t_real(end), t_arrival+80)]);
 
if first_restart_seen_found
    xline(t_pred(first_restart_k_pred), '--', ...
        sprintf('first restart seen t=%.1f', t_pred(first_restart_k_pred)));
end
 
%% =======================
% Plot 2: velocità reale
% =======================
 
figure;
hold on;
grid on;
title(sprintf('Real velocity near stop - vehicle %d, stop %.1f m', veh_id, stop_to_analyze));
xlabel('time [s]');
ylabel('velocity [m/s]');
plot(t_real, vel_real, 'LineWidth', 1.2);
yline(0, 'k--', 'v=0');
yline(real_stationary_threshold, '--', 'stationary threshold');
yline(real_slow_at_stop_threshold, '--', 'slow threshold');
 
xlim([max(t_real(1), t_arrival-40), min(t_real(end), t_arrival+80)]);
 
if first_restart_seen_found
    xline(t_pred(first_restart_k_pred), '--', ...
        sprintf('first restart seen t=%.1f', t_pred(first_restart_k_pred)));
end
 
%% =======================
% Plot 3: restart_seen durante tutta la scansione
% =======================
 
figure;
hold on;
grid on;
title(sprintf('Restart seen during scan - vehicle %d, stop %.1f m', veh_id, stop_to_analyze));
xlabel('current prediction time [s]');
ylabel('restart seen');
ylim([-0.1 1.1]);
 
stairs(T_restart_scan.t_current_s, T_restart_scan.pred_restart_seen, 'LineWidth', 1.5);
yline(0, 'k--');
yline(1, 'k--');
xline(t_arrival, 'k:', 'real arrival');
 
if first_restart_seen_found
    xline(t_pred(first_restart_k_pred), '--', ...
        sprintf('first restart seen t=%.1f', t_pred(first_restart_k_pred)));
end
 
%% =======================
% Plot 4: predizione al primo istante con restart
% =======================
 
if first_restart_seen_found
 
    Tn = first_restart_T_nodes;
 
    figure;
    hold on;
    grid on;
    title(sprintf('Predicted position at first restart seen - vehicle %d, stop %.1f m', veh_id, stop_to_analyze));
    xlabel('tau inside horizon [s]');
    ylabel('s predicted [m]');
    plot(Tn.tau_s, Tn.s_pred_m, 'LineWidth', 1.5);
    yline(stop_to_analyze, 'k--', 'stop');
    xline(first_restart_A.first_stop_tau, '--', 'FIRST STOP');
    xline(first_restart_A.last_stop_tau, '--', 'LAST STOP');
    xline(first_restart_A.restart_tau, '--', 'RESTART');
 
    figure;
    hold on;
    grid on;
    title(sprintf('Predicted velocity at first restart seen - vehicle %d, stop %.1f m', veh_id, stop_to_analyze));
    xlabel('tau inside horizon [s]');
    ylabel('v predicted [m/s]');
    plot(Tn.tau_s, Tn.v_pred_ms, 'LineWidth', 1.5);
    yline(0, 'k--', 'v=0');
    yline(restart_threshold, '--', sprintf('restart threshold %.2f', restart_threshold));
    xline(first_restart_A.first_stop_tau, '--', 'FIRST STOP');
    xline(first_restart_A.last_stop_tau, '--', 'LAST STOP');
    xline(first_restart_A.restart_tau, '--', 'RESTART');
 
end
 
%% =======================
% Salvataggi finali
% =======================
 
assignin('base','T_restart_scan',T_restart_scan);
assignin('base','T_real_slow_samples',T_real_slow_samples);
assignin('base','T_real_stationary_samples',T_real_stationary_samples);
assignin('base','T_real_slow_intervals',T_real_slow_intervals);
assignin('base','T_real_stationary_intervals',T_real_stationary_intervals);
 
fprintf('\n============================================================\n');
fprintf('OUTPUT SALVATI NEL WORKSPACE\n');
fprintf('============================================================\n');
fprintf('- T_restart_scan\n');
fprintf('- T_first_restart_pred_nodes, se trovato restart\n');
fprintf('- first_restart_A, se trovato restart\n');
fprintf('- T_real_slow_samples\n');
fprintf('- T_real_stationary_samples\n');
fprintf('- T_real_slow_intervals\n');
fprintf('- T_real_stationary_intervals\n');
 
if save_csv
    fprintf('\nCSV salvati nella cartella: %s\n', csv_folder);
end
 
%% ============================================================
% FUNZIONI LOCALI
% ============================================================
 
function [t, data] = local_get_signal(out, fieldname)
 
    sig = out.(fieldname);
 
    if isa(sig,'timeseries')
        t = sig.Time(:);
        data = sig.Data;
 
    elseif isstruct(sig)
        t = sig.time(:);
        data = sig.signals.values;
 
    else
        error('Formato non riconosciuto per out.%s', fieldname);
    end
 
end
 
function [t_pred, Xpred_raw, pred_name] = local_find_prediction_signal(out, t_real)
 
    candidate_names = { ...
        'state_pred', ...
        'statePred', ...
        'states_pred', ...
        'state_prediction', ...
        'x_pred', ...
        'xPred', ...
        'X_pred', ...
        'Xpred', ...
        'X', ...
        'acado_X', ...
        'state_pred_old'};
 
    for i = 1:length(candidate_names)
 
        name = candidate_names{i};
 
        try
            sig = out.(name);
            has_signal = true;
        catch
            has_signal = false;
        end
 
        if has_signal
 
            if isa(sig,'timeseries')
 
                Xpred_raw = sig.Data;
                t_pred = sig.Time(:);
                pred_name = name;
                return;
 
            elseif isstruct(sig)
 
                Xpred_raw = sig.signals.values;
 
                if isfield(sig,'time')
                    t_pred = sig.time(:);
                else
                    t_pred = local_infer_prediction_time(Xpred_raw, t_real);
                end
 
                pred_name = name;
                return;
 
            else
 
                Xpred_raw = sig;
                t_pred = local_infer_prediction_time(Xpred_raw, t_real);
                pred_name = name;
                return;
 
            end
        end
    end
 
    error('Non trovo la traiettoria predetta. Devi loggare out.state_pred.');
 
end
 
function t_pred = local_infer_prediction_time(Xpred_raw, t_real)
 
    Xtmp = squeeze(Xpred_raw);
    dims = size(Xtmp);
 
    if ndims(Xtmp) == 3
        nt_pred = dims(3);
    elseif ismatrix(Xtmp)
        nt_pred = dims(1);
    else
        nt_pred = length(t_real);
    end
 
    if nt_pred <= 1
        t_pred = t_real(1);
    else
        t_pred = linspace(t_real(1), t_real(end), nt_pred).';
    end
 
end
 
function Xpred = local_format_prediction(Xraw, nt_pred, N, nx)
 
    Xraw = squeeze(Xraw);
 
    Nh = N + 1;
 
    dims = size(Xraw);
 
    fprintf('\nDimensione grezza predizione:\n');
    disp(dims);
 
    % Caso 1: [Nh x nx x nt_pred]
    if ndims(Xraw) == 3 && size(Xraw,1) == Nh && size(Xraw,2) == nx
        Xpred = permute(Xraw, [3 1 2]);
        return;
    end
 
    % Caso 2: [nt_pred x Nh x nx]
    if ndims(Xraw) == 3 && size(Xraw,1) == nt_pred && size(Xraw,2) == Nh && size(Xraw,3) == nx
        Xpred = Xraw;
        return;
    end
 
    % Caso 3: [nx x Nh x nt_pred]
    if ndims(Xraw) == 3 && size(Xraw,1) == nx && size(Xraw,2) == Nh
        Xpred = permute(Xraw, [3 2 1]);
        return;
    end
 
    % Caso 4: [nt_pred x Nh*nx]
    if ismatrix(Xraw) && size(Xraw,1) == nt_pred && size(Xraw,2) == Nh*nx
        Xtmp = Xraw;
        Xpred = reshape(Xtmp, [nt_pred, nx, Nh]);
        Xpred = permute(Xpred, [1 3 2]);
        return;
    end
 
    % Caso 5: [Nh*nx x nt_pred]
    if ismatrix(Xraw) && size(Xraw,2) == nt_pred && size(Xraw,1) == Nh*nx
        Xtmp = Xraw.';
        Xpred = reshape(Xtmp, [nt_pred, nx, Nh]);
        Xpred = permute(Xpred, [1 3 2]);
        return;
    end
 
    error(['Formato predizione non riconosciuto. Trovato: ', mat2str(size(Xraw))]);
 
end
 
function A = local_analyze_prediction_stop( ...
    s_pred, v_pred, a_pred, ...
    t_current, stop_to_analyze, ...
    tau, node_index, ...
    near_stop_tol_pred, v_stop_threshold, restart_threshold, ...
    build_table)
 
    Np = length(s_pred);
 
    abs_time = t_current + tau;
    dist_to_stop = stop_to_analyze - s_pred;
 
    near_stop = abs(s_pred - stop_to_analyze) <= near_stop_tol_pred;
    stopped_near_stop = near_stop & abs(v_pred) <= v_stop_threshold;
 
    passed_stop = s_pred > stop_to_analyze + near_stop_tol_pred;
    before_stop = s_pred < stop_to_analyze - near_stop_tol_pred;
 
    idx_first_stop = find(stopped_near_stop, 1, 'first');
    idx_last_stop  = find(stopped_near_stop, 1, 'last');
 
    if isempty(idx_last_stop)
        idx_restart = find(v_pred > restart_threshold & tau > 0, 1, 'first');
    else
        idx_restart = find(v_pred > restart_threshold & node_index > node_index(idx_last_stop), 1, 'first');
    end
 
    if isempty(idx_first_stop)
        first_stop_tau = NaN;
        last_stop_tau = NaN;
        dwell_duration = 0;
    else
        first_stop_tau = tau(idx_first_stop);
        last_stop_tau = tau(idx_last_stop);
        dwell_duration = last_stop_tau - first_stop_tau;
    end
 
    if isempty(idx_restart)
        restart_tau = NaN;
        restart_abs_time = NaN;
        restart_pos = NaN;
    else
        restart_tau = tau(idx_restart);
        restart_abs_time = abs_time(idx_restart);
        restart_pos = s_pred(idx_restart);
    end
 
    restart_seen = isfinite(restart_tau);
    has_stop = ~isempty(idx_first_stop);
 
    n_stopped_steps = sum(stopped_near_stop);
 
    if isempty(idx_last_stop) || idx_last_stop >= length(v_pred)
        max_vel_after_stop = NaN;
    else
        max_vel_after_stop = max(v_pred(idx_last_stop+1:end));
    end
 
    pred_min_dist_stop = min(abs(s_pred - stop_to_analyze));
 
    A = struct();
 
    A.has_stop = has_stop;
    A.restart_seen = restart_seen;
 
    A.idx_first_stop = idx_first_stop;
    A.idx_last_stop = idx_last_stop;
    A.idx_restart = idx_restart;
 
    A.first_stop_tau = first_stop_tau;
    A.last_stop_tau = last_stop_tau;
    A.dwell_duration = dwell_duration;
 
    A.restart_tau = restart_tau;
    A.restart_abs_time = restart_abs_time;
    A.restart_pos = restart_pos;
 
    A.n_stopped_steps = n_stopped_steps;
    A.max_vel_after_stop = max_vel_after_stop;
    A.pred_min_dist_stop = pred_min_dist_stop;
 
    A.T_nodes = table();
 
    if ~build_table
        return;
    end
 
    phase = cell(Np,1);
    event = cell(Np,1);
 
    for j = 1:Np
 
        event{j} = '';
 
        if before_stop(j)
 
            phase{j} = 'APPROACH';
 
        elseif near_stop(j) && abs(v_pred(j)) <= v_stop_threshold
 
            phase{j} = 'DWELL_OR_STOPPED_AT_STOP';
 
        elseif near_stop(j) && v_pred(j) > restart_threshold
 
            phase{j} = 'RESTART_NEAR_STOP';
 
        elseif near_stop(j) && v_pred(j) > v_stop_threshold && v_pred(j) <= restart_threshold
 
            phase{j} = 'CREEPING_NEAR_STOP';
 
        elseif passed_stop(j) && v_pred(j) > restart_threshold
 
            phase{j} = 'AFTER_STOP_MOVING';
 
        elseif passed_stop(j) && abs(v_pred(j)) <= v_stop_threshold
 
            phase{j} = 'PAST_STOP_BUT_STOPPED';
 
        else
 
            phase{j} = 'OTHER';
 
        end
    end
 
    if ~isempty(idx_first_stop)
        event{idx_first_stop} = 'FIRST_STOP';
        event{idx_last_stop}  = 'LAST_STOP';
    end
 
    if ~isempty(idx_restart)
        if isempty(event{idx_restart})
            event{idx_restart} = 'RESTART';
        else
            event{idx_restart} = [event{idx_restart}, '+RESTART'];
        end
    end
 
    A.T_nodes = table( ...
        node_index, ...
        tau, ...
        abs_time, ...
        s_pred, ...
        v_pred, ...
        a_pred, ...
        dist_to_stop, ...
        near_stop, ...
        stopped_near_stop, ...
        phase, ...
        event, ...
        'VariableNames', { ...
        'node', ...
        'tau_s', ...
        'abs_time_s', ...
        's_pred_m', ...
        'v_pred_ms', ...
        'a_pred_ms2', ...
        'dist_to_stop_m', ...
        'near_stop', ...
        'stopped_near_stop', ...
        'phase', ...
        'event'});
 
end
 
function T_intervals = local_mask_to_intervals(mask, t, pos, vel, acc)
 
    mask = mask(:);
 
    d = diff([false; mask; false]);
 
    idx_start = find(d == 1);
    idx_end   = find(d == -1) - 1;
 
    n = length(idx_start);
 
    if n == 0
        T_intervals = table();
        return;
    end
 
    interval_id = (1:n).';
    t_start = zeros(n,1);
    t_end = zeros(n,1);
    duration = zeros(n,1);
    pos_start = zeros(n,1);
    pos_end = zeros(n,1);
    pos_min = zeros(n,1);
    pos_max = zeros(n,1);
    vel_max_abs = zeros(n,1);
    acc_max_abs = zeros(n,1);
    n_samples = zeros(n,1);
 
    for i = 1:n
 
        idx = idx_start(i):idx_end(i);
 
        t_start(i) = t(idx_start(i));
        t_end(i) = t(idx_end(i));
        duration(i) = t_end(i) - t_start(i);
 
        pos_start(i) = pos(idx_start(i));
        pos_end(i) = pos(idx_end(i));
        pos_min(i) = min(pos(idx));
        pos_max(i) = max(pos(idx));
 
        vel_max_abs(i) = max(abs(vel(idx)));
        acc_max_abs(i) = max(abs(acc(idx)));
 
        n_samples(i) = length(idx);
 
    end
 
    T_intervals = table( ...
        interval_id, ...
        t_start, ...
        t_end, ...
        duration, ...
        n_samples, ...
        pos_start, ...
        pos_end, ...
        pos_min, ...
        pos_max, ...
        vel_max_abs, ...
        acc_max_abs, ...
        'VariableNames', { ...
        'interval_id', ...
        't_start_s', ...
        't_end_s', ...
        'duration_s', ...
        'n_samples', ...
        'pos_start_m', ...
        'pos_end_m', ...
        'pos_min_m', ...
        'pos_max_m', ...
        'vel_max_abs_ms', ...
        'acc_max_abs_ms2'});
 
end
 
function local_disp_table_limited(T, max_rows)
 
    if height(T) == 0
        fprintf('Tabella vuota.\n');
        return;
    end
 
    n = height(T);
 
    if n <= max_rows
        disp(T);
    else
        n_head = floor(max_rows/2);
        n_tail = max_rows - n_head;
 
        fprintf('La tabella ha %d righe. Mostro prime %d e ultime %d righe.\n', ...
            n, n_head, n_tail);
 
        disp(T(1:n_head,:));
        fprintf('...\n');
        disp(T(n-n_tail+1:n,:));
    end
 
end
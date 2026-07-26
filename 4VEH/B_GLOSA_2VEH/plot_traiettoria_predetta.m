%%=========================================================================
% STAMPA ORIZZONTE DI PREDIZIONE A UN ISTANTE SCELTO
%=========================================================================
% Richiede nel workspace:
%   out.state_pred
%   out.ctrl_pred          opzionale
%   out.online_data        opzionale
%
% Stato:
%   [pos1 vel1 acc1 pos2 vel2 acc2 pos3 vel3 acc3]
%
% Controlli:
%   default:
%   U = [jerk1 jerk2 jerk3 sJmin1 sJmin2 sJmin3]
%
% OnlineData:
%   63 elementi = 21 per veicolo
%==========================================================================
 
clc;
 
%%=========================================================================
% PARAMETRI DA SCEGLIERE
%==========================================================================
 
t_query = 403;       % [s] istante da analizzare
veh_id  = 1;         % 1, 2 oppure 3
 
nx  = 9;
nu  = 6;
nod = 63;
 
% Se nel tuo ctrl_pred l'ordine fosse diverso, cambia qui.
% Layout standard assunto:
% U = [jerk1 jerk2 jerk3 sJmin1 sJmin2 sJmin3]
ctrl_layout = 1;
 
%%=========================================================================
% LETTURA INDICE PIU' VICINO
%==========================================================================
 
[t_pred, ~] = get_logged_time_and_values(out.state_pred);
 
[~, idx_k] = min(abs(t_pred - t_query));
t_k = t_pred(idx_k);
 
fprintf('\n============================================================\n');
fprintf('ORIZZONTE DI PREDIZIONE\n');
fprintf('============================================================\n');
fprintf('Istante richiesto        : %.3f s\n', t_query);
fprintf('Istante disponibile      : %.3f s\n', t_k);
fprintf('Indice campione          : %d\n', idx_k);
fprintf('Veicolo analizzato       : %d\n', veh_id);
fprintf('============================================================\n\n');
 
%%=========================================================================
% ESTRAZIONE STATE PREDICTION
%==========================================================================
 
Xraw = sample_logged_signal_by_index(out.state_pred, idx_k);
Xpred = format_state_prediction(Xraw, nx);
 
N_nodes = size(Xpred,1);
N = N_nodes - 1;
 
tau = get_prediction_time_vector(N_nodes);
t_abs = t_k + tau;
 
[pos_col, vel_col, acc_col] = vehicle_columns(veh_id);
 
pos = Xpred(:,pos_col);
vel = Xpred(:,vel_col);
acc = Xpred(:,acc_col);
 
%%=========================================================================
% ESTRAZIONE CTRL PREDICTION
%==========================================================================
 
jerk = nan(N_nodes,1);
sJmin = nan(N_nodes,1);
 
has_ctrl = isfield(out,'ctrl_pred');
 
if has_ctrl
 
    try
        Uraw = sample_logged_signal_by_index(out.ctrl_pred, idx_k);
        Upred = format_control_prediction(Uraw, nu);
 
        [jerk_col, sJmin_col] = control_columns(veh_id, ctrl_layout);
 
        nU = size(Upred,1);
 
        jerk(1:nU) = Upred(:,jerk_col);
        sJmin(1:nU) = Upred(:,sJmin_col);
 
        % ultimo nodo: non esiste controllo, lascio NaN
 
    catch ME
        warning('ctrl_pred non leggibile: %s', ME.message);
    end
 
else
    warning('out.ctrl_pred non disponibile.');
end
 
%%=========================================================================
% ESTRAZIONE ONLINE DATA
%==========================================================================
 
s_stop_active = nan(N_nodes,1);
x_stop        = nan(N_nodes,1);
x_dwell       = nan(N_nodes,1);
w_stop        = nan(N_nodes,1);
 
s_TL1         = nan(N_nodes,1);
s_TL2         = nan(N_nodes,1);
s_TL3         = nan(N_nodes,1);
s_TL4         = nan(N_nodes,1);
 
x_TL1         = nan(N_nodes,1);
x_TL2         = nan(N_nodes,1);
x_TL3         = nan(N_nodes,1);
x_TL4         = nan(N_nodes,1);
 
x_TLtail1     = nan(N_nodes,1);
x_TLtail2     = nan(N_nodes,1);
x_TLtail3     = nan(N_nodes,1);
x_TLtail4     = nan(N_nodes,1);
 
has_od = isfield(out,'online_data');
 
if has_od
 
    try
        ODraw = sample_logged_signal_by_index(out.online_data, idx_k);
        ODpred = format_online_data(ODraw, nod);
 
        D = parse_vehicle_online_data(ODpred, veh_id);
 
        nOD = min(size(ODpred,1), N_nodes);
 
        s_stop_active(1:nOD) = D.s_stop(1:nOD);
        x_stop(1:nOD)        = D.x_stop(1:nOD);
        x_dwell(1:nOD)       = D.x_dwell(1:nOD);
        w_stop(1:nOD)        = D.w_stop(1:nOD);
 
        s_TL1(1:nOD) = D.s_TL_active(1:nOD,1);
        s_TL2(1:nOD) = D.s_TL_active(1:nOD,2);
        s_TL3(1:nOD) = D.s_TL_active(1:nOD,3);
        s_TL4(1:nOD) = D.s_TL_active(1:nOD,4);
 
        x_TL1(1:nOD) = D.x_TL(1:nOD,1);
        x_TL2(1:nOD) = D.x_TL(1:nOD,2);
        x_TL3(1:nOD) = D.x_TL(1:nOD,3);
        x_TL4(1:nOD) = D.x_TL(1:nOD,4);
 
        x_TLtail1(1:nOD) = D.x_TL_tail(1:nOD,1);
        x_TLtail2(1:nOD) = D.x_TL_tail(1:nOD,2);
        x_TLtail3(1:nOD) = D.x_TL_tail(1:nOD,3);
        x_TLtail4(1:nOD) = D.x_TL_tail(1:nOD,4);
 
    catch ME
        warning('online_data non leggibile: %s', ME.message);
    end
 
else
    warning('out.online_data non disponibile.');
end
 
%%=========================================================================
% TABELLA ORIZZONTE COMPLETA
%==========================================================================
 
node = (0:N).';
 
T_horizon = table( ...
    node, ...
    tau, ...
    t_abs, ...
    pos, ...
    vel, ...
    acc, ...
    jerk, ...
    sJmin, ...
    s_stop_active, ...
    x_stop, ...
    x_dwell, ...
    w_stop, ...
    s_TL1, x_TL1, x_TLtail1, ...
    s_TL2, x_TL2, x_TLtail2, ...
    s_TL3, x_TL3, x_TLtail3, ...
    s_TL4, x_TL4, x_TLtail4, ...
    'VariableNames', { ...
    'node', ...
    'tau', ...
    'time_abs', ...
    'pos', ...
    'vel', ...
    'acc', ...
    'jerk', ...
    'sJmin', ...
    's_stop_active', ...
    'x_stop', ...
    'x_dwell', ...
    'w_stop', ...
    's_TL1','x_TL1','x_TLtail1', ...
    's_TL2','x_TL2','x_TLtail2', ...
    's_TL3','x_TL3','x_TLtail3', ...
    's_TL4','x_TL4','x_TLtail4'} );
 
disp(T_horizon);
 
%%=========================================================================
% STAMPA RIASSUNTO
%==========================================================================
 
fprintf('\n============================================================\n');
fprintf('RIASSUNTO ORIZZONTE\n');
fprintf('============================================================\n');
fprintf('Numero nodi prediction  : %d\n', N_nodes);
fprintf('Numero intervalli       : %d\n', N);
fprintf('Tempo finale prediction : %.3f s\n', t_abs(end));
fprintf('Durata orizzonte        : %.3f s\n', tau(end));
fprintf('\n');
 
fprintf('Nodo iniziale:\n');
fprintf('  pos = %.3f m, vel = %.3f m/s, acc = %.3f m/s^2\n', ...
    pos(1), vel(1), acc(1));
 
fprintf('Nodo finale:\n');
fprintf('  pos = %.3f m, vel = %.3f m/s, acc = %.3f m/s^2\n', ...
    pos(end), vel(end), acc(end));
 
if has_ctrl
    fprintf('\nControllo nodo 0:\n');
    fprintf('  jerk = %.6f, sJmin = %.6f\n', jerk(1), sJmin(1));
end
 
if has_od
    fprintf('\nOnlineData nodo 0:\n');
    fprintf('  s_stop_active = %.3f, x_stop = %.0f, x_dwell = %.0f, w_stop = %.3f\n', ...
        s_stop_active(1), x_stop(1), x_dwell(1), w_stop(1));
 
    fprintf('  TL1: s = %.3f, x_head = %.0f, x_tail = %.0f\n', ...
        s_TL1(1), x_TL1(1), x_TLtail1(1));
    fprintf('  TL2: s = %.3f, x_head = %.0f, x_tail = %.0f\n', ...
        s_TL2(1), x_TL2(1), x_TLtail2(1));
    fprintf('  TL3: s = %.3f, x_head = %.0f, x_tail = %.0f\n', ...
        s_TL3(1), x_TL3(1), x_TLtail3(1));
    fprintf('  TL4: s = %.3f, x_head = %.0f, x_tail = %.0f\n', ...
        s_TL4(1), x_TL4(1), x_TLtail4(1));
end
 
%%=========================================================================
% PLOT RAPIDO
%==========================================================================
 
figure;
plot(t_abs, pos, 'o-', 'LineWidth', 1.2);
grid on;
xlabel('Tempo assoluto [s]');
ylabel('Posizione predetta [m]');
title(sprintf('Veicolo %d - posizione predetta a t = %.1f s', veh_id, t_k));
 
figure;
plot(t_abs, vel, 'o-', 'LineWidth', 1.2);
grid on;
xlabel('Tempo assoluto [s]');
ylabel('Velocità predetta [m/s]');
title(sprintf('Veicolo %d - velocità predetta a t = %.1f s', veh_id, t_k));
 
figure;
plot(t_abs, acc, 'o-', 'LineWidth', 1.2);
grid on;
xlabel('Tempo assoluto [s]');
ylabel('Accelerazione predetta [m/s^2]');
title(sprintf('Veicolo %d - accelerazione predetta a t = %.1f s', veh_id, t_k));
 
if has_ctrl
    figure;
    stairs(t_abs, jerk, 'LineWidth', 1.2);
    grid on;
    xlabel('Tempo assoluto [s]');
    ylabel('Jerk applicato/predetto [m/s^3]');
    title(sprintf('Veicolo %d - jerk predetto a t = %.1f s', veh_id, t_k));
end
 
%%=========================================================================
% FUNZIONI LOCALI
%==========================================================================
 
function [pos_col, vel_col, acc_col] = vehicle_columns(veh_id)
 
    if veh_id == 1
        pos_col = 1;
        vel_col = 2;
        acc_col = 3;
    elseif veh_id == 2
        pos_col = 4;
        vel_col = 5;
        acc_col = 6;
    elseif veh_id == 3
        pos_col = 7;
        vel_col = 8;
        acc_col = 9;
    else
        error('veh_id deve essere 1, 2 oppure 3.');
    end
 
end
 
function [jerk_col, sJmin_col] = control_columns(veh_id, ctrl_layout)
 
    if ctrl_layout == 1
        % U = [jerk1 jerk2 jerk3 sJmin1 sJmin2 sJmin3]
        jerk_col  = veh_id;
        sJmin_col = veh_id + 3;
    elseif ctrl_layout == 2
        % U = [jerk1 sJmin1 jerk2 sJmin2 jerk3 sJmin3]
        jerk_col  = 2*veh_id - 1;
        sJmin_col = 2*veh_id;
    else
        error('ctrl_layout non riconosciuto.');
    end
 
end
 
function [t, values] = get_logged_time_and_values(logsig)
 
    if isa(logsig,'timeseries')
        t = logsig.Time(:);
        values = logsig.Data;
        return;
    end
 
    if isstruct(logsig)
        if isfield(logsig,'time') && isfield(logsig,'signals')
            t = logsig.time(:);
            values = logsig.signals.values;
            return;
        end
 
        if isfield(logsig,'Time') && isfield(logsig,'Data')
            t = logsig.Time(:);
            values = logsig.Data;
            return;
        end
    end
 
    if isnumeric(logsig)
        values = logsig;
        t = (0:size(values,1)-1).';
        return;
    end
 
    error('Formato log non riconosciuto.');
 
end
 
function sample = sample_logged_signal_by_index(logsig, idx)
 
    [t, values] = get_logged_time_and_values(logsig);
 
    nt = numel(t);
    dims = size(values);
 
    if isvector(values)
        v = values(:);
 
        if numel(v) == nt
            sample = v(idx);
        else
            sample = v;
        end
 
        return;
    end
 
    time_dim = find(dims == nt, 1, 'first');
 
    if isempty(time_dim)
        error('Impossibile identificare la dimensione temporale del segnale.');
    end
 
    subs = repmat({':'},1,ndims(values));
    subs{time_dim} = idx;
 
    sample = squeeze(values(subs{:}));
 
end
 
function X = format_state_prediction(raw, nx)
 
    A = squeeze(raw);
 
    if ismatrix(A)
 
        if size(A,2) == nx
            X = A;
            return;
        end
 
        if size(A,1) == nx
            X = A.';
            return;
        end
 
    end
 
    v = A(:);
 
    if mod(numel(v),nx) ~= 0
        error('state_pred non compatibile con nx = %d. Num elementi = %d.', nx, numel(v));
    end
 
    n_nodes = numel(v)/nx;
 
    X1 = reshape(v,nx,n_nodes).';
    X2 = reshape(v,n_nodes,nx);
 
    if state_prediction_score(X1) <= state_prediction_score(X2)
        X = X1;
    else
        X = X2;
    end
 
end
 
function score = state_prediction_score(X)
 
    score = 0;
 
    if size(X,2) < 9
        score = 1e12;
        return;
    end
 
    vel_cols = [2 5 8];
    acc_cols = [3 6 9];
    pos_cols = [1 4 7];
 
    score = score + 1e5*sum(~isfinite(X(:)));
 
    for k = 1:numel(vel_cols)
        score = score + 100*sum(abs(X(:,vel_cols(k))) > 80);
        score = score + 100*sum(abs(X(:,acc_cols(k))) > 20);
        score = score + 10*sum(diff(X(:,pos_cols(k))) < -5);
    end
 
end
 
function U = format_control_prediction(raw, nu)
 
    A = squeeze(raw);
 
    if ismatrix(A)
 
        if size(A,2) == nu
            U = A;
            return;
        end
 
        if size(A,1) == nu
            U = A.';
            return;
        end
 
    end
 
    v = A(:);
 
    if mod(numel(v),nu) ~= 0
        error('ctrl_pred non compatibile con nu = %d. Num elementi = %d.', nu, numel(v));
    end
 
    n_nodes = numel(v)/nu;
 
    U1 = reshape(v,nu,n_nodes).';
    U2 = reshape(v,n_nodes,nu);
 
    if control_prediction_score(U1) <= control_prediction_score(U2)
        U = U1;
    else
        U = U2;
    end
 
end
 
function score = control_prediction_score(U)
 
    score = 0;
    score = score + 1e5*sum(~isfinite(U(:)));
    score = score + 100*sum(abs(U(:)) > 100);
 
end
 
function OD = format_online_data(raw, nod)
 
    A = squeeze(raw);
 
    if ismatrix(A)
 
        if size(A,2) == nod
            OD = A;
            return;
        end
 
        if size(A,1) == nod
            OD = A.';
            return;
        end
 
    end
 
    v = A(:);
 
    if mod(numel(v),nod) ~= 0
        error('online_data non compatibile con nod = %d. Num elementi = %d.', nod, numel(v));
    end
 
    n_nodes = numel(v)/nod;
 
    OD1 = reshape(v,nod,n_nodes).';
    OD2 = reshape(v,n_nodes,nod);
 
    if online_data_score(OD1) <= online_data_score(OD2)
        OD = OD1;
    else
        OD = OD2;
    end
 
end
 
function score = online_data_score(OD)
 
    score = 0;
 
    score = score + 1e5*sum(~isfinite(OD(:)));
 
    if size(OD,2) < 63
        score = score + 1e12;
        return;
    end
 
    flag_cols = [];
 
    for base = [0 21 42]
        flag_cols = [flag_cols, ...
            base + 1:base + 4, ...
            base + 6:base + 8, ...
            base + 14:base + 17]; %#ok<AGROW>
    end
 
    F = OD(:,flag_cols);
 
    score = score + 10*sum(F(:) < -0.2);
    score = score + 10*sum(F(:) >  1.2);
 
end
 
function D = parse_vehicle_online_data(OD, veh_id)
 
    base = (veh_id - 1)*21;
 
    D.x_TL        = OD(:,base + 1:base + 4);
    D.s_stop      = OD(:,base + 5);
    D.x_stop      = OD(:,base + 6);
    D.x_dwell     = OD(:,base + 7);
    D.w_stop      = OD(:,base + 8);
    D.k_road      = OD(:,base + 9);
    D.Vmax        = OD(:,base + 10);
    D.dt_schd     = OD(:,base + 11);
    D.s_st        = OD(:,base + 12);
    D.s_hor       = OD(:,base + 13);
    D.x_TL_tail   = OD(:,base + 14:base + 17);
    D.s_TL_active = OD(:,base + 18:base + 21);
 
end
 
function tau = get_prediction_time_vector(n_nodes)
 
    if evalin('base','exist(''timepoints'',''var'')')
        tp = evalin('base','timepoints');
        tp = tp(:);
 
        if numel(tp) == n_nodes
            tau = tp;
            return;
        end
    end
 
    tau = (0:n_nodes-1).';
 
end
%%=========================================================================
% DEBUG VINCOLI NMPC 2 VEH - SENZA AGGIUNGERE TO WORKSPACE
%=========================================================================
% Usa direttamente i segnali già salvati dal modello Simulink:
%
%   out.status
%   out.state_pred
%   out.ctrl_pred
%   out.online_data
%   out.states
%
% Obiettivo:
%   - trovare lo step in cui status == -2
%   - estrarre prediction states, controls e OnlineData
%   - ricostruire tutti i vincoli dell'OCP
%   - ordinare i vincoli per violazione massima
%==========================================================================

clc;

%%=========================================================================
% 1. PARAMETRI DEBUG
%==========================================================================

target_status = -2;     % codice errore solver da cercare
idx_user      = [];     % lascia [] per primo status == -2, oppure imposta indice manuale

N    = 50;              % prediction horizon ACADO
nx   = 6;               % [pos vel acc pos2 vel2 acc2]
nu   = 2;               % [jerk jerk2]
nod  = 62;              % numero OnlineData
tol  = 1e-7;            % tolleranza violazione vincoli
nshow = 40;             % numero righe da stampare

%%=========================================================================
% 2. CARICAMENTO LOG DAL WORKSPACE
%==========================================================================

if exist('out','var')
    log_status      = out.status;
    log_state_pred  = out.state_pred;
    log_ctrl_pred   = out.ctrl_pred;
    log_online_data = out.online_data;

    if isprop(out,'states') || isfield(out,'states')
        log_states = out.states;
    else
        log_states = [];
    end
else
    % Fallback nel caso tu abbia salvato i To Workspace direttamente nel base workspace
    log_status      = status;
    log_state_pred  = state_pred;
    log_ctrl_pred   = ctrl_pred;
    log_online_data = online_data;

    if exist('states','var')
        log_states = states;
    else
        log_states = [];
    end
end

%%=========================================================================
% 3. TROVA STEP CON STATUS == -2
%==========================================================================

[t_status, status_values] = getLoggedTimeAndValues(log_status);
status_vec = squeeze(status_values);
status_vec = status_vec(:);

if isempty(idx_user)
    idx_fail = find(round(status_vec) == target_status, 1, 'first');

    if isempty(idx_fail)
        error('Nessuno step trovato con status == %d.', target_status);
    end
else
    idx_fail = idx_user;
end

t_fail = t_status(idx_fail);

fprintf('\n============================================================\n');
fprintf('DEBUG VINCOLI NMPC\n');
fprintf('============================================================\n');
fprintf('Status target     : %d\n', target_status);
fprintf('Indice status     : %d\n', idx_fail);
fprintf('Tempo simulazione : %.6f s\n', t_fail);
fprintf('Status effettivo  : %g\n', status_vec(idx_fail));
fprintf('============================================================\n\n');

%%=========================================================================
% 4. ESTRAZIONE SAMPLE A t_fail
%==========================================================================

[Xraw, idx_X, t_X]   = sampleLoggedSignalAtTime(log_state_pred,  t_fail);
[Uraw, idx_U, t_U]   = sampleLoggedSignalAtTime(log_ctrl_pred,   t_fail);
[ODraw, idx_OD, t_OD] = sampleLoggedSignalAtTime(log_online_data, t_fail);

fprintf('Campione state_pred  : idx = %d, t = %.6f s\n', idx_X,  t_X);
fprintf('Campione ctrl_pred   : idx = %d, t = %.6f s\n', idx_U,  t_U);
fprintf('Campione online_data : idx = %d, t = %.6f s\n\n', idx_OD, t_OD);

%%=========================================================================
% 5. ESTRAZIONE STATO CORRENTE, SE DISPONIBILE
%==========================================================================

x0_logged = [];

if ~isempty(log_states)
    try
        [x0_raw, ~, ~] = sampleLoggedSignalAtTime(log_states, t_fail);
        x0_logged = squeeze(x0_raw);
        x0_logged = x0_logged(:).';

        if numel(x0_logged) ~= nx
            x0_logged = [];
        end
    catch
        x0_logged = [];
    end
end

%%=========================================================================
% 6. FORMATTAZIONE PREDICTION E ONLINEDATA
%==========================================================================

X  = formatStatePrediction(Xraw, N, nx, x0_logged);
U  = formatControlPrediction(Uraw, N, nu);
OD = formatOnlineData(ODraw, N, nod);

fprintf('Dimensioni estratte:\n');
fprintf('  X  = [%d x %d]\n', size(X,1),  size(X,2));
fprintf('  U  = [%d x %d]\n', size(U,1),  size(U,2));
fprintf('  OD = [%d x %d]\n\n', size(OD,1), size(OD,2));

if size(X,2) ~= nx
    error('X ha numero colonne errato. Atteso nx = %d.', nx);
end

if size(U,2) ~= nu
    error('U ha numero colonne errato. Atteso nu = %d.', nu);
end

if size(OD,2) ~= nod
    error('OD ha numero colonne errato. Atteso nod = %d.', nod);
end

%%=========================================================================
% 7. CALCOLO RESIDUI VINCOLI
%==========================================================================

[Tviol_max, Tviol_node, Tall_max] = computeConstraintResiduals2veh(X, U, OD, tol);

%%=========================================================================
% 8. OUTPUT PRINCIPALE
%==========================================================================

fprintf('Stato predetto al nodo 0:\n');
fprintf('  pos  = %.6f m\n',   X(1,1));
fprintf('  vel  = %.6f m/s\n', X(1,2));
fprintf('  acc  = %.6f m/s^2\n', X(1,3));
fprintf('  pos2 = %.6f m\n',   X(1,4));
fprintf('  vel2 = %.6f m/s\n', X(1,5));
fprintf('  acc2 = %.6f m/s^2\n\n', X(1,6));

if isempty(Tviol_max)
    fprintf('Nessun vincolo ricostruito risulta violato oltre tol = %.1e.\n', tol);
    fprintf('Mostro comunque i vincoli più vicini alla violazione:\n\n');

    disp(Tall_max(1:min(nshow,height(Tall_max)),:));
else
    fprintf('Vincoli violati, ordinati per massima violazione:\n\n');
    disp(Tviol_max(1:min(nshow,height(Tviol_max)),:));

    fprintf('\nViolazioni puntuali lungo la prediction:\n\n');
    disp(Tviol_node(1:min(nshow,height(Tviol_node)),:));
end

%%=========================================================================
% 9. CONTROLLO SPECIFICO SUL NODO INIZIALE h = 0
%==========================================================================

T0 = Tviol_node(Tviol_node.PredictionNode == 0,:);

fprintf('\n============================================================\n');
fprintf('CHECK NODO INIZIALE h = 0\n');
fprintf('============================================================\n');

if isempty(T0)
    fprintf('Nessuna violazione al nodo iniziale.\n');
    fprintf('La infeasibility è probabilmente generata lungo la prediction.\n');
else
    fprintf('Vincoli già violati al nodo iniziale:\n\n');
    disp(T0);
    fprintf('In questo caso il solver riceve uno stato iniziale incompatibile con i vincoli hard.\n');
end

%%=========================================================================
% 10. VARIABILI LASCIATE NEL WORKSPACE
%==========================================================================

% Variabili utili lasciate nel workspace:
%   X
%   U
%   OD
%   Tviol_max
%   Tviol_node
%   Tall_max
%   T0
%   idx_fail
%   t_fail


%%=========================================================================
% FUNZIONI LOCALI
%==========================================================================

function [t, values] = getLoggedTimeAndValues(logsig)

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


function [sample, idx, t_sample] = sampleLoggedSignalAtTime(logsig, t_query)

    [t, values] = getLoggedTimeAndValues(logsig);

    [~, idx] = min(abs(t - t_query));
    t_sample = t(idx);

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

    subs = repmat({':'}, 1, ndims(values));
    subs{time_dim} = idx;

    sample = squeeze(values(subs{:}));
end


function X = formatStatePrediction(raw, N, nx, x0_logged)

    nRows = N + 1;
    nCols = nx;

    A = squeeze(raw);

    if isequal(size(A), [nRows nCols])
        X = A;
        return;
    end

    if isequal(size(A), [nCols nRows])
        X = A.';
        return;
    end

    v = A(:);

    if numel(v) ~= nRows*nCols
        error('state_pred ha %d elementi. Attesi %d.', numel(v), nRows*nCols);
    end

    % Candidato 1: ordinamento tipo [x0; x1; ...; xN]
    X1 = reshape(v, nCols, nRows).';

    % Candidato 2: ordinamento MATLAB column-major diretto
    X2 = reshape(v, nRows, nCols);

    if ~isempty(x0_logged)
        e1 = norm(X1(1,:) - x0_logged);
        e2 = norm(X2(1,:) - x0_logged);

        if e1 <= e2
            X = X1;
        else
            X = X2;
        end
    else
        % Fallback: scegli il candidato più fisicamente coerente
        score1 = statePredictionScore(X1);
        score2 = statePredictionScore(X2);

        if score1 <= score2
            X = X1;
        else
            X = X2;
        end
    end
end


function score = statePredictionScore(X)

    pos  = X(:,1);
    vel  = X(:,2);
    acc  = X(:,3);
    pos2 = X(:,4);
    vel2 = X(:,5);
    acc2 = X(:,6);

    score = 0;

    score = score + 1e3*sum(~isfinite(X(:)));
    score = score + 1e2*sum(abs(vel)  > 80);
    score = score + 1e2*sum(abs(vel2) > 80);
    score = score + 1e2*sum(abs(acc)  > 20);
    score = score + 1e2*sum(abs(acc2) > 20);

    score = score + 10*sum(diff(pos)  < -5);
    score = score + 10*sum(diff(pos2) < -5);
end


function U = formatControlPrediction(raw, N, nu)

    A = squeeze(raw);

    if isequal(size(A), [N nu])
        U = A;
        return;
    end

    if isequal(size(A), [nu N])
        U = A.';
        return;
    end

    if isequal(size(A), [N+1 nu])
        U = A;
        return;
    end

    if isequal(size(A), [nu N+1])
        U = A.';
        return;
    end

    v = A(:);

    if numel(v) == N*nu
        nRows = N;
    elseif numel(v) == (N+1)*nu
        nRows = N + 1;
    else
        error('ctrl_pred ha %d elementi. Attesi %d oppure %d.', ...
            numel(v), N*nu, (N+1)*nu);
    end

    U1 = reshape(v, nu, nRows).';
    U2 = reshape(v, nRows, nu);

    score1 = controlPredictionScore(U1);
    score2 = controlPredictionScore(U2);

    if score1 <= score2
        U = U1;
    else
        U = U2;
    end
end


function score = controlPredictionScore(U)

    score = 0;
    score = score + 1e3*sum(~isfinite(U(:)));
    score = score + 1e2*sum(abs(U(:)) > 10);
end


function OD = formatOnlineData(raw, N, nod)

    A = squeeze(raw);

    if isequal(size(A), [N+1 nod])
        OD = A;
        return;
    end

    if isequal(size(A), [nod N+1])
        OD = A.';
        return;
    end

    if isequal(size(A), [N nod])
        OD = A;
        return;
    end

    if isequal(size(A), [nod N])
        OD = A.';
        return;
    end

    v = A(:);

    if numel(v) == (N+1)*nod
        nRows = N + 1;
    elseif numel(v) == N*nod
        nRows = N;
    else
        error('online_data ha %d elementi. Attesi %d oppure %d.', ...
            numel(v), (N+1)*nod, N*nod);
    end

    OD1 = reshape(v, nod, nRows).';
    OD2 = reshape(v, nRows, nod);

    score1 = onlineDataScore(OD1);
    score2 = onlineDataScore(OD2);

    if score1 <= score2
        OD = OD1;
    else
        OD = OD2;
    end
end


function score = onlineDataScore(OD)

    score = 0;
    score = score + 1e4*sum(~isfinite(OD(:)));

    if size(OD,2) < 62
        score = score + 1e8;
        return;
    end

    % Colonne flag attese circa in [0,1]
    flag_cols = [1:11, 13:15, 21:31, 32:42, 44:46, 52:62];

    flags = OD(:,flag_cols);
    score = score + 10*sum(flags(:) < -0.2);
    score = score + 10*sum(flags(:) >  1.2);

    % Vmax e Vmax_2 devono essere positivi e ragionevoli
    Vmax  = OD(:,17);
    Vmax2 = OD(:,48);

    score = score + 100*sum(Vmax  <= 0 | Vmax  > 100);
    score = score + 100*sum(Vmax2 <= 0 | Vmax2 > 100);

    % k_road e k_road_2 devono essere finiti e non enormi
    kroad  = OD(:,16);
    kroad2 = OD(:,47);

    score = score + 100*sum(abs(kroad)  > 1);
    score = score + 100*sum(abs(kroad2) > 1);
end


function [Tviol_max, Tviol_node, Tall_max] = computeConstraintResiduals2veh(X, U, OD, tol)

    %%=====================================================================
    % Parametri coerenti con ACADO_2veh.m
    %======================================================================

    s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, ...
            1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0];

    s_max = 5200;

    Ax_min = -1.5;
    Ax_max =  1.5;

    Ay_max = 2.0;

    jerk_min = -0.5;
    jerk_max =  0.5;

    L_platoon = 10;

    eps_stop_back  = 0.5;
    eps_stop_front = 0.5;
    eps_stop_v     = 0.1;
    eps_stop_a     = 0.3;

    V_max_static = 50/3.6;
    d_safe_TL = 0;

    %%=====================================================================
    % Allineamento dimensioni
    %======================================================================

    nX  = size(X,1);
    nOD = size(OD,1);

    nNode = min(nX, nOD);

    X  = X(1:nNode,:);
    OD = OD(1:nNode,:);

    %%=====================================================================
    % Stati e controlli
    %======================================================================

    pos  = X(:,1);
    vel  = X(:,2);
    acc  = X(:,3);

    pos2 = X(:,4);
    vel2 = X(:,5);
    acc2 = X(:,6);

    jerk  = U(:,1);
    jerk2 = U(:,2);

    %%=====================================================================
    % OnlineData mapping
    %======================================================================

    x_TL      = OD(:,1:11);

    s_stop    = OD(:,12);
    x_stop    = OD(:,13);
    x_dwell   = OD(:,14);

    k_road    = OD(:,16);
    Vmax      = OD(:,17);

    x_TL_tail = OD(:,21:31);

    x_TL_2      = OD(:,32:42);

    s_stop_2    = OD(:,43);
    x_stop_2    = OD(:,44);
    x_dwell_2   = OD(:,45);

    k_road_2    = OD(:,47);
    Vmax_2      = OD(:,48);

    x_TL_tail_2 = OD(:,52:62);

    %%=====================================================================
    % Tabelle interne
    %======================================================================

    C_name = {};
    C_node = [];
    C_max  = [];

    N_name = {};
    N_node = [];
    N_res  = [];

    function addConstraint(name, r)

        r = full(r(:));

        [rmax, imax] = max(r);

        C_name{end+1,1} = name;
        C_node(end+1,1) = imax - 1;
        C_max(end+1,1)  = rmax;

        idx = find(r > tol);

        for kk = 1:numel(idx)
            ii = idx(kk);

            N_name{end+1,1} = name;
            N_node(end+1,1) = ii - 1;
            N_res(end+1,1)  = r(ii);
        end
    end

    %%=====================================================================
    % 1. Accelerazione longitudinale
    %======================================================================

    addConstraint('veh1 acc <= Ax_max', acc - Ax_max);
    addConstraint('veh1 acc >= Ax_min', Ax_min - acc);

    addConstraint('veh2 acc <= Ax_max', acc2 - Ax_max);
    addConstraint('veh2 acc >= Ax_min', Ax_min - acc2);

    %%=====================================================================
    % 2. Jerk
    %======================================================================

    addConstraint('veh1 jerk <= jerk_max', jerk - jerk_max);
    addConstraint('veh1 jerk >= jerk_min', jerk_min - jerk);

    addConstraint('veh2 jerk <= jerk_max', jerk2 - jerk_max);
    addConstraint('veh2 jerk >= jerk_min', jerk_min - jerk2);

    %%=====================================================================
    % 3. Accelerazione laterale
    %======================================================================

    addConstraint('veh1 vel^2*k_road <= Ay_max', vel.^2 .* k_road - Ay_max);
    addConstraint('veh2 vel2^2*k_road_2 <= Ay_max', vel2.^2 .* k_road_2 - Ay_max);

    %%=====================================================================
    % 4. Velocità
    %======================================================================

    addConstraint('veh1 vel <= Vmax', vel - Vmax);
    addConstraint('veh1 vel >= 0', -vel);

    addConstraint('veh2 vel2 <= Vmax_2', vel2 - Vmax_2);
    addConstraint('veh2 vel2 >= 0', -vel2);

    %%=====================================================================
    % 5. Gap minimo tra i due veicoli
    %======================================================================

    gap = pos - pos2 - L_platoon;

    addConstraint('gap = pos-pos2-L_platoon >= 2', 2 - gap);

    %%=====================================================================
    % 6. Fermata: approccio
    %======================================================================

    r_stop_1 = pos  - (s_stop   .* (1 - x_stop)   + s_max .* x_stop);
    r_stop_2 = pos2 - (s_stop_2 .* (1 - x_stop_2) + s_max .* x_stop_2);

    addConstraint('veh1 stop approach', r_stop_1);
    addConstraint('veh2 stop approach', r_stop_2);

    %%=====================================================================
    % 7. Fermata: dwell box
    %======================================================================

    r_dwell_front_1 = pos - ((s_stop + eps_stop_front) .* x_dwell + ...
                             s_max .* (1 - x_dwell));

    r_dwell_back_1 = (s_stop - eps_stop_back) .* x_dwell - ...
                     s_max .* (1 - x_dwell) - pos;

    r_dwell_front_2 = pos2 - ((s_stop_2 + eps_stop_front) .* x_dwell_2 + ...
                              s_max .* (1 - x_dwell_2));

    r_dwell_back_2 = (s_stop_2 - eps_stop_back) .* x_dwell_2 - ...
                     s_max .* (1 - x_dwell_2) - pos2;

    addConstraint('veh1 dwell front box', r_dwell_front_1);
    addConstraint('veh1 dwell back box',  r_dwell_back_1);

    addConstraint('veh2 dwell front box', r_dwell_front_2);
    addConstraint('veh2 dwell back box',  r_dwell_back_2);

    %%=====================================================================
    % 8. Fermata: velocità e accelerazione durante dwell
    %======================================================================

    r_dwell_vel_1 = vel - (eps_stop_v .* x_dwell + ...
                           V_max_static .* (1 - x_dwell));

    r_dwell_acc_max_1 = acc - (eps_stop_a .* x_dwell + ...
                               Ax_max .* (1 - x_dwell));

    r_dwell_acc_min_1 = -(acc + (eps_stop_a .* x_dwell + ...
                                 abs(Ax_min) .* (1 - x_dwell)));

    r_dwell_vel_2 = vel2 - (eps_stop_v .* x_dwell_2 + ...
                            V_max_static .* (1 - x_dwell_2));

    r_dwell_acc_max_2 = acc2 - (eps_stop_a .* x_dwell_2 + ...
                                Ax_max .* (1 - x_dwell_2));

    r_dwell_acc_min_2 = -(acc2 + (eps_stop_a .* x_dwell_2 + ...
                                  abs(Ax_min) .* (1 - x_dwell_2)));

    addConstraint('veh1 dwell vel',     r_dwell_vel_1);
    addConstraint('veh1 dwell acc max', r_dwell_acc_max_1);
    addConstraint('veh1 dwell acc min', r_dwell_acc_min_1);

    addConstraint('veh2 dwell vel',     r_dwell_vel_2);
    addConstraint('veh2 dwell acc max', r_dwell_acc_max_2);
    addConstraint('veh2 dwell acc min', r_dwell_acc_min_2);

    %%=====================================================================
    % 9. Semafori testa/coda veicolo 1
    %======================================================================

    for i = 1:11

        r_head_1 = pos - ((s_TL(i) - d_safe_TL) .* (1 - x_TL(:,i)) + ...
                          s_max .* x_TL(:,i));

        r_tail_1 = (pos - L_platoon) - ...
                   ((s_TL(i) - d_safe_TL) .* (1 - x_TL_tail(:,i)) + ...
                    s_max .* x_TL_tail(:,i));

        addConstraint(sprintf('veh1 TL%02d head', i), r_head_1);
        addConstraint(sprintf('veh1 TL%02d tail', i), r_tail_1);
    end

    %%=====================================================================
    % 10. Semafori testa/coda veicolo 2
    %======================================================================

    for i = 1:11

        r_head_2 = pos2 - ((s_TL(i) - d_safe_TL) .* (1 - x_TL_2(:,i)) + ...
                           s_max .* x_TL_2(:,i));

        r_tail_2 = (pos2 - L_platoon) - ...
                   ((s_TL(i) - d_safe_TL) .* (1 - x_TL_tail_2(:,i)) + ...
                    s_max .* x_TL_tail_2(:,i));

        addConstraint(sprintf('veh2 TL%02d head', i), r_head_2);
        addConstraint(sprintf('veh2 TL%02d tail', i), r_tail_2);
    end

    %%=====================================================================
    % Tabelle finali
    %======================================================================

    Tall_max = table(C_name, C_node, C_max, ...
        'VariableNames', {'Constraint','PredictionNode','MaxResidual'});

    Tall_max = sortrows(Tall_max, 'MaxResidual', 'descend');

    Tviol_max = Tall_max(Tall_max.MaxResidual > tol,:);

    Tviol_node = table(N_name, N_node, N_res, ...
        'VariableNames', {'Constraint','PredictionNode','Residual'});

    if ~isempty(Tviol_node)
        Tviol_node = sortrows(Tviol_node, 'Residual', 'descend');
    end
end
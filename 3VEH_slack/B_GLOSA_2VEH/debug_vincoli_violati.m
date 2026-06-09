%%=========================================================================
% DEBUG VINCOLI NMPC 3 VEH - STATUS -2 / -30
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
%   - trovare tutti gli step in cui status == -2 oppure status == -30
%   - salvare tutti gli errori in Tfail_all
%   - selezionare uno step di errore da analizzare in dettaglio
%   - estrarre prediction states, controls e OnlineData
%   - ricostruire tutti i vincoli dell'OCP ACADO 3 veicoli / 4 TL attivi
%   - ordinare i vincoli per violazione massima
%
% Convenzione stati:
%   X = [pos vel acc pos2 vel2 acc2 pos3 vel3 acc3]
%
% Convenzione controlli:
%   U = [jerk jerk2 jerk3]
%
% Convenzione OnlineData:
%   nod = 63 = 21 dati per veicolo x 3 veicoli
%
% Per ogni veicolo, blocco da 21:
%   1:4     x_TL1 ... x_TL4
%   5       s_stop_active
%   6       x_stop_active
%   7       x_dwell_active
%   8       w_stop_active
%   9       k_road
%   10      Vmax
%   11      dt_schd
%   12      s_st
%   13      s_hor
%   14:17   x_TL1_tail ... x_TL4_tail
%   18:21   s_TL1_active ... s_TL4_active
%==========================================================================

clc;

%%=========================================================================
% 1. PARAMETRI DEBUG
%==========================================================================

target_statuses = [-2; -30];   % status solver da cercare

idx_user = [];                  % [] -> usa automaticamente uno step trovato
                                % numero intero -> forza indice assoluto status

fail_to_analyze = 1;            % se idx_user = [], analizza il k-esimo errore
                                % trovato in idx_fail_all

N    = 40;                      % prediction horizon ACADO
nx   = 9;                       % [pos vel acc pos2 vel2 acc2 pos3 vel3 acc3]
nu   = 3;                       % [jerk jerk2 jerk3]
nod  = 63;                      % 21 OnlineData per veicolo x 3 veicoli
tol  = 1e-7;                    % tolleranza violazione vincoli
nshow = 40;                     % numero righe da stampare

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

    % Fallback nel caso tu abbia salvato i To Workspace direttamente
    % nel base workspace.
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
% 3. TROVA TUTTI GLI STEP CON STATUS == -2 O STATUS == -30
%==========================================================================

[t_status, status_values] = getLoggedTimeAndValues(log_status);

status_vec = squeeze(status_values);
status_vec = status_vec(:);

status_int = round(status_vec);

idx_fail_all = find(ismember(status_int, target_statuses));

if isempty(idx_fail_all)
    error('Nessuno step trovato con status == -2 o status == -30.');
end

t_fail_all = t_status(idx_fail_all);

Tfail_all = table( ...
    idx_fail_all(:), ...
    t_fail_all(:), ...
    status_vec(idx_fail_all(:)), ...
    'VariableNames', {'Index','Time','Status'} );

fprintf('\n============================================================\n');
fprintf('STEP CON ERRORE SOLVER -2 O -30\n');
fprintf('============================================================\n');
disp(Tfail_all);

fprintf('Numero totale step in errore: %d\n',numel(idx_fail_all));
fprintf('Numero status -2 : %d\n',sum(status_int == -2));
fprintf('Numero status -30: %d\n',sum(status_int == -30));

%%=========================================================================
% 4. SELEZIONE DELLO STEP DA ANALIZZARE IN DETTAGLIO
%==========================================================================

if isempty(idx_user)

    if fail_to_analyze < 1 || fail_to_analyze > numel(idx_fail_all)
        error(['fail_to_analyze fuori range. Valore = %d, ', ...
               'range valido = [1, %d].'], ...
               fail_to_analyze, numel(idx_fail_all));
    end

    idx_fail = idx_fail_all(fail_to_analyze);

else

    idx_fail = idx_user;

    if idx_fail < 1 || idx_fail > numel(status_vec)
        error('idx_user fuori range. Valore = %d, range valido = [1, %d].', ...
              idx_fail, numel(status_vec));
    end

    if ~ismember(round(status_vec(idx_fail)), target_statuses)
        warning(['idx_user = %d non corrisponde a status -2 o -30. ', ...
                 'Status effettivo = %g.'], ...
                 idx_fail, status_vec(idx_fail));
    end

end

t_fail = t_status(idx_fail);

fprintf('\n============================================================\n');
fprintf('DEBUG VINCOLI NMPC 3 VEH\n');
fprintf('============================================================\n');
fprintf('Status target        : [-2 -30]\n');
fprintf('Indice analizzato    : %d\n', idx_fail);
fprintf('Tempo simulazione    : %.6f s\n', t_fail);
fprintf('Status effettivo     : %g\n', status_vec(idx_fail));
fprintf('Errore analizzato    : %d / %d\n', fail_to_analyze, numel(idx_fail_all));
fprintf('============================================================\n\n');

%%=========================================================================
% 5. ESTRAZIONE SAMPLE A t_fail
%==========================================================================

[Xraw, idx_X, t_X]     = sampleLoggedSignalAtTime(log_state_pred,  t_fail);
[Uraw, idx_U, t_U]     = sampleLoggedSignalAtTime(log_ctrl_pred,   t_fail);
[ODraw, idx_OD, t_OD]  = sampleLoggedSignalAtTime(log_online_data, t_fail);

fprintf('Campione state_pred  : idx = %d, t = %.6f s\n', idx_X,  t_X);
fprintf('Campione ctrl_pred   : idx = %d, t = %.6f s\n', idx_U,  t_U);
fprintf('Campione online_data : idx = %d, t = %.6f s\n\n', idx_OD, t_OD);

%%=========================================================================
% 6. ESTRAZIONE STATO CORRENTE, SE DISPONIBILE
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
% 7. FORMATTAZIONE PREDICTION E ONLINEDATA
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
% 8. CALCOLO RESIDUI VINCOLI
%==========================================================================

[Tviol_max, Tviol_node, Tall_max] = computeConstraintResiduals3veh(X, U, OD, tol);

%%=========================================================================
% 9. OUTPUT PRINCIPALE
%==========================================================================

fprintf('Stato predetto al nodo 0:\n');
fprintf('  pos  = %.6f m, vel  = %.6f m/s, acc  = %.6f m/s^2\n', ...
        X(1,1), X(1,2), X(1,3));
fprintf('  pos2 = %.6f m, vel2 = %.6f m/s, acc2 = %.6f m/s^2\n', ...
        X(1,4), X(1,5), X(1,6));
fprintf('  pos3 = %.6f m, vel3 = %.6f m/s, acc3 = %.6f m/s^2\n\n', ...
        X(1,7), X(1,8), X(1,9));

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
% 10. CONTROLLO SPECIFICO SUL NODO INIZIALE h = 0
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
% 11. VARIABILI LASCIATE NEL WORKSPACE
%==========================================================================
%
% Variabili utili lasciate nel workspace:
%
%   status_vec
%   status_int
%   target_statuses
%   idx_fail_all
%   t_fail_all
%   Tfail_all
%   idx_fail
%   t_fail
%   X
%   U
%   OD
%   Tviol_max
%   Tviol_node
%   Tall_max
%   T0
%
%==========================================================================


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
        error('state_pred ha %d elementi. Attesi %d.', ...
              numel(v), nRows*nCols);
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

    vel  = X(:,2);
    acc  = X(:,3);

    vel2 = X(:,5);
    acc2 = X(:,6);

    vel3 = X(:,8);
    acc3 = X(:,9);

    pos  = X(:,1);
    pos2 = X(:,4);
    pos3 = X(:,7);

    score = 0;

    score = score + 1e3*sum(~isfinite(X(:)));

    score = score + 1e2*sum(abs(vel)  > 80);
    score = score + 1e2*sum(abs(vel2) > 80);
    score = score + 1e2*sum(abs(vel3) > 80);

    score = score + 1e2*sum(abs(acc)  > 20);
    score = score + 1e2*sum(abs(acc2) > 20);
    score = score + 1e2*sum(abs(acc3) > 20);

    score = score + 10*sum(diff(pos)  < -5);
    score = score + 10*sum(diff(pos2) < -5);
    score = score + 10*sum(diff(pos3) < -5);

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

    if size(OD,2) < 63
        score = score + 1e8;
        return;
    end

    % Flag attesi circa in [0,1].
    % Blocco veicolo:
    % [1:4, 6:8, 14:17] rispetto al blocco locale.
    flag_cols = [];

    for base = [0 21 42]

        flag_cols = [flag_cols, ...
                     base + 1:base + 4, ...
                     base + 6:base + 8, ...
                     base + 14:base + 17]; %#ok<AGROW>

    end

    flags = OD(:,flag_cols);

    score = score + 10*sum(flags(:) < -0.2);
    score = score + 10*sum(flags(:) >  1.2);

    % Vmax dei tre veicoli devono essere positivi e ragionevoli.
    Vmax1 = OD(:,10);
    Vmax2 = OD(:,31);
    Vmax3 = OD(:,52);

    score = score + 100*sum(Vmax1 <= 0 | Vmax1 > 100);
    score = score + 100*sum(Vmax2 <= 0 | Vmax2 > 100);
    score = score + 100*sum(Vmax3 <= 0 | Vmax3 > 100);

    % k_road dei tre veicoli devono essere finiti e non enormi.
    kroad1 = OD(:,9);
    kroad2 = OD(:,30);
    kroad3 = OD(:,51);

    score = score + 100*sum(abs(kroad1) > 1);
    score = score + 100*sum(abs(kroad2) > 1);
    score = score + 100*sum(abs(kroad3) > 1);

end


function [Tviol_max, Tviol_node, Tall_max] = computeConstraintResiduals3veh(X, U, OD, tol)

    %%=====================================================================
    % Parametri coerenti con ACADO_3veh_4TL.m
    %======================================================================

    s_max = 5200;

    Ax_min = -1.5;
    Ax_max =  1.5;

    Ay_max = 2.0;

    jerk_min = -0.5;
    jerk_max =  0.5;

    L_platoon = 7.8;

    eps_stop_back  = 0.5;
    eps_stop_front = 0.5;
    eps_stop_v     = 0.1;
    eps_stop_a     = 0.3;

    V_max_static = 50/3.6;
    d_safe_TL = 0;
    v_eps = 0.01;

    gap_min = 0.5;

    %%=====================================================================
    % Allineamento dimensioni
    %======================================================================

    nX  = size(X,1);
    nOD = size(OD,1);

    nNode = min(nX, nOD);

    X  = X(1:nNode,:);
    OD = OD(1:nNode,:);

    % I controlli possono essere N x nu oppure N+1 x nu.
    nU = size(U,1);
    nNodeU = min(nNode,nU);

    %%=====================================================================
    % Stati e controlli
    %======================================================================

    pos  = X(:,1);
    vel  = X(:,2);
    acc  = X(:,3);

    pos2 = X(:,4);
    vel2 = X(:,5);
    acc2 = X(:,6);

    pos3 = X(:,7);
    vel3 = X(:,8);
    acc3 = X(:,9);

    jerk  = U(1:nNodeU,1);
    jerk2 = U(1:nNodeU,2);
    jerk3 = U(1:nNodeU,3);

    %%=====================================================================
    % OnlineData mapping K4 - 21 colonne per veicolo
    %======================================================================

    D1 = parseVehicleOD(OD,1);
    D2 = parseVehicleOD(OD,2);
    D3 = parseVehicleOD(OD,3);

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

    addConstraint('veh3 acc <= Ax_max', acc3 - Ax_max);
    addConstraint('veh3 acc >= Ax_min', Ax_min - acc3);

    %%=====================================================================
    % 2. Jerk
    %======================================================================

    addConstraint('veh1 jerk <= jerk_max', jerk - jerk_max);
    addConstraint('veh1 jerk >= jerk_min', jerk_min - jerk);

    addConstraint('veh2 jerk <= jerk_max', jerk2 - jerk_max);
    addConstraint('veh2 jerk >= jerk_min', jerk_min - jerk2);

    addConstraint('veh3 jerk <= jerk_max', jerk3 - jerk_max);
    addConstraint('veh3 jerk >= jerk_min', jerk_min - jerk3);

    %%=====================================================================
    % 3. Accelerazione laterale
    %======================================================================

    addConstraint('veh1 vel^2*k_road <= Ay_max', ...
                  vel.^2 .* D1.k_road - Ay_max);

    addConstraint('veh2 vel2^2*k_road_2 <= Ay_max', ...
                  vel2.^2 .* D2.k_road - Ay_max);

    addConstraint('veh3 vel3^2*k_road_3 <= Ay_max', ...
                  vel3.^2 .* D3.k_road - Ay_max);

    %%=====================================================================
    % 4. Velocità
    %======================================================================

    addConstraint('veh1 vel <= Vmax', vel - D1.Vmax);
    addConstraint('veh1 vel >= -v_eps', -v_eps - vel);

    addConstraint('veh2 vel2 <= Vmax_2', vel2 - D2.Vmax);
    addConstraint('veh2 vel2 >= -v_eps', -v_eps - vel2);

    addConstraint('veh3 vel3 <= Vmax_3', vel3 - D3.Vmax);
    addConstraint('veh3 vel3 >= -v_eps', -v_eps - vel3);

    %%=====================================================================
    % 5. Gap minimo
    %======================================================================

    gap12 = pos  - pos2 - L_platoon;
    gap23 = pos2 - pos3 - L_platoon;

    addConstraint('gap12 >= gap_min', gap_min - gap12);
    addConstraint('gap23 >= gap_min', gap_min - gap23);

    %%=====================================================================
    % 6. Fermate bus
    %======================================================================

    addStopConstraints(1,pos, vel, acc, D1);
    addStopConstraints(2,pos2,vel2,acc2,D2);
    addStopConstraints(3,pos3,vel3,acc3,D3);

    %%=====================================================================
    % 7. Semafori testa/coda per 4 slot mobili
    %======================================================================

    addTLConstraints(1,pos, D1);
    addTLConstraints(2,pos2,D2);
    addTLConstraints(3,pos3,D3);

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

    %%=====================================================================
    % Nested helper: stop constraints
    %======================================================================

    function addStopConstraints(id,pos_i,vel_i,acc_i,D)

        prefix = sprintf('veh%d',id);

        %--------------------------------------------------------------
        % Stop approach
        % Vincolo ACADO:
        % pos_i <= s_stop*(1-x_stop) + s_max*x_stop
        %--------------------------------------------------------------

        r_stop = pos_i - ...
                 (D.s_stop .* (1 - D.x_stop) + s_max .* D.x_stop);

        addConstraint([prefix ' stop approach'], r_stop);

        %--------------------------------------------------------------
        % Dwell box
        %--------------------------------------------------------------

        r_dwell_front = pos_i - ...
                        ((D.s_stop + eps_stop_front) .* D.x_dwell + ...
                         s_max .* (1 - D.x_dwell));

        r_dwell_back = ...
                        (D.s_stop - eps_stop_back) .* D.x_dwell - ...
                        s_max .* (1 - D.x_dwell) - pos_i;

        addConstraint([prefix ' dwell front box'], r_dwell_front);
        addConstraint([prefix ' dwell back box'],  r_dwell_back);

        %--------------------------------------------------------------
        % Dwell velocity and acceleration
        %--------------------------------------------------------------

        r_dwell_vel = vel_i - ...
                      (eps_stop_v .* D.x_dwell + ...
                       V_max_static .* (1 - D.x_dwell));

        r_dwell_acc_max = acc_i - ...
                          (eps_stop_a .* D.x_dwell + ...
                           Ax_max .* (1 - D.x_dwell));

        r_dwell_acc_min = ...
                         -(acc_i + ...
                          (eps_stop_a .* D.x_dwell + ...
                           abs(Ax_min) .* (1 - D.x_dwell)));

        addConstraint([prefix ' dwell vel'],     r_dwell_vel);
        addConstraint([prefix ' dwell acc max'], r_dwell_acc_max);
        addConstraint([prefix ' dwell acc min'], r_dwell_acc_min);

    end

    %%=====================================================================
    % Nested helper: traffic light constraints
    %======================================================================

    function addTLConstraints(id,pos_i,D)

        prefix = sprintf('veh%d',id);

        for kk = 1:4

            sTL = D.s_TL_active(:,kk);

            %----------------------------------------------------------
            % Testa veicolo
            %
            % Vincolo ACADO:
            % pos_i <= (sTL-d_safe_TL)*(1-x_TL) + s_max*x_TL
            %----------------------------------------------------------

            r_head = pos_i - ...
                     ((sTL - d_safe_TL) .* (1 - D.x_TL(:,kk)) + ...
                       s_max .* D.x_TL(:,kk));

            %----------------------------------------------------------
            % Coda veicolo
            %
            % Vincolo ACADO:
            % pos_i - L_platoon <=
            %   (sTL-d_safe_TL)*(1-x_TL_tail) + s_max*x_TL_tail
            %----------------------------------------------------------

            r_tail = (pos_i - L_platoon) - ...
                     ((sTL - d_safe_TL) .* (1 - D.x_TL_tail(:,kk)) + ...
                       s_max .* D.x_TL_tail(:,kk));

            addConstraint(sprintf('%s TLslot%02d head', prefix, kk), r_head);
            addConstraint(sprintf('%s TLslot%02d tail', prefix, kk), r_tail);

        end

    end

end


function D = parseVehicleOD(OD,veh_id)

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
%%=========================================================================
% DEBUG VINCOLI NMPC 2 VEH - STATUS -2 / -30
%=========================================================================
% Usa i segnali salvati dal modello Simulink:
%
%   out.status
%   out.state_pred
%   out.ctrl_pred
%   out.online_data
%   out.states
%
% Convenzione stati:
%   X = [pos vel acc pos2 vel2 acc2]
%
% Convenzione controlli:
%   U = [jerk jerk2 s_Jmin1 s_Jmin2]
%
% Nota: s_Jmin1 e s_Jmin2 rilassano SOLO il limite inferiore sul jerk.
%       Il limite superiore sul jerk rimane hard.
%
% Convenzione OnlineData:
%   nod = 42 = 21 dati per veicolo x 2 veicoli
%
% Per ogni veicolo, blocco da 21:
%   1:4     x_TL1 ... x_TL4      (flag testa)
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
target_statuses = [-2; -30];
idx_user       = [];   % [] = automatico, intero = forza indice assoluto
fail_to_analyze = 1;   % k-esimo errore da analizzare se idx_user = []
N    = 44;    % prediction horizon ACADO
nx   = 6;    % [pos vel acc pos2 vel2 acc2]
nu   = 4;    % [jerk jerk2 s_Jmin1 s_Jmin2]
nod  = 42;   % 21 OnlineData per veicolo x 2 veicoli
tol  = 1e-7; % tolleranza violazione vincoli
nshow = 40;  % numero righe da stampare
%%=========================================================================
% 2. CARICAMENTO LOG DAL WORKSPACE
%==========================================================================
if exist('out','var')
    log_status      = out.status;
    log_state_pred  = out.state_pred;
    log_ctrl_pred   = out.ctrl_pred;
    log_online_data = out.online_data;

    % Il segnale out.states non sempre esiste. Uso try/catch per evitare
    % errori con Simulink.SimulationOutput, struct o timeseries custom.
    try
        log_states = out.states;
    catch
        log_states = [];
    end
else
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
% 3. TROVA STEP CON STATUS ERRATO
%==========================================================================
[t_status, status_values] = getLoggedTimeAndValues(log_status);
status_vec  = squeeze(status_values);
status_vec  = status_vec(:);
status_int  = round(status_vec);
idx_fail_all = find(ismember(status_int, target_statuses));
if isempty(idx_fail_all)
    error('Nessuno step trovato con status == -2 o status == -30.');
end
t_fail_all = t_status(idx_fail_all);
Tfail_all = table(idx_fail_all(:), t_fail_all(:), status_vec(idx_fail_all(:)), ...
    'VariableNames', {'Index','Time','Status'});
fprintf('\n============================================================\n');
fprintf('STEP CON ERRORE SOLVER -2 O -30\n');
fprintf('============================================================\n');
disp(Tfail_all);
fprintf('Totale step in errore : %d\n',   numel(idx_fail_all));
fprintf('Status -2  : %d\n', sum(status_int == -2));
fprintf('Status -30 : %d\n', sum(status_int == -30));
%%=========================================================================
% 4. SELEZIONE STEP DA ANALIZZARE
%==========================================================================
if isempty(idx_user)
    if fail_to_analyze < 1 || fail_to_analyze > numel(idx_fail_all)
        error('fail_to_analyze=%d fuori range [1,%d].', fail_to_analyze, numel(idx_fail_all));
    end
    idx_fail = idx_fail_all(fail_to_analyze);
else
    idx_fail = idx_user;
    if idx_fail < 1 || idx_fail > numel(status_vec)
        error('idx_user=%d fuori range [1,%d].', idx_fail, numel(status_vec));
    end
    if ~ismember(round(status_vec(idx_fail)), target_statuses)
        warning('idx_user=%d ha status=%g (non -2/-30).', idx_fail, status_vec(idx_fail));
    end
end
t_fail = t_status(idx_fail);
fprintf('\n============================================================\n');
fprintf('DEBUG VINCOLI NMPC 2 VEH\n');
fprintf('============================================================\n');
fprintf('Indice analizzato : %d\n',   idx_fail);
fprintf('Tempo simulazione : %.6f s\n', t_fail);
fprintf('Status effettivo  : %g\n',    status_vec(idx_fail));
fprintf('Errore            : %d / %d\n', fail_to_analyze, numel(idx_fail_all));
fprintf('============================================================\n\n');
%%=========================================================================
% 5. ESTRAZIONE CAMPIONE A t_fail
%==========================================================================
[Xraw,  idx_X,  t_X]  = sampleLoggedSignalAtTime(log_state_pred,  t_fail);
[Uraw,  idx_U,  t_U]  = sampleLoggedSignalAtTime(log_ctrl_pred,   t_fail);
[ODraw, idx_OD, t_OD] = sampleLoggedSignalAtTime(log_online_data, t_fail);
fprintf('Campione state_pred  : idx=%d  t=%.6f s\n', idx_X,  t_X);
fprintf('Campione ctrl_pred   : idx=%d  t=%.6f s\n', idx_U,  t_U);
fprintf('Campione online_data : idx=%d  t=%.6f s\n\n', idx_OD, t_OD);
%%=========================================================================
% 6. STATO CORRENTE (se disponibile)
%==========================================================================
x0_logged = [];
if ~isempty(log_states)
    try
        [x0_raw,~,~] = sampleLoggedSignalAtTime(log_states, t_fail);
        x0_logged = squeeze(x0_raw); x0_logged = x0_logged(:).';
        if numel(x0_logged) ~= nx, x0_logged = []; end
    catch, x0_logged = []; end
end
%%=========================================================================
% 7. FORMATTAZIONE PREDIZIONI E ONLINEDATA
%==========================================================================
X  = formatStatePrediction(Xraw,  N, nx, x0_logged);
U  = formatControlPrediction(Uraw, N, nu);
OD = formatOnlineData(ODraw, N, nod);
fprintf('Dimensioni estratte:\n');
fprintf('  X  = [%d x %d]\n', size(X,1),  size(X,2));
fprintf('  U  = [%d x %d]\n', size(U,1),  size(U,2));
fprintf('  OD = [%d x %d]\n\n', size(OD,1), size(OD,2));
if size(X,2)  ~= nx,  error('X ha %d colonne. Atteso nx=%d.',  size(X,2),  nx);  end
if size(U,2)  ~= nu,  error('U ha %d colonne. Atteso nu=%d.',  size(U,2),  nu);  end
if size(OD,2) ~= nod, error('OD ha %d colonne. Atteso nod=%d.',size(OD,2), nod); end
%%=========================================================================
% 8. CALCOLO RESIDUI VINCOLI
%==========================================================================
[Tviol_max, Tviol_node, Tall_max] = computeConstraintResiduals2veh(X, U, OD, tol);
%%=========================================================================
% 9. STAMPA RISULTATI
%==========================================================================
fprintf('Stato predetto al nodo 0:\n');
fprintf('  pos  = %.4f m   vel  = %.4f m/s   acc  = %.4f m/s^2\n', X(1,1),X(1,2),X(1,3));
fprintf('  pos2 = %.4f m   vel2 = %.4f m/s   acc2 = %.4f m/s^2\n\n', X(1,4),X(1,5),X(1,6));
if isempty(Tviol_max)
    fprintf('Nessun vincolo violato oltre tol=%.1e.\n', tol);
    fprintf('Vincoli piu'' vicini alla violazione:\n\n');
    disp(Tall_max(1:min(nshow,height(Tall_max)),:));
else
    fprintf('Vincoli violati (ordinati per massima violazione):\n\n');
    disp(Tviol_max(1:min(nshow,height(Tviol_max)),:));
    fprintf('\nViolazioni puntuali lungo la prediction:\n\n');
    disp(Tviol_node(1:min(nshow,height(Tviol_node)),:));
end
%%=========================================================================
% 10. CHECK NODO INIZIALE h=0
%==========================================================================
T0 = Tviol_node(Tviol_node.PredictionNode == 0,:);
fprintf('\n============================================================\n');
fprintf('CHECK NODO INIZIALE h = 0\n');
fprintf('============================================================\n');
if isempty(T0)
    fprintf('Nessuna violazione al nodo iniziale.\n');
    fprintf('La infeasibility e'' generata lungo la prediction.\n');
else
    fprintf('Vincoli gia'' violati al nodo iniziale:\n\n');
    disp(T0);
    fprintf('Stato iniziale incompatibile con i vincoli hard.\n');
end
%%=========================================================================
% FUNZIONI LOCALI
%==========================================================================
function [t, values] = getLoggedTimeAndValues(logsig)
    if isa(logsig,'timeseries')
        t = logsig.Time(:); values = logsig.Data; return;
    end
    if isstruct(logsig)
        if isfield(logsig,'time') && isfield(logsig,'signals')
            t = logsig.time(:); values = logsig.signals.values; return;
        end
        if isfield(logsig,'Time') && isfield(logsig,'Data')
            t = logsig.Time(:); values = logsig.Data; return;
        end
    end
    if isnumeric(logsig)
        values = logsig; t = (0:size(values,1)-1).'; return;
    end
    error('Formato log non riconosciuto.');
end
function [sample, idx, t_sample] = sampleLoggedSignalAtTime(logsig, t_query)
    [t, values] = getLoggedTimeAndValues(logsig);
    [~, idx] = min(abs(t - t_query));
    t_sample = t(idx);
    nt   = numel(t);
    dims = size(values);
    if isvector(values)
        v = values(:);
        if numel(v) == nt, sample = v(idx);
        else,              sample = v; end
        return;
    end
    time_dim = find(dims == nt, 1, 'first');
    if isempty(time_dim)
        error('Impossibile identificare la dimensione temporale.');
    end
    subs = repmat({':'},1,ndims(values));
    subs{time_dim} = idx;
    sample = squeeze(values(subs{:}));
end
function X = formatStatePrediction(raw, N, nx, x0_logged)
    nRows = N+1; nCols = nx;
    A = squeeze(raw);
    if isequal(size(A),[nRows nCols]), X = A; return; end
    if isequal(size(A),[nCols nRows]), X = A.'; return; end
    v = A(:);
    if numel(v) ~= nRows*nCols
        error('state_pred ha %d elementi. Attesi %d.', numel(v), nRows*nCols);
    end
    X1 = reshape(v, nCols, nRows).';
    X2 = reshape(v, nRows, nCols);
    if ~isempty(x0_logged)
        if norm(X1(1,:)-x0_logged) <= norm(X2(1,:)-x0_logged), X=X1; else, X=X2; end
    else
        if statePredictionScore2veh(X1) <= statePredictionScore2veh(X2), X=X1; else, X=X2; end
    end
end
function score = statePredictionScore2veh(X)
    score = 0;
    score = score + 1e3*sum(~isfinite(X(:)));
    score = score + 1e2*sum(abs(X(:,2)) > 80);  % vel1
    score = score + 1e2*sum(abs(X(:,5)) > 80);  % vel2
    score = score + 1e2*sum(abs(X(:,3)) > 20);  % acc1
    score = score + 1e2*sum(abs(X(:,6)) > 20);  % acc2
    score = score + 10 *sum(diff(X(:,1)) < -5); % pos1 non decresce
    score = score + 10 *sum(diff(X(:,4)) < -5); % pos2 non decresce
end
function U = formatControlPrediction(raw, N, nu)
    A = squeeze(raw);
    if isequal(size(A),[N   nu]), U=A;   return; end
    if isequal(size(A),[nu  N ]), U=A.'; return; end
    if isequal(size(A),[N+1 nu]), U=A;   return; end
    if isequal(size(A),[nu  N+1]), U=A.'; return; end
    v = A(:);
    if numel(v)==N*nu
        nRows=N;
    elseif numel(v)==(N+1)*nu
        nRows=N+1;
    else
        error('ctrl_pred ha %d elementi. Attesi %d o %d.',numel(v),N*nu,(N+1)*nu);
    end
    U1 = reshape(v,nu,nRows).';
    U2 = reshape(v,nRows,nu);
    s1 = 1e3*sum(~isfinite(U1(:))) + 1e2*sum(abs(U1(:))>10);
    s2 = 1e3*sum(~isfinite(U2(:))) + 1e2*sum(abs(U2(:))>10);
    if s1<=s2, U=U1; else, U=U2; end
end
function OD = formatOnlineData(raw, N, nod)
    A = squeeze(raw);
    if isequal(size(A),[N+1 nod]), OD=A;   return; end
    if isequal(size(A),[nod N+1]), OD=A.'; return; end
    if isequal(size(A),[N   nod]), OD=A;   return; end
    if isequal(size(A),[nod N  ]), OD=A.'; return; end
    v = A(:);
    if numel(v)==(N+1)*nod
        nRows=N+1;
    elseif numel(v)==N*nod
        nRows=N;
    else
        error('online_data ha %d elementi. Attesi %d o %d.',numel(v),(N+1)*nod,N*nod);
    end
    OD1 = reshape(v,nod,nRows).';
    OD2 = reshape(v,nRows,nod);
    if onlineDataScore2veh(OD1) <= onlineDataScore2veh(OD2), OD=OD1; else, OD=OD2; end
end
function score = onlineDataScore2veh(OD)
    score = 0;
    score = score + 1e4*sum(~isfinite(OD(:)));
    if size(OD,2) < 42, score = score + 1e8; return; end
    % Flag attesi in [0,1]: testa TL (1:4), x_stop/x_dwell/w_stop (6:8), coda TL (14:17)
    flag_cols = [];
    for base = [0 21]
        flag_cols = [flag_cols, base+(1:4), base+(6:8), base+(14:17)]; %#ok<AGROW>
    end
    flags = OD(:,flag_cols);
    score = score + 10*sum(flags(:) < -0.2);
    score = score + 10*sum(flags(:) >  1.2);
    % Vmax e k_road devono essere ragionevoli
    for base = [0 21]
        Vm = OD(:,base+10);
        kr = OD(:,base+9);
        score = score + 100*sum(Vm <= 0 | Vm > 100);
        score = score + 100*sum(abs(kr) > 1);
    end
end
function [Tviol_max, Tviol_node, Tall_max] = computeConstraintResiduals2veh(X, U, OD, tol)
    %--------------------------------------------------------------
    % Parametri coerenti con ACADO_2veh_slack.m
    %--------------------------------------------------------------
    s_max          = 5200;
    Ax_min         = -1.5;
    Ax_max         =  1.5;
    Ay_max         =  2.0;
    jerk_min       = -0.5;
    jerk_max       =  0.5;
    L_platoon      =  7.8;
    eps_stop_back  =  0.5;
    eps_stop_front =  0.5;
    eps_stop_v     =  0.1;
    eps_stop_a     =  0.3;
    V_max_static   = 50/3.6;
    d_safe_TL      =  0.0;
    v_eps          =  0.05;
    gap_min        =  0.5;
    sJmin_max     =  1.0;
    %--------------------------------------------------------------
    % Allineamento dimensioni
    %--------------------------------------------------------------
    nNode  = min(size(X,1), size(OD,1));
    X      = X(1:nNode,:);
    OD     = OD(1:nNode,:);
    nNodeU = min(nNode, size(U,1));
    %--------------------------------------------------------------
    % Stati
    %--------------------------------------------------------------
    pos  = X(:,1); vel  = X(:,2); acc  = X(:,3);
    pos2 = X(:,4); vel2 = X(:,5); acc2 = X(:,6);
    %--------------------------------------------------------------
    % Controlli: U = [jerk jerk2 s_Jmin1 s_Jmin2]
    %
    % In ACADO il limite inferiore sul jerk e' soft:
    %   jerk  - jerk_min + s_Jmin1 >= 0
    %   jerk2 - jerk_min + s_Jmin2 >= 0
    % con:
    %   0 <= s_Jmin1 <= sJmin_max
    %   0 <= s_Jmin2 <= sJmin_max
    %--------------------------------------------------------------
    jerk    = U(1:nNodeU,1);
    jerk2   = U(1:nNodeU,2);
    s_Jmin1 = U(1:nNodeU,3);
    s_Jmin2 = U(1:nNodeU,4);
    %--------------------------------------------------------------
    % OnlineData
    %--------------------------------------------------------------
    D1 = parseVehicleOD(OD,1);
    D2 = parseVehicleOD(OD,2);
    %--------------------------------------------------------------
    % Strutture accumulo
    %--------------------------------------------------------------
    C_name = {}; C_node = []; C_max = [];
    N_name = {}; N_node = []; N_res = [];
    function addConstraint(name, r)
        r = full(r(:));
        [rmax, imax] = max(r);
        C_name{end+1,1} = name;
        C_node(end+1,1) = imax - 1;
        C_max(end+1,1)  = rmax;
        idx_v = find(r > tol);
        for kk = 1:numel(idx_v)
            N_name{end+1,1} = name;
            N_node(end+1,1) = idx_v(kk) - 1;
            N_res(end+1,1)  = r(idx_v(kk));
        end
    end
    %--------------------------------------------------------------
    % 1. Accelerazione longitudinale
    %--------------------------------------------------------------
    addConstraint('veh1 acc <= Ax_max',  acc  - Ax_max);
    addConstraint('veh1 acc >= Ax_min',  Ax_min - acc);
    addConstraint('veh2 acc <= Ax_max',  acc2 - Ax_max);
    addConstraint('veh2 acc >= Ax_min',  Ax_min - acc2);
    %--------------------------------------------------------------
    % 2. Jerk
    %--------------------------------------------------------------
    % Limite superiore hard, coerente con:
    %   ocp.subjectTo(jerk  <= jerk_max)
    %   ocp.subjectTo(jerk2 <= jerk_max)
    addConstraint('veh1 jerk <= jerk_max', jerk  - jerk_max);
    addConstraint('veh2 jerk <= jerk_max', jerk2 - jerk_max);

    % Limite inferiore soft, coerente con:
    %   ocp.subjectTo(jerk  - jerk_min + s_Jmin1 >= 0)
    %   ocp.subjectTo(jerk2 - jerk_min + s_Jmin2 >= 0)
    % Residuo in forma <= 0:
    %   jerk_min - jerk - s_Jmin <= 0
    addConstraint('veh1 jerk - jerk_min + s_Jmin1 >= 0', ...
        jerk_min - jerk - s_Jmin1);
    addConstraint('veh2 jerk2 - jerk_min + s_Jmin2 >= 0', ...
        jerk_min - jerk2 - s_Jmin2);

    % Bound sugli slack, coerenti con ACADO.
    addConstraint('veh1 s_Jmin1 >= 0', -s_Jmin1);
    addConstraint('veh2 s_Jmin2 >= 0', -s_Jmin2);
    addConstraint('veh1 s_Jmin1 <= sJmin_max', s_Jmin1 - sJmin_max);
    addConstraint('veh2 s_Jmin2 <= sJmin_max', s_Jmin2 - sJmin_max);
    %--------------------------------------------------------------
    % 3. Accelerazione laterale
    %--------------------------------------------------------------
    addConstraint('veh1 vel^2*k_road <= Ay_max',  vel.^2  .* D1.k_road - Ay_max);
    addConstraint('veh2 vel2^2*k_road_2 <= Ay_max', vel2.^2 .* D2.k_road - Ay_max);
    %--------------------------------------------------------------
    % 4. Velocita'
    %--------------------------------------------------------------
    addConstraint('veh1 vel <= Vmax',    vel  - D1.Vmax);
    addConstraint('veh1 vel >= -v_eps',  -v_eps - vel);
    addConstraint('veh2 vel2 <= Vmax_2', vel2 - D2.Vmax);
    addConstraint('veh2 vel2 >= -v_eps', -v_eps - vel2);
    %--------------------------------------------------------------
    % 5. Gap minimo 1-2
    %--------------------------------------------------------------
    gap12 = pos - pos2 - L_platoon;
    addConstraint('gap12 >= gap_min', gap_min - gap12);
    %--------------------------------------------------------------
    % 6. Fermate bus
    %--------------------------------------------------------------
    addStopConstraints(1, pos,  vel,  acc,  D1);
    addStopConstraints(2, pos2, vel2, acc2, D2);
    %--------------------------------------------------------------
    % 7. Semafori (4 slot per veicolo, testa e coda)
    %--------------------------------------------------------------
    addTLConstraints(1, pos,  D1);
    addTLConstraints(2, pos2, D2);
    %--------------------------------------------------------------
    % Tabelle finali
    %--------------------------------------------------------------
    Tall_max = table(C_name, C_node, C_max, ...
        'VariableNames', {'Constraint','PredictionNode','MaxResidual'});
    Tall_max  = sortrows(Tall_max, 'MaxResidual', 'descend');
    Tviol_max = Tall_max(Tall_max.MaxResidual > tol, :);
    Tviol_node = table(N_name, N_node, N_res, ...
        'VariableNames', {'Constraint','PredictionNode','Residual'});
    if ~isempty(Tviol_node)
        Tviol_node = sortrows(Tviol_node, 'Residual', 'descend');
    end
    %--------------------------------------------------------------
    % Helper: vincoli fermate
    %--------------------------------------------------------------
    function addStopConstraints(id, pos_i, vel_i, acc_i, D)
        pfx = sprintf('veh%d',id);
        % Approccio
        addConstraint([pfx ' stop approach'], ...
            pos_i - (D.s_stop.*(1-D.x_stop) + s_max.*D.x_stop));
        % Dwell box
        addConstraint([pfx ' dwell front'], ...
            pos_i - ((D.s_stop+eps_stop_front).*D.x_dwell + s_max.*(1-D.x_dwell)));
        addConstraint([pfx ' dwell back'], ...
            (D.s_stop-eps_stop_back).*D.x_dwell - s_max.*(1-D.x_dwell) - pos_i);
        % Dwell vel
        addConstraint([pfx ' dwell vel'], ...
            vel_i - (eps_stop_v.*D.x_dwell + V_max_static.*(1-D.x_dwell)));
        % Dwell acc
        addConstraint([pfx ' dwell acc max'], ...
            acc_i - (eps_stop_a.*D.x_dwell + Ax_max.*(1-D.x_dwell)));
        addConstraint([pfx ' dwell acc min'], ...
            -(acc_i + (eps_stop_a.*D.x_dwell + abs(Ax_min).*(1-D.x_dwell))));
    end
    %--------------------------------------------------------------
    % Helper: vincoli semafori
    %--------------------------------------------------------------
    function addTLConstraints(id, pos_i, D)
        pfx = sprintf('veh%d',id);
        for kk = 1:4
            sTL = D.s_TL_active(:,kk);
            % Testa
            addConstraint(sprintf('%s TLslot%02d head',pfx,kk), ...
                pos_i - ((sTL-d_safe_TL).*(1-D.x_TL(:,kk)) + s_max.*D.x_TL(:,kk)));
            % Coda
            addConstraint(sprintf('%s TLslot%02d tail',pfx,kk), ...
                (pos_i-L_platoon) - ((sTL-d_safe_TL).*(1-D.x_TL_tail(:,kk)) + s_max.*D.x_TL_tail(:,kk)));
        end
    end
end
function D = parseVehicleOD(OD, veh_id)
    base          = (veh_id-1)*21;
    D.x_TL        = OD(:, base+1  : base+4);
    D.s_stop      = OD(:, base+5);
    D.x_stop      = OD(:, base+6);
    D.x_dwell     = OD(:, base+7);
    D.w_stop      = OD(:, base+8);
    D.k_road      = OD(:, base+9);
    D.Vmax        = OD(:, base+10);
    D.dt_schd     = OD(:, base+11);
    D.s_st        = OD(:, base+12);
    D.s_hor       = OD(:, base+13);
    D.x_TL_tail   = OD(:, base+14 : base+17);
    D.s_TL_active = OD(:, base+18 : base+21);
end

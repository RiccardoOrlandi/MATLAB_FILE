function diagSpike = analyze_execTime_spikes_3veh(out,cfg)
%==========================================================================
% ANALISI SPIKE EXECUTION TIME - NMPC 3 VEHICLES / 4 TL
%==========================================================================
% Obiettivo:
%   - identificare gli spike di execution time;
%   - stampare stato [s,v,a] e jerk di ciascun veicolo;
%   - ricostruire gap, stato solver, objective, iterazioni;
%   - identificare i vincoli attivi o quasi attivi;
%   - classificare la causa probabile dello spike.
%
% Uso base:
%   diagSpike = analyze_execTime_spikes_3veh(out);
%
% Uso con configurazione:
%   cfg.spikePrctile = 95;
%   cfg.maxSpikes = 20;
%   cfg.window = 10;
%   diagSpike = analyze_execTime_spikes_3veh(out,cfg);
%
% Output:
%   diagSpike.summary           tabella spike principali
%   diagSpike.activeConstraints tabella vincoli attivi / quasi attivi
%   diagSpike.cfg               configurazione usata
%==========================================================================

if nargin < 2
    cfg = struct();
end

cfg = local_default_cfg(cfg);

%% ========================================================================
%  1. Estrazione segnali principali
%  ========================================================================

[t_exec,T_exec] = local_get_signal(out,{'executionTime'});
T_exec = local_to_col(T_exec);

[t_states,X_raw] = local_get_signal(out,{'states'});
X = local_signal_to_matrix(X_raw,numel(t_states));

if size(X,2) < 9
    error('out.states deve contenere almeno 9 colonne: [s1 v1 a1 s2 v2 a2 s3 v3 a3].');
end

X = X(:,1:9);

[t_u,U_raw,has_u] = local_try_get_signal(out,{'u','ctrl','control'});
if has_u
    U = local_signal_to_matrix(U_raw,numel(t_u));
else
    t_u = t_states;
    U = nan(numel(t_states),3);
end

if size(U,2) < 3
    U(:,end+1:3) = nan;
end

[t_status,status_raw,has_status] = local_try_get_signal(out,{'status','solver_status'});
if has_status
    solver_status = local_to_col(status_raw);
else
    t_status = t_exec;
    solver_status = nan(size(t_exec));
end

[t_J,J_raw,has_J] = local_try_get_signal(out,{'Objective_Value','objective','J'});
if has_J
    J = local_to_col(J_raw);
else
    t_J = t_exec;
    J = nan(size(t_exec));
end

[t_iter,iter_raw,has_iter] = local_try_get_signal(out,{'nIteration','nIterations','iterations','iter'});
if has_iter
    nIter = local_to_col(iter_raw);
else
    t_iter = t_exec;
    nIter = nan(size(t_exec));
end

%% ========================================================================
%  2. Identificazione spike execution time
%  ========================================================================

exec_med = local_nanmedian(T_exec);
exec_mad = local_nanmedian(abs(T_exec - exec_med));
exec_p95 = local_nanpercentile(T_exec,95);
exec_p99 = local_nanpercentile(T_exec,99);

if isempty(cfg.spikeThreshold)
    thr_prc = local_nanpercentile(T_exec,cfg.spikePrctile);
    thr_mad = exec_med + cfg.madFactor*max(exec_mad,eps);
    spikeThreshold = max(thr_prc,thr_mad);
else
    spikeThreshold = cfg.spikeThreshold;
end

spikeIdx = local_detect_spikes(t_exec,T_exec,spikeThreshold,cfg.minPeakDistance);

if numel(spikeIdx) > cfg.maxSpikes
    [~,ord] = sort(T_exec(spikeIdx),'descend');
    spikeIdx = spikeIdx(ord(1:cfg.maxSpikes));
    spikeIdx = sort(spikeIdx);
end

nSpikes = numel(spikeIdx);

fprintf('\n============================================================\n');
fprintf('ANALISI SPIKE EXECUTION TIME NMPC\n');
fprintf('============================================================\n');
fprintf('Median execution time  : %.6f s\n',exec_med);
fprintf('MAD execution time     : %.6f s\n',exec_mad);
fprintf('95 percentile          : %.6f s\n',exec_p95);
fprintf('99 percentile          : %.6f s\n',exec_p99);
fprintf('Threshold spike        : %.6f s\n',spikeThreshold);
fprintf('Numero spike trovati   : %d\n',nSpikes);

if nSpikes == 0
    warning('Nessuno spike rilevato con la soglia corrente.');
    diagSpike = struct();
    diagSpike.summary = table();
    diagSpike.activeConstraints = table();
    diagSpike.cfg = cfg;
    return
end

%% ========================================================================
%  3. Analisi dettagliata spike
%  ========================================================================

summaryRows = cell(nSpikes,1);
activeRows  = {};

for kk = 1:nSpikes

    idxE = spikeIdx(kk);
    t0   = t_exec(idxE);

    x_now = local_sample_signal(t_states,X,t0,'linear');
    u_now = local_sample_signal(t_u,U(:,1:3),t0,'nearest');

    status_now = local_sample_signal(t_status,solver_status,t0,'nearest');
    J_now      = local_sample_signal(t_J,J,t0,'nearest');
    iter_now   = local_sample_signal(t_iter,nIter,t0,'nearest');

    s1 = x_now(1); v1 = x_now(2); a1 = x_now(3);
    s2 = x_now(4); v2 = x_now(5); a2 = x_now(6);
    s3 = x_now(7); v3 = x_now(8); a3 = x_now(9);

    u1 = u_now(1); u2 = u_now(2); u3 = u_now(3);

    gap12 = s1 - s2 - cfg.L_platoon;
    gap23 = s2 - s3 - cfg.L_platoon;

    [Xhor,hasXhor]  = local_get_horizon_from_out(out,cfg.statePredNames,t0,9);
    [Uhor,hasUhor]  = local_get_horizon_from_out(out,cfg.ctrlPredNames,t0,3);
    [ODhor,hasODhor] = local_get_horizon_from_out(out,cfg.onlineDataNames,t0,63);

    if ~hasXhor
        Xhor = x_now;
    end

    if ~hasUhor
        Uhor = u_now;
    end

    if ~hasODhor
        ODhor = nan(size(Xhor,1),63);
    end

    nH = size(Xhor,1);

    if size(Uhor,1) < nH
        Uhor(end+1:nH,:) = nan;
    end

    if size(ODhor,1) < nH
        ODhor(end+1:nH,:) = nan;
    end

    if size(Uhor,2) < 3
        Uhor(:,end+1:3) = nan;
    end

    if size(ODhor,2) < 63
        ODhor(:,end+1:63) = nan;
    end

    if hasXhor
        x0PredMismatch = norm(Xhor(1,1:9) - x_now(1:9),2);
    else
        x0PredMismatch = nan;
    end

    rows_k = local_eval_active_constraints( ...
        kk,t0,Xhor(:,1:9),Uhor(:,1:3),ODhor(:,1:63),cfg);

    activeRows = [activeRows; rows_k]; %#ok<AGROW>

    nActive = size(rows_k,1);
    nViol = local_count_violations(rows_k,cfg.violationTol);

    likelyCause = local_classify_cause( ...
        x_now,u_now,gap12,gap23,rows_k,status_now,J_now,iter_now,cfg);

    summaryRows{kk} = { ...
        kk,idxE,t0,T_exec(idxE), ...
        T_exec(idxE)/max(exec_med,eps), ...
        (T_exec(idxE)-exec_med)/max(exec_mad,eps), ...
        status_now,iter_now,J_now, ...
        s1,v1*3.6,a1,u1, ...
        s2,v2*3.6,a2,u2, ...
        s3,v3*3.6,a3,u3, ...
        gap12,gap23,min(gap12,gap23)-cfg.gap_min, ...
        nActive,nViol,x0PredMismatch,likelyCause};

end

summaryNames = { ...
    'SpikeID','IdxExec','Time_s','ExecutionTime_s', ...
    'ExecOverMedian','ExecZmad', ...
    'SolverStatus','NIteration','Objective', ...
    's1_m','v1_kmh','a1_mps2','u1_mps3', ...
    's2_m','v2_kmh','a2_mps2','u2_mps3', ...
    's3_m','v3_kmh','a3_mps2','u3_mps3', ...
    'gap12_m','gap23_m','minGapMargin_m', ...
    'NActiveConstraints','NViolations','x0PredMismatch','LikelyCause'};

T_summary = cell2table(vertcat(summaryRows{:}),'VariableNames',summaryNames);

if isempty(activeRows)
    T_active = table();
else
    activeNames = {'SpikeID','HorizonNode','PredTime_s','Vehicle', ...
                   'Constraint','Value','Bound','Margin','Unit','Level'};
    T_active = cell2table(activeRows,'VariableNames',activeNames);
end

%% ========================================================================
%  4. Stampa risultati
%  ========================================================================

fprintf('\n============================================================\n');
fprintf('TABELLA SPIKE\n');
fprintf('============================================================\n');
disp(T_summary)

fprintf('\n============================================================\n');
fprintf('VINCOLI ATTIVI / QUASI ATTIVI\n');
fprintf('============================================================\n');

if isempty(T_active)
    fprintf('Nessun vincolo attivo rilevato con la tolleranza impostata.\n');
else
    disp(T_active)
end

%% ========================================================================
%  5. Plot diagnostici
%  ========================================================================

local_plot_overview(t_exec,T_exec,spikeIdx,spikeThreshold, ...
                    t_states,X,t_status,solver_status,t_J,J,t_iter,nIter,cfg);

nLocal = min(cfg.nLocalPlots,nSpikes);
for kk = 1:nLocal
    local_plot_spike_window(kk,spikeIdx(kk), ...
        t_exec,T_exec,t_states,X,t_u,U, ...
        t_status,solver_status,t_J,J,t_iter,nIter,cfg);
end

%% ========================================================================
%  6. Output
%  ========================================================================

diagSpike = struct();
diagSpike.summary = T_summary;
diagSpike.activeConstraints = T_active;
diagSpike.spikeIdx = spikeIdx;
diagSpike.threshold = spikeThreshold;
diagSpike.cfg = cfg;

assignin('base','diagSpike',diagSpike);
assignin('base','SpikeSummary',T_summary);
assignin('base','ActiveConstraints',T_active);

fprintf('\nOggetti salvati nel workspace:\n');
fprintf('  diagSpike\n');
fprintf('  SpikeSummary\n');
fprintf('  ActiveConstraints\n');

end

%% =========================================================================
%  CONFIGURAZIONE
% =========================================================================

function cfg = local_default_cfg(cfg)

cfg = local_set_default(cfg,'L_platoon',8);
cfg = local_set_default(cfg,'s_max',5200);

cfg = local_set_default(cfg,'Ts',1);
cfg = local_set_default(cfg,'N',50);

cfg = local_set_default(cfg,'Ax_min',-1.5);
cfg = local_set_default(cfg,'Ax_max', 1.5);
cfg = local_set_default(cfg,'Ay_max', 2.0);

cfg = local_set_default(cfg,'jerk_min',-0.5);
cfg = local_set_default(cfg,'jerk_max', 0.5);

cfg = local_set_default(cfg,'gap_min',0.5);
cfg = local_set_default(cfg,'v_eps',0.01);

cfg = local_set_default(cfg,'eps_stop_back',0.5);
cfg = local_set_default(cfg,'eps_stop_front',0.5);
cfg = local_set_default(cfg,'eps_stop_v',0.1);
cfg = local_set_default(cfg,'eps_stop_a',0.3);

cfg = local_set_default(cfg,'d_safe_TL',0);

cfg = local_set_default(cfg,'Vmax_default_mps',[50 50 50]/3.6);
cfg = local_set_default(cfg,'convertVmaxFromKmh','auto');

cfg = local_set_default(cfg,'spikePrctile',95);
cfg = local_set_default(cfg,'spikeThreshold',[]);
cfg = local_set_default(cfg,'madFactor',6);
cfg = local_set_default(cfg,'minPeakDistance',3);
cfg = local_set_default(cfg,'maxSpikes',30);

cfg = local_set_default(cfg,'activeTolAbs',1e-2);
cfg = local_set_default(cfg,'activeTolGap',5e-2);
cfg = local_set_default(cfg,'activeTolSpeed',5e-2);
cfg = local_set_default(cfg,'activeTolAcc',5e-2);
cfg = local_set_default(cfg,'activeTolJerk',5e-2);
cfg = local_set_default(cfg,'violationTol',1e-5);

cfg = local_set_default(cfg,'window',10);
cfg = local_set_default(cfg,'nLocalPlots',6);

cfg = local_set_default(cfg,'s_TL',[]);
cfg = local_set_default(cfg,'s_stop',[]);

cfg = local_set_default(cfg,'statePredNames', ...
    {'state_pred','states_pred','x_pred','X_pred','xPred','XPred'});

cfg = local_set_default(cfg,'ctrlPredNames', ...
    {'ctrl_pred','control_pred','u_pred','U_pred','uPred','UPred'});

cfg = local_set_default(cfg,'onlineDataNames', ...
    {'online_data','OnlineData','onlineData','onlinedata', ...
     'OD','od','od_array','OD_array','OnlineDataArray'});

end

function cfg = local_set_default(cfg,name,value)
if ~isfield(cfg,name) || isempty(cfg.(name))
    cfg.(name) = value;
end
end

%% =========================================================================
%  VALUTAZIONE VINCOLI ATTIVI
% =========================================================================

function rows = local_eval_active_constraints(spikeID,t0,Xhor,Uhor,ODhor,cfg)

rows = {};

nH = size(Xhor,1);

for h = 1:nH

    x = Xhor(h,:);
    u = Uhor(min(h,size(Uhor,1)),:);
    od = ODhor(min(h,size(ODhor,1)),:);

    predTime = t0 + (h-1)*cfg.Ts;

    s = [x(1), x(4), x(7)];
    v = [x(2), x(5), x(8)];
    a = [x(3), x(6), x(9)];
    jerk = [u(1), u(2), u(3)];

    gap12 = s(1) - s(2) - cfg.L_platoon;
    gap23 = s(2) - s(3) - cfg.L_platoon;

    rows = local_add_if_active(rows,spikeID,h,predTime,0, ...
        'gap12 >= gap_min',gap12,cfg.gap_min,gap12-cfg.gap_min,'m',cfg.activeTolGap);

    rows = local_add_if_active(rows,spikeID,h,predTime,0, ...
        'gap23 >= gap_min',gap23,cfg.gap_min,gap23-cfg.gap_min,'m',cfg.activeTolGap);

    for veh = 1:3

        blk = (veh-1)*21;

        odVeh = od(blk+1:blk+21);

        xTL_head = odVeh(1:4);
        s_stop   = odVeh(5);
        x_stop   = odVeh(6);
        x_dwell  = odVeh(7);
        w_stop   = odVeh(8);
        k_road   = odVeh(9);
        Vmax     = local_get_vmax(odVeh(10),veh,cfg);
        xTL_tail = odVeh(14:17);
        sTL_act  = odVeh(18:21);

        if ~isfinite(k_road)
            k_road = 0;
        end

        k_road = abs(k_road);

        %--------------------------------------------------------------
        % Vincoli accelerazione
        %--------------------------------------------------------------
        rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
            'acc <= Ax_max',a(veh),cfg.Ax_max,cfg.Ax_max-a(veh),'m/s^2',cfg.activeTolAcc);

        rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
            'acc >= Ax_min',a(veh),cfg.Ax_min,a(veh)-cfg.Ax_min,'m/s^2',cfg.activeTolAcc);

        %--------------------------------------------------------------
        % Vincoli jerk
        %--------------------------------------------------------------
        if isfinite(jerk(veh))
            rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                'jerk <= jerk_max',jerk(veh),cfg.jerk_max,cfg.jerk_max-jerk(veh),'m/s^3',cfg.activeTolJerk);

            rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                'jerk >= jerk_min',jerk(veh),cfg.jerk_min,jerk(veh)-cfg.jerk_min,'m/s^3',cfg.activeTolJerk);
        end

        %--------------------------------------------------------------
        % Vincoli velocità
        %--------------------------------------------------------------
        rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
            'vel <= Vmax',v(veh),Vmax,Vmax-v(veh),'m/s',cfg.activeTolSpeed);

        rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
            'vel >= -v_eps',v(veh),-cfg.v_eps,v(veh)+cfg.v_eps,'m/s',cfg.activeTolSpeed);

        %--------------------------------------------------------------
        % Accelerazione laterale
        %--------------------------------------------------------------
        ay_est = v(veh)^2*k_road;

        rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
            'v^2*k_road <= Ay_max',ay_est,cfg.Ay_max,cfg.Ay_max-ay_est,'m/s^2',cfg.activeTolAcc);

        %--------------------------------------------------------------
        % Bus stop: approach
        % pos <= s_stop*(1-x_stop) + s_max*x_stop
        %--------------------------------------------------------------
        if isfinite(s_stop) && isfinite(x_stop)
            bound_stop = s_stop*(1-x_stop) + cfg.s_max*x_stop;
            margin = bound_stop - s(veh);

            if x_stop < 0.5 || margin < cfg.activeTolAbs
                rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                    'stop approach: pos <= bound_stop',s(veh),bound_stop,margin,'m',cfg.activeTolAbs);
            end
        end

        %--------------------------------------------------------------
        % Bus stop: dwell box
        %--------------------------------------------------------------
        if isfinite(s_stop) && isfinite(x_dwell)

            dwellUpper = (s_stop + cfg.eps_stop_front)*x_dwell + cfg.s_max*(1-x_dwell);
            dwellLower = (s_stop - cfg.eps_stop_back )*x_dwell - cfg.s_max*(1-x_dwell);

            marginUpper = dwellUpper - s(veh);
            marginLower = s(veh) - dwellLower;

            if x_dwell > 0.5 || marginUpper < cfg.activeTolAbs
                rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                    'dwell pos upper',s(veh),dwellUpper,marginUpper,'m',cfg.activeTolAbs);
            end

            if x_dwell > 0.5 || marginLower < cfg.activeTolAbs
                rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                    'dwell pos lower',s(veh),dwellLower,marginLower,'m',cfg.activeTolAbs);
            end

            dwellVelBound = cfg.eps_stop_v*x_dwell + (50/3.6)*(1-x_dwell);
            dwellAccBound = cfg.eps_stop_a*x_dwell + cfg.Ax_max*(1-x_dwell);
            dwellAccMinAbs = cfg.eps_stop_a*x_dwell + abs(cfg.Ax_min)*(1-x_dwell);

            rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                'dwell vel upper',v(veh),dwellVelBound,dwellVelBound-v(veh),'m/s',cfg.activeTolSpeed);

            rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                'dwell acc upper',a(veh),dwellAccBound,dwellAccBound-a(veh),'m/s^2',cfg.activeTolAcc);

            rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                'dwell acc lower',a(veh),-dwellAccMinAbs,a(veh)+dwellAccMinAbs,'m/s^2',cfg.activeTolAcc);

            if x_dwell > 0.5 && isfinite(w_stop)
                rows = local_add_context(rows,spikeID,h,predTime,veh, ...
                    'CONTEXT: dwell_active = 1',x_dwell,1,nan,'-','context');
            end
        end

        %--------------------------------------------------------------
        % Traffic light: testa e coda
        % flag = 0 => rosso, bound = s_TL
        % flag = 1 => verde, bound = s_max
        %--------------------------------------------------------------
        for jj = 1:4

            if isfinite(xTL_head(jj)) && isfinite(sTL_act(jj))
                boundTL = (sTL_act(jj)-cfg.d_safe_TL)*(1-xTL_head(jj)) + cfg.s_max*xTL_head(jj);
                marginTL = boundTL - s(veh);

                if xTL_head(jj) < 0.5 || marginTL < cfg.activeTolAbs
                    cname = sprintf('TL%d head: pos <= bound',jj);
                    rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                        cname,s(veh),boundTL,marginTL,'m',cfg.activeTolAbs);
                end
            end

            if isfinite(xTL_tail(jj)) && isfinite(sTL_act(jj))
                tailPos = s(veh) - cfg.L_platoon;
                boundTLtail = (sTL_act(jj)-cfg.d_safe_TL)*(1-xTL_tail(jj)) + cfg.s_max*xTL_tail(jj);
                marginTLtail = boundTLtail - tailPos;

                if xTL_tail(jj) < 0.5 || marginTLtail < cfg.activeTolAbs
                    cname = sprintf('TL%d tail: pos_tail <= bound',jj);
                    rows = local_add_if_active(rows,spikeID,h,predTime,veh, ...
                        cname,tailPos,boundTLtail,marginTLtail,'m',cfg.activeTolAbs);
                end
            end

        end

    end
end

end


function rows = local_add_if_active(rows,spikeID,h,predTime,veh,cname,value,bound,margin,unit,tol)

if ~isfinite(value) || ~isfinite(bound) || ~isfinite(margin)
    return
end

violationTol = 1e-3;   % tolleranza numerica robusta

if margin < -violationTol
    level = 'violated';
elseif margin <= tol
    level = 'active';
else
    return
end

rows(end+1,:) = {spikeID,h,predTime,veh,cname,value,bound,margin,unit,level};

end
function rows = local_add_context(rows,spikeID,h,predTime,veh,cname,value,bound,margin,unit,level)
rows(end+1,:) = {spikeID,h,predTime,veh,cname,value,bound,margin,unit,level};
end

function nViol = local_count_violations(rows,violationTol)

nViol = 0;

if isempty(rows)
    return
end

for ii = 1:size(rows,1)
    margin = rows{ii,8};
    if isnumeric(margin) && isfinite(margin) && margin < -violationTol
        nViol = nViol + 1;
    end
end

end

function Vmax = local_get_vmax(raw,veh,cfg)

if nargin < 3 || veh > numel(cfg.Vmax_default_mps)
    Vmax = 50/3.6;
else
    Vmax = cfg.Vmax_default_mps(veh);
end

if ~isfinite(raw)
    return
end

switch lower(cfg.convertVmaxFromKmh)
    case 'always'
        Vmax = raw/3.6;
    case 'never'
        Vmax = raw;
    otherwise
        if raw > 25
            Vmax = raw/3.6;
        else
            Vmax = raw;
        end
end

end

%% =========================================================================
%  CLASSIFICAZIONE CAUSA
% =========================================================================

function cause = local_classify_cause(x,u,gap12,gap23,rows,status,J,iter,cfg)

s = [x(1),x(4),x(7)];
v = [x(2),x(5),x(8)];
a = [x(3),x(6),x(9)];

nearStopSpeed = any(abs(v) < 0.15);
nearGap = min(gap12,gap23) <= cfg.gap_min + 0.10;

hasDwell = local_rows_contain(rows,'dwell_active');
hasTL    = local_rows_contain(rows,'TL');
hasStop  = local_rows_contain(rows,'stop approach') || local_rows_contain(rows,'dwell');
hasGap   = local_rows_contain(rows,'gap12') || local_rows_contain(rows,'gap23');
hasJerk  = local_rows_contain(rows,'jerk');
hasAcc   = local_rows_contain(rows,'acc');
hasVel0  = local_rows_contain(rows,'vel >= -v_eps');
hasViol = local_has_real_violation(rows,1e-3);

if hasViol
    cause = 'vincolo violato: possibile infeasibilita numerica o set vincoli incompatibile';
elseif hasDwell && nearStopSpeed
    cause = 'uscita/ripartenza da dwell: cambio set vincoli e ricalcolo traiettoria';
elseif hasStop && nearStopSpeed
    cause = 'fermata bus attiva con velocita quasi nulla';
elseif hasTL && nearStopSpeed
    cause = 'hold/ripartenza da semaforo rosso o transizione TL';
elseif hasGap || nearGap
    cause = 'gap safety prossimo al minimo: interazione veicoli critica';
elseif hasJerk && hasAcc
    cause = 'saturazione simultanea jerk/accelerazione';
elseif hasJerk
    cause = 'jerk saturo: richiesta variazione accelerazione aggressiva';
elseif hasAcc
    cause = 'accelerazione al limite: manovra dinamicamente vincolata';
elseif hasVel0
    cause = 'velocita nulla/quasi nulla: possibile riattivazione dinamica';
elseif isfinite(status) && status < 0
    cause = 'solver status negativo: controllare vincoli attivi e inizializzazione';
elseif isfinite(iter) && iter > 0
    cause = 'aumento iterazioni solver senza vincolo dominante evidente';
elseif isfinite(J) && J > 0
    cause = 'objective elevata senza vincolo dominante evidente';
else
    cause = 'spike isolato: verificare online data e warm start';
end

end

function tf = local_rows_contain(rows,pattern)

tf = false;

if isempty(rows)
    return
end

for ii = 1:size(rows,1)
    cname = rows{ii,5};
    if ischar(cname) && contains(lower(cname),lower(pattern))
        tf = true;
        return
    end
end

end

function tf = local_rows_contain_level(rows,pattern)

tf = false;

if isempty(rows)
    return
end

for ii = 1:size(rows,1)
    lev = rows{ii,10};
    if ischar(lev) && contains(lower(lev),lower(pattern))
        tf = true;
        return
    end
end

end
function tf = local_has_real_violation(rows,tol)

tf = false;

if isempty(rows)
    return
end

for ii = 1:size(rows,1)
    margin = rows{ii,8};

    if isnumeric(margin) && isfinite(margin) && margin < -tol
        tf = true;
        return
    end
end

end

%% =========================================================================
%  PLOT
% =========================================================================

function local_plot_overview(t_exec,T_exec,spikeIdx,thr, ...
    t_states,X,t_status,status,t_J,J,t_iter,nIter,cfg)

figure('Name','Execution time spike overview');
clf

tl = tiledlayout(5,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(tl);
plot(ax1,t_exec,T_exec,'LineWidth',1.2);
hold(ax1,'on');
yline(ax1,thr,'--k','Threshold');
plot(ax1,t_exec(spikeIdx),T_exec(spikeIdx),'or','LineWidth',1.5);
grid(ax1,'on');
ylabel(ax1,'Exec time [s]');
title(ax1,'Execution time spikes');

ax2 = nexttile(tl);
plot(ax2,t_states,X(:,2)*3.6,'LineWidth',1.2); hold(ax2,'on');
plot(ax2,t_states,X(:,5)*3.6,'LineWidth',1.2);
plot(ax2,t_states,X(:,8)*3.6,'LineWidth',1.2);
grid(ax2,'on');
ylabel(ax2,'v [km/h]');
legend(ax2,{'veh1','veh2','veh3'},'Location','best');
title(ax2,'Velocita veicoli');

ax3 = nexttile(tl);
plot(ax3,t_states,X(:,3),'LineWidth',1.2); hold(ax3,'on');
plot(ax3,t_states,X(:,6),'LineWidth',1.2);
plot(ax3,t_states,X(:,9),'LineWidth',1.2);
yline(ax3,cfg.Ax_max,'--k');
yline(ax3,cfg.Ax_min,'--k');
grid(ax3,'on');
ylabel(ax3,'a_x [m/s^2]');
title(ax3,'Accelerazione longitudinale');

ax4 = nexttile(tl);
gap12 = X(:,1) - X(:,4) - cfg.L_platoon;
gap23 = X(:,4) - X(:,7) - cfg.L_platoon;
plot(ax4,t_states,gap12,'LineWidth',1.2); hold(ax4,'on');
plot(ax4,t_states,gap23,'LineWidth',1.2);
yline(ax4,cfg.gap_min,'--k');
grid(ax4,'on');
ylabel(ax4,'gap [m]');
legend(ax4,{'gap12','gap23'},'Location','best');
title(ax4,'Gap netti');

ax5 = nexttile(tl);
yyaxis(ax5,'left');
stairs(ax5,t_status,status,'LineWidth',1.2);
ylabel(ax5,'status');
yyaxis(ax5,'right');
plot(ax5,t_J,J,'LineWidth',1.2);
hold(ax5,'on');
plot(ax5,t_iter,nIter,'LineWidth',1.2);
ylabel(ax5,'J / iter');
grid(ax5,'on');
xlabel(ax5,'Time [s]');
title(ax5,'Solver status, objective, iterations');

linkaxes([ax1 ax2 ax3 ax4 ax5],'x');

end

function local_plot_spike_window(localID,idxE, ...
    t_exec,T_exec,t_states,X,t_u,U,t_status,status,t_J,J,t_iter,nIter,cfg)

t0 = t_exec(idxE);
tw = cfg.window;
idxWinExec = t_exec >= t0-tw & t_exec <= t0+tw;
idxWinState = t_states >= t0-tw & t_states <= t0+tw;
idxWinU = t_u >= t0-tw & t_u <= t0+tw;
idxWinStatus = t_status >= t0-tw & t_status <= t0+tw;
idxWinJ = t_J >= t0-tw & t_J <= t0+tw;
idxWinIter = t_iter >= t0-tw & t_iter <= t0+tw;

figure('Name',sprintf('Spike %d - local diagnostics',localID));
clf

tl = tiledlayout(6,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(tl);
plot(ax1,t_exec(idxWinExec),T_exec(idxWinExec),'LineWidth',1.2);
hold(ax1,'on');
xline(ax1,t0,'--r');
grid(ax1,'on');
ylabel(ax1,'Exec [s]');
title(ax1,sprintf('Spike %d at t = %.3f s',localID,t0));

ax2 = nexttile(tl);
plot(ax2,t_states(idxWinState),X(idxWinState,1),'LineWidth',1.2); hold(ax2,'on');
plot(ax2,t_states(idxWinState),X(idxWinState,4),'LineWidth',1.2);
plot(ax2,t_states(idxWinState),X(idxWinState,7),'LineWidth',1.2);
xline(ax2,t0,'--r');
grid(ax2,'on');
ylabel(ax2,'s [m]');
legend(ax2,{'s1','s2','s3'},'Location','best');
title(ax2,'Posizione');

ax3 = nexttile(tl);
plot(ax3,t_states(idxWinState),X(idxWinState,2)*3.6,'LineWidth',1.2); hold(ax3,'on');
plot(ax3,t_states(idxWinState),X(idxWinState,5)*3.6,'LineWidth',1.2);
plot(ax3,t_states(idxWinState),X(idxWinState,8)*3.6,'LineWidth',1.2);
xline(ax3,t0,'--r');
grid(ax3,'on');
ylabel(ax3,'v [km/h]');
title(ax3,'Velocita');

ax4 = nexttile(tl);
plot(ax4,t_states(idxWinState),X(idxWinState,3),'LineWidth',1.2); hold(ax4,'on');
plot(ax4,t_states(idxWinState),X(idxWinState,6),'LineWidth',1.2);
plot(ax4,t_states(idxWinState),X(idxWinState,9),'LineWidth',1.2);
yline(ax4,cfg.Ax_max,'--k');
yline(ax4,cfg.Ax_min,'--k');
xline(ax4,t0,'--r');
grid(ax4,'on');
ylabel(ax4,'a_x [m/s^2]');
title(ax4,'Accelerazione');

ax5 = nexttile(tl);
plot(ax5,t_u(idxWinU),U(idxWinU,1),'LineWidth',1.2); hold(ax5,'on');
plot(ax5,t_u(idxWinU),U(idxWinU,2),'LineWidth',1.2);
plot(ax5,t_u(idxWinU),U(idxWinU,3),'LineWidth',1.2);
yline(ax5,cfg.jerk_max,'--k');
yline(ax5,cfg.jerk_min,'--k');
xline(ax5,t0,'--r');
grid(ax5,'on');
ylabel(ax5,'u [m/s^3]');
title(ax5,'Jerk / input NMPC');

ax6 = nexttile(tl);
gap12 = X(:,1) - X(:,4) - cfg.L_platoon;
gap23 = X(:,4) - X(:,7) - cfg.L_platoon;
plot(ax6,t_states(idxWinState),gap12(idxWinState),'LineWidth',1.2); hold(ax6,'on');
plot(ax6,t_states(idxWinState),gap23(idxWinState),'LineWidth',1.2);
yline(ax6,cfg.gap_min,'--k');
xline(ax6,t0,'--r');
grid(ax6,'on');
ylabel(ax6,'gap [m]');
xlabel(ax6,'Time [s]');
title(ax6,'Gap');

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x');

end

%% =========================================================================
%  ESTRAZIONE SEGNALI
% =========================================================================

function [t,val] = local_get_signal(out,names)

[t,val,ok] = local_try_get_signal(out,names);

if ~ok
    error('Segnale non trovato: %s',strjoin(names,', '));
end

end

function [t,val,ok] = local_try_get_signal(out,names)

ok = false;
t = [];
val = [];

for ii = 1:numel(names)

    sig = local_get_field_safe(out,names{ii});

    if isempty(sig)
        continue
    end

    [t,val,ok] = local_parse_signal(sig);

    if ok
        return
    end
end

end

function sig = local_get_field_safe(out,name)

sig = [];

try
    if isstruct(out) && isfield(out,name)
        sig = out.(name);
        return
    end
catch
end

try
    sig = out.(name);
    return
catch
end

try
    sig = out.get(name);
    return
catch
end

end

function [t,val,ok] = local_parse_signal(sig)

ok = false;
t = [];
val = [];

try
    if isstruct(sig) && isfield(sig,'time') && isfield(sig,'signals')
        t = sig.time(:);
        val = sig.signals.values;
        ok = true;
        return
    end
catch
end

try
    if isa(sig,'timeseries')
        t = sig.Time(:);
        val = sig.Data;
        ok = true;
        return
    end
catch
end

try
    if isstruct(sig) && isfield(sig,'Time') && isfield(sig,'Data')
        t = sig.Time(:);
        val = sig.Data;
        ok = true;
        return
    end
catch
end

end

function M = local_signal_to_matrix(values,n_time)

values = squeeze(values);

if isempty(values)
    M = nan(n_time,1);
    return
end

if isvector(values)
    M = values(:);
    return
end

if size(values,1) == n_time
    M = values;
elseif size(values,2) == n_time
    M = values.';
else
    M = reshape(values,[],n_time).';
end

end

function x = local_to_col(values)
x = squeeze(values);
x = x(:);
end

function row = local_sample_signal(t,M,t0,mode)

if isempty(t) || isempty(M)
    row = nan(1,1);
    return
end

M = squeeze(M);

if isvector(M)
    M = M(:);
end

if size(M,1) ~= numel(t)
    M = local_signal_to_matrix(M,numel(t));
end

switch lower(mode)
    case 'linear'
        if numel(t) >= 2
            row = interp1(t,M,t0,'linear','extrap');
        else
            row = M(1,:);
        end
    otherwise
        [~,idx] = min(abs(t-t0));
        row = M(idx,:);
end

end

%% =========================================================================
%  ESTRAZIONE HORIZON: state_pred, ctrl_pred, online_data
% =========================================================================

function [H,ok] = local_get_horizon_from_out(out,names,t0,nCol)

H = [];
ok = false;

for ii = 1:numel(names)

    sig = local_get_field_safe(out,names{ii});

    if isempty(sig)
        continue
    end

    [t,val,okSig] = local_parse_signal(sig);

    if ~okSig
        continue
    end

    [~,idx] = min(abs(t-t0));

    Htmp = local_extract_horizon_at_index(val,idx,numel(t),nCol);

    if ~isempty(Htmp)
        H = Htmp;
        ok = true;
        return
    end
end

end

function H = local_extract_horizon_at_index(values,idx,nTime,nCol)

H = [];

V = squeeze(values);

if isempty(V)
    return
end

sz = size(V);
nd = ndims(V);

if nd >= 3

    timeDimCandidates = find(sz == nTime);
    colDimCandidates  = find(sz == nCol);

    if isempty(timeDimCandidates) || isempty(colDimCandidates)
        return
    end

    timeDim = timeDimCandidates(end);
    colDim  = colDimCandidates(end);

    idxCell = repmat({':'},1,nd);
    idxCell{timeDim} = idx;

    S = squeeze(V(idxCell{:}));

    if isempty(S)
        return
    end

    if isvector(S)
        if mod(numel(S),nCol) == 0
            H = reshape(S,nCol,[]).';
        end
        return
    end

    if size(S,2) == nCol
        H = S;
    elseif size(S,1) == nCol
        H = S.';
    else
        flat = S(:).';
        if mod(numel(flat),nCol) == 0
            H = reshape(flat,nCol,[]).';
        end
    end

elseif nd == 2

    if size(V,1) == nTime
        row = V(idx,:);
    elseif size(V,2) == nTime
        row = V(:,idx).';
    else
        if size(V,2) == nCol
            H = V;
            return
        elseif size(V,1) == nCol
            H = V.';
            return
        else
            return
        end
    end

    if mod(numel(row),nCol) == 0
        H = reshape(row,nCol,[]).';
    end

else

    flat = V(:).';
    if mod(numel(flat),nCol) == 0
        H = reshape(flat,nCol,[]).';
    end

end

end

%% =========================================================================
%  SPIKE DETECTION
% =========================================================================

function spikeIdx = local_detect_spikes(t,y,threshold,minPeakDistance)

idxCand = find(isfinite(y) & y >= threshold);

if isempty(idxCand)
    spikeIdx = [];
    return
end

isPeak = false(size(idxCand));

for ii = 1:numel(idxCand)
    k = idxCand(ii);

    if k == 1
        leftOK = true;
    else
        leftOK = y(k) >= y(k-1);
    end

    if k == numel(y)
        rightOK = true;
    else
        rightOK = y(k) >= y(k+1);
    end

    isPeak(ii) = leftOK && rightOK;
end

spikeIdx = idxCand(isPeak);

if isempty(spikeIdx)
    spikeIdx = idxCand;
end

[~,ord] = sort(y(spikeIdx),'descend');
spikeIdxSorted = spikeIdx(ord);

accepted = [];

for ii = 1:numel(spikeIdxSorted)
    k = spikeIdxSorted(ii);

    if isempty(accepted)
        accepted = k;
    else
        if all(abs(t(k) - t(accepted)) >= minPeakDistance)
            accepted(end+1) = k; %#ok<AGROW>
        end
    end
end

spikeIdx = sort(accepted);

end

%% =========================================================================
%  STATISTICHE ROBUSTE
% =========================================================================

function m = local_nanmedian(x)

x = x(isfinite(x));

if isempty(x)
    m = nan;
else
    m = median(x);
end

end

function p = local_nanpercentile(x,prc)

x = x(isfinite(x));
x = sort(x(:));

if isempty(x)
    p = nan;
    return
end

if numel(x) == 1
    p = x;
    return
end

q = prc/100;
pos = 1 + (numel(x)-1)*q;
lo = floor(pos);
hi = ceil(pos);

if lo == hi
    p = x(lo);
else
    p = x(lo) + (x(hi)-x(lo))*(pos-lo);
end

end
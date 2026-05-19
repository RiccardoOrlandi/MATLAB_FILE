function inspect_spike_debug(out, t_query, mode)

% ============================================================
% INSPECT SPIKE DEBUG
% ============================================================
%
% Uso:
%
%   inspect_spike_debug(out, [350 380], 'range')
%       Analizza tutti i campioni tra 350 s e 380 s.
%
%   inspect_spike_debug(out, 350:1:380, 'list')
%       Stampa i campioni più vicini agli istanti specificati.
%
%   inspect_spike_debug(out, [350 360 370 380])
%       Default: modalità 'list'.
%
% Compatible with:
%   out.debug_TL as timeseries
%   out.debug_TL as structure with signals.values
% ============================================================

if nargin < 3
    mode = 'list';
end

%dbg_obj = out.debug_TL;
dbg_obj = out.debug_TL_2;

if isa(dbg_obj,'timeseries')
    dbg = dbg_obj.Data;
    t   = dbg_obj.Time;
else
    dbg = dbg_obj.signals.values;
    t   = dbg_obj.time;
end

names = {'time','ii','s_TL','pos','vel','pos_tail', ...
         'reached_old','reached_new', ...
         'xTL_1','xTL_2','xTL_3', ...
         'head_crossed_now','tail_crossed_now','head_stopped_at_light', ...
         'mem_case','main_case', ...
         'current_green_end','i_head_cross','i_tail_cross','green_window_ok', ...
         'tail_at_green_end', ...
         'Online_head_k1','Online_tail_k1','Online_head_k2','Online_tail_k2', ...
         'dist_to_TL','s_TL_head_lim_hor(2)','s_TL_tail_lim_hor(2)','s_TL_head_lim_hor(i_head_cross)','s_TL_tail_lim_hor(i_tail_cross)'};

% ============================================================
% RANGE MODE: analisi continua su intervallo temporale
% ============================================================

if strcmpi(mode,'range')

    if numel(t_query) ~= 2
        error('In modalità range, t_query deve essere [t_min t_max].');
    end

    t_min = t_query(1);
    t_max = t_query(2);

    idx_time = find(t >= t_min & t <= t_max);

    if isempty(idx_time)
        error('Nessun campione debug_TL trovato tra %.3f s e %.3f s.', t_min, t_max);
    end

    fprintf('\n====================================================\n');
    fprintf('DEBUG RANGE: %.3f s -> %.3f s\n', t(idx_time(1)), t(idx_time(end)));
    fprintf('Number of samples: %d\n', numel(idx_time));
    fprintf('====================================================\n');

    ALL = [];

    for kk = idx_time(:)'

        D = local_get_debug_sample(dbg, kk, length(t));

        % D deve essere N_TL x 26
        if size(D,2) ~= 26 && size(D,1) == 26
            D = D';
        end

        if size(D,2) ~= 26
            error('Formato debug_TL non riconosciuto. Size sample = [%d %d].', size(D,1), size(D,2));
        end

        dist = D(:,26);

        % Semafori rilevanti: davanti entro 150 m oppure appena superati entro 20 m
        idx_tl = find(dist < 150 & dist > -20);

        if isempty(idx_tl)
            [~,idx0] = min(abs(dist));
            idx_tl = idx0;
        end

        ALL = [ALL; D(idx_tl,:)]; %#ok<AGROW>
    end

    T = array2table(ALL, 'VariableNames', names);

    % ========================================================
    % Tabella compatta: colonne realmente utili per diagnosticare spike
    % ========================================================

    T_compact = T(:, {'time','ii','s_TL','pos','vel','pos_tail', ...
                      'dist_to_TL', ...
                      'xTL_1','xTL_2','xTL_3', ...
                      'mem_case','main_case', ...
                      'current_green_end','i_head_cross','i_tail_cross', ...
                      'green_window_ok','tail_at_green_end', ...
                      'Online_head_k2','Online_tail_k2'});

    disp(T_compact);

    % ========================================================
    % Diagnosi automatica: switching dei main_case
    % ========================================================

    fprintf('\n--- MAIN CASE SWITCHES ---\n');

    unique_TL = unique(T.ii);

    for jj = 1:numel(unique_TL)

        tl_id = unique_TL(jj);
        idx = T.ii == tl_id;

        tt = T.time(idx);
        mc = T.main_case(idx);

        if numel(mc) < 2
            continue
        end

        dmc = [false; diff(mc) ~= 0];

        if any(dmc)
            fprintf('\nTL %d:\n', tl_id);

            idx_sw = find(dmc);

            for ss = idx_sw(:)'
                fprintf('  t = %.3f s: main_case %g -> %g\n', ...
                    tt(ss), mc(ss-1), mc(ss));
            end
        end
    end

    % ========================================================
    % Plot diagnostici
    % ========================================================

    figure
    hold on
    grid on
    for jj = 1:numel(unique_TL)
        tl_id = unique_TL(jj);
        idx = T.ii == tl_id;
        stairs(T.time(idx), T.main_case(idx), 'LineWidth', 1.2);
    end
    xlabel('time [s]')
    ylabel('main\_case')
    title(sprintf('main\\_case between %.1f s and %.1f s', t_min, t_max))
    legend(compose('TL %d', unique_TL), 'Location', 'best')
    ylim([0 10])

    figure
    hold on
    grid on
    for jj = 1:numel(unique_TL)
        tl_id = unique_TL(jj);
        idx = T.ii == tl_id;
        stairs(T.time(idx), T.Online_head_k2(idx), 'LineWidth', 1.2);
    end
    xlabel('time [s]')
    ylabel('OnlineData head kk=2')
    title(sprintf('Head TL constraint at kk=2 between %.1f s and %.1f s', t_min, t_max))
    legend(compose('TL %d', unique_TL), 'Location', 'best')
    ylim([-0.1 1.1])

    figure
    hold on
    grid on
    for jj = 1:numel(unique_TL)
        tl_id = unique_TL(jj);
        idx = T.ii == tl_id;
        stairs(T.time(idx), T.Online_tail_k2(idx), 'LineWidth', 1.2);
    end
    xlabel('time [s]')
    ylabel('OnlineData tail kk=2')
    title(sprintf('Tail TL constraint at kk=2 between %.1f s and %.1f s', t_min, t_max))
    legend(compose('TL %d', unique_TL), 'Location', 'best')
    ylim([-0.1 1.1])

    return
end

% ============================================================
% LIST MODE: comportamento originale
% ============================================================

for tt = t_query

    [~,k] = min(abs(t - tt));

    D = local_get_debug_sample(dbg,k,length(t));

    fprintf('\n====================================================\n');
    fprintf('DEBUG time = %.3f s, requested = %.3f s\n', t(k), tt);
    fprintf('====================================================\n');

    % D deve essere N_TL x 26
    if size(D,2) ~= 26 && size(D,1) == 26
        D = D';
    end

    if size(D,2) ~= 26
        error('Formato debug_TL non riconosciuto. Size del sample = [%d %d].', size(D,1), size(D,2));
    end

    dist = D(:,26);

    % Mostro semafori davanti entro 150 m oppure appena superati entro 20 m
    idx = find(dist < 150 & dist > -20);

    if isempty(idx)
        [~,idx0] = min(abs(dist));
        idx = idx0;
    end

    T = array2table(D(idx,:), 'VariableNames', names);
    disp(T);

end

end


function D = local_get_debug_sample(dbg,k,Nt)

sz = size(dbg);

if ndims(dbg) == 2

    % Caso tipico:
    %   [Nt x 26]
    % oppure:
    %   [26 x Nt]

    if sz(1) == Nt
        D = squeeze(dbg(k,:));

    elseif sz(2) == Nt
        D = squeeze(dbg(:,k))';

    else
        error('Formato 2D debug_TL non riconosciuto. Size = [%s].', num2str(sz));
    end

elseif ndims(dbg) == 3

    % Possibili formati:
    %   [Nt x N_TL x 26]
    %   [N_TL x 26 x Nt]
    %   [N_TL x Nt x 26]

    if sz(1) == Nt
        D = squeeze(dbg(k,:,:));

    elseif sz(3) == Nt
        D = squeeze(dbg(:,:,k));

    elseif sz(2) == Nt
        D = squeeze(dbg(:,k,:));

    else
        error('Formato 3D debug_TL non riconosciuto. Size = [%s].', num2str(sz));
    end

else
    error('debug_TL ha ndims = %d, non gestito.', ndims(dbg));
end

end
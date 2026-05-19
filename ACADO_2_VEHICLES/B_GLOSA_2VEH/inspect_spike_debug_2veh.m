function inspect_spike_debug_2veh(out, t_query, mode, veh_id)

if nargin < 3
    mode = 'list';
end

if nargin < 4
    veh_id = 1;
end

if veh_id == 1
    dbg_obj = out.debug_TL;
elseif veh_id == 2
    dbg_obj = out.debug_TL_2;
else
    error('veh_id deve essere 1 oppure 2.');
end

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
         'dist_to_TL'};

if strcmpi(mode,'range')

    if numel(t_query) ~= 2
        error('In modalità range, t_query deve essere [t_min t_max].');
    end

    t_min = t_query(1);
    t_max = t_query(2);

    idx_time = find(t >= t_min & t <= t_max);

    if isempty(idx_time)
        error('Nessun campione debug trovato tra %.3f s e %.3f s.', t_min, t_max);
    end

    fprintf('\n====================================================\n');
    fprintf('DEBUG VEHICLE %d RANGE: %.3f s -> %.3f s\n', ...
        veh_id, t(idx_time(1)), t(idx_time(end)));
    fprintf('Number of samples: %d\n', numel(idx_time));
    fprintf('====================================================\n');

    ALL = [];

    for kk = idx_time(:)'

        D = local_get_debug_sample(dbg, kk, length(t));

        if size(D,2) ~= 26 && size(D,1) == 26
            D = D';
        end

        if size(D,2) ~= 26
            error('Formato debug_TL non riconosciuto. Size sample = [%d %d].', ...
                size(D,1), size(D,2));
        end

        dist = D(:,26);

        idx_tl = find(dist < 150 & dist > -20);

        if isempty(idx_tl)
            [~,idx0] = min(abs(dist));
            idx_tl = idx0;
        end

        ALL = [ALL; D(idx_tl,:)]; %#ok<AGROW>

    end

    T = array2table(ALL, 'VariableNames', names);

    T_compact = T(:, {'time','ii','s_TL','pos','vel','pos_tail', ...
                      'dist_to_TL', ...
                      'xTL_1','xTL_2','xTL_3', ...
                      'mem_case','main_case', ...
                      'current_green_end','i_head_cross','i_tail_cross', ...
                      'green_window_ok','tail_at_green_end', ...
                      'Online_head_k2','Online_tail_k2'});

    disp(T_compact);

    fprintf('\n--- MAIN CASE SWITCHES VEHICLE %d ---\n', veh_id);

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
    title(sprintf('Vehicle %d: main\\_case between %.1f s and %.1f s', ...
        veh_id, t_min, t_max))
    legend(compose('TL %d', unique_TL), 'Location', 'best')
    ylim([0 11])

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
    title(sprintf('Vehicle %d: Head TL constraint at kk=2', veh_id))
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
    title(sprintf('Vehicle %d: Tail TL constraint at kk=2', veh_id))
    legend(compose('TL %d', unique_TL), 'Location', 'best')
    ylim([-0.1 1.1])

    return

end

for tt = t_query

    [~,k] = min(abs(t - tt));

    D = local_get_debug_sample(dbg,k,length(t));

    fprintf('\n====================================================\n');
    fprintf('DEBUG VEHICLE %d time = %.3f s, requested = %.3f s\n', ...
        veh_id, t(k), tt);
    fprintf('====================================================\n');

    if size(D,2) ~= 26 && size(D,1) == 26
        D = D';
    end

    if size(D,2) ~= 26
        error('Formato debug_TL non riconosciuto. Size sample = [%d %d].', ...
            size(D,1), size(D,2));
    end

    dist = D(:,26);

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

    if sz(1) == Nt
        D = squeeze(dbg(k,:));

    elseif sz(2) == Nt
        D = squeeze(dbg(:,k))';

    else
        error('Formato 2D debug_TL non riconosciuto. Size = [%s].', num2str(sz));
    end

elseif ndims(dbg) == 3

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
%==========================================================================
% GENERAZIONE PIANO SEMAFORICO ESTESO - 30 TL - 2000 s
%==========================================================================
% Estende deterministicamente lo SPAT misurato del corridoio Piola-Lugano
% oltre la fine della registrazione.
%
% I primi 1600 s restano IDENTICI al dato misurato, campione per campione.
% Oltre, per ciascun semaforo la sequenza prosegue con il ciclo medio
% misurato su quel semaforo: si completa la fase in corso e poi si alterna
% verde/rosso con le durate medie del semaforo stesso.
%
% Questo conserva periodo e split di ogni incrocio e gli offset reciproci,
% che sono l'informazione sfruttata dal GLOSA.
%
% ATTENZIONE: la coda oltre 1600 s e' SINTETICA, non misurata.
%==========================================================================

clear; clc;

FILE_IN  = 'TL_data_piola-lugano_lookupTable_1600s_30TL.mat';
FILE_OUT = 'TL_data_piola-lugano_lookupTable_2000s_30TL_ESTESO.mat';
T_END    = 2000;   % [s] durata finale della tabella

D = load(FILE_IN);
time_bag_in = D.time_bag(:);
tl_col_in   = D.tl_col;
tl_tim_in   = D.tl_tim;

Ts       = median(diff(time_bag_in));
n_in     = size(tl_col_in,1);
n_tl     = size(tl_col_in,2);
t_end_in = time_bag_in(end);

n_out = round(T_END/Ts);
n_add = n_out - n_in;
if n_add <= 0
    error('La tabella copre gia %.1f s.', t_end_in);
end

% margine interno: genero oltre la fine per poter calcolare il countdown
n_margin = round(600/Ts);

fprintf('SPAT misurato : %.1f s, %d campioni, %d semafori\n', t_end_in, n_in, n_tl);
fprintf('SPAT esteso   : %.1f s, %d campioni (%d aggiunti)\n\n', T_END, n_out, n_add);

tl_col = zeros(n_out, n_tl);
tl_tim = zeros(n_out, n_tl);
tl_col(1:n_in,:) = tl_col_in;
tl_tim(1:n_in,:) = tl_tim_in;

fprintf('%3s %9s %9s %9s %6s %14s\n', 'TL', 'ciclo', 'verde', 'rosso', 'fase', 'residuo [s]');

for j = 1:n_tl
    x = tl_col_in(:,j);
    d = find(diff(x) ~= 0);

    if isempty(d)
        ext = repmat(x(end), n_add + n_margin, 1);
        fprintf('%3d %9s %9s %9s %6d %14s\n', j, '-', '-', '-', x(end), 'costante');
    else
        bounds = [0; d; n_in];
        segLen = diff(bounds);
        segVal = x(bounds(1:end-1)+1);
        % primo e ultimo segmento sono troncati dalla registrazione: esclusi
        inner_len = segLen(2:end-1);
        inner_val = segVal(2:end-1);
        n_green = round(mean(inner_len(inner_val == 1)));
        n_red   = round(mean(inner_len(inner_val == 0)));
        if ~isfinite(n_green), n_green = round(mean(inner_len)); end
        if ~isfinite(n_red),   n_red   = round(mean(inner_len)); end
        n_green = max(1, n_green);
        n_red   = max(1, n_red);

        s_now   = x(end);
        elapsed = n_in - d(end);
        if s_now == 1, n_cur = n_green; else, n_cur = n_red; end
        n_res = max(1, n_cur - elapsed);

        ext = zeros(n_add + n_margin, 1);
        p = 0;
        k = min(n_res, numel(ext));
        ext(p+1:p+k) = s_now; p = p + k;
        s = s_now;
        while p < numel(ext)
            s = 1 - s;
            if s == 1, k = n_green; else, k = n_red; end
            k = min(k, numel(ext) - p);
            ext(p+1:p+k) = s; p = p + k;
        end
        fprintf('%3d %9.1f %9.1f %9.1f %6d %14.1f\n', j, (n_green+n_red)*Ts, n_green*Ts, n_red*Ts, s_now, n_res*Ts);
    end

    full = [x; ext];
    tl_col(n_in+1:n_out, j) = full(n_in+1:n_out);

    % countdown al prossimo cambio, stessa convenzione di tl_tim
    if all(tl_tim_in(:,j) == -1)
        tl_tim(n_in+1:n_out, j) = -1;
    else
        sw = find(diff(full) ~= 0);
        for i = n_in+1:n_out
            nx = sw(find(sw >= i, 1));
            if isempty(nx)
                tl_tim(i,j) = -1;
            else
                tl_tim(i,j) = (nx - i + 1)*Ts;
            end
        end
    end
end

% costruito per indice: (Ts:Ts:T_END) accumula errore in virgola mobile
time_bag = [time_bag_in; ((n_in+1:n_out)'*Ts)];

%--------------------------------------------------------------------------
% CONTROLLI
%--------------------------------------------------------------------------
assert(numel(time_bag) == n_out, 'time_bag incoerente');
assert(isequal(tl_col(1:n_in,:), tl_col_in), 'la parte misurata e stata alterata');
assert(isequal(tl_tim(1:n_in,:), tl_tim_in), 'tl_tim misurato alterato');
assert(all(ismember(tl_col(:), [0 1])), 'valori non binari');

n_seam = sum(tl_col(n_in,:) ~= tl_col(n_in+1,:));
fprintf('\ncommutazioni spurie alla giunzione (t = %.1f s): %d su %d\n', t_end_in, n_seam, n_tl);

save(FILE_OUT, 'time_bag', 'tl_col', 'tl_tim');
fprintf('salvato: %s\n', FILE_OUT);
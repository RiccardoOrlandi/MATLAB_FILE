function KPI = compute_comparison_KPI(out,config)
%==========================================================================
% COMPUTE_COMPARISON_KPI
%==========================================================================
% Calcola i KPI per l'analisi comparativa 1/2/3 veicoli.
%
% MODELLO ENERGETICO
%   La massa totale di ogni veicolo e' separata in:
%     mass_empty  : tara del veicolo [kg]
%     mass_pax    : massa passeggeri = nPax * pax_mass_kg [kg]
%     mass_total  : mass_empty + mass_pax  (usata nel calcolo P = m*a*v)
%
%   P_in  = mass_total * a * v
%   E     = integral(max(P_in, 0) dt)   [J] -> [kWh]
%
%   Questo consente di rieseguire i KPI con diversi load factor
%   (fascia critica / media / notturna) senza rifare la simulazione,
%   variando solo config.nPax.
%
% CONVENZIONE STATI
%   1 veh : [s1 v1 a1]
%   2 veh : [s1 v1 a1 s2 v2 a2]
%   3 veh : [s1 v1 a1 s2 v2 a2 s3 v3 a3]
%
% CAMPI OBBLIGATORI config
%   config.nVeh                numero veicoli
%   config.mass_empty          [1 x nVeh] tara per veicolo [kg]
%   config.nPax                [1 x nVeh] n. passeggeri per veicolo [-]
%   config.v_stop              soglia velocita' fermo [m/s]
%   config.t_min_stop          durata minima stop conteggiato [s]
%
% CAMPI OPZIONALI config
%   config.pax_mass_kg         massa per passeggero [kg]  (default 75)
%   config.s_stop              coordinate fermate obbligatorie [m]
%   config.stop_exclusion_radius  raggio esclusione fermata [m] (default 3)
%   config.route_target_s      coordinata arrivo per tempo viaggio [m]
%   config.arrival_tol_s       tolleranza target arrivo [m] (default 1)
%   config.L_vehicle           lunghezza veicolo per gap netto [m] (default 18)
%   config.fascia              stringa descrittiva fascia oraria
%   config.t_trim_start        secondi da escludere all'inizio [s] (default 0)
%   config.t_trim_end          secondi da escludere alla fine   [s] (default 0)
%==========================================================================

%% ========================================================================
% 0. Validazione e lettura configurazione
% ========================================================================

required_fields = {'nVeh','mass_empty','nPax','v_stop','t_min_stop'};
for k = 1:numel(required_fields)
    if ~isfield(config,required_fields{k})
        error('Campo config.%s mancante.',required_fields{k});
    end
end

nVeh = config.nVeh;

mass_empty = local_row_vector(config.mass_empty);
nPax       = local_row_vector(config.nPax);

if numel(mass_empty) == 1 && nVeh > 1
    mass_empty = repmat(mass_empty,1,nVeh);
end
if numel(nPax) == 1 && nVeh > 1
    nPax = repmat(nPax,1,nVeh);
end

if numel(mass_empty) ~= nVeh
    error('config.mass_empty deve avere 1 oppure nVeh elementi.');
end
if numel(nPax) ~= nVeh
    error('config.nPax deve avere 1 oppure nVeh elementi.');
end

% Massa per passeggero
if isfield(config,'pax_mass_kg')
    pax_mass_kg = config.pax_mass_kg;
else
    pax_mass_kg = 75;   % [kg/pax] valore di default
end

% Massa passeggeri e massa totale per veicolo
mass_pax   = nPax .* pax_mass_kg;          % [kg] contributo passeggeri
mass_total = mass_empty + mass_pax;         % [kg] massa dinamica effettiva

% Parametri stop
if isfield(config,'s_stop')
    s_stop = local_row_vector(config.s_stop);
else
    s_stop = [];
end

if isfield(config,'stop_exclusion_radius')
    stop_exclusion_radius = config.stop_exclusion_radius;
else
    stop_exclusion_radius = 3.0;
end

if isfield(config,'arrival_tol_s')
    arrival_tol_s = config.arrival_tol_s;
else
    arrival_tol_s = 1.0;
end

if isfield(config,'L_vehicle')
    L_vehicle = config.L_vehicle;
else
    L_vehicle = 18.0;
end

if isfield(config,'t_trim_start')
    t_trim_start = max(config.t_trim_start, 0);
else
    t_trim_start = 0;
end

if isfield(config,'t_trim_end')
    t_trim_end = max(config.t_trim_end, 0);
else
    t_trim_end = 0;
end

%% ========================================================================
% 1. Estrazione stati
% ========================================================================

if ~local_has_field(out,'states')
    error('La struttura out non contiene out.states.');
end

t = out.states.time(:);
X = out.states.signals.values;
X = squeeze(X);

if size(X,1) ~= numel(t) && size(X,2) == numel(t)
    X = X.';
end

if size(X,2) < 3*nVeh
    error('out.states deve contenere almeno %d colonne per nVeh=%d.',3*nVeh,nVeh);
end

s = zeros(numel(t),nVeh);
v = zeros(numel(t),nVeh);
a = zeros(numel(t),nVeh);

for i = 1:nVeh
    idx = 3*(i-1);
    s(:,i) = X(:,idx+1);
    v(:,i) = X(:,idx+2);
    a(:,i) = X(:,idx+3);
end

%% ========================================================================
% 2. Estrazione jerk di controllo
% ========================================================================

if local_has_field(out,'u')
    t_u  = out.u.time(:);
    Uraw = squeeze(out.u.signals.values);
    if isvector(Uraw)
        U = Uraw(:);
    elseif size(Uraw,1) == numel(t_u)
        U = Uraw;
    elseif size(Uraw,2) == numel(t_u)
        U = Uraw.';
    else
        U = reshape(Uraw,[],numel(t_u)).';
    end
    if size(U,2) < nVeh
        U(:,end+1:nVeh) = NaN;
    end
else
    t_u = t;
    U   = NaN(numel(t),nVeh);
end

%% ========================================================================
% 1.5 Trimming temporale
% ========================================================================
% Rimuove i primi t_trim_start secondi e gli ultimi t_trim_end secondi
% da TUTTI i segnali prima del calcolo dei KPI.
% Il tempo di simulazione effettivo T_sim riflette la finestra trimmed.
% ========================================================================

if t_trim_start > 0 || t_trim_end > 0
    t0_trim = t(1)   + t_trim_start;
    t1_trim = t(end) - t_trim_end;

    if t1_trim <= t0_trim
        error('t_trim_start+t_trim_end (%.1fs) >= durata sim (%.1fs).', t_trim_start+t_trim_end, t(end)-t(1));
    end

    % Maschera stati
    mask_t = t >= t0_trim & t <= t1_trim;
    t = t(mask_t);
    s = s(mask_t,:);
    v = v(mask_t,:);
    a = a(mask_t,:);

    % Maschera jerk (asse tempi diverso)
    mask_u = t_u >= t0_trim & t_u <= t1_trim;
    t_u = t_u(mask_u);
    U   = U(mask_u,:);

    % Log
    fprintf('[KPI trim] finestra: %.1f s -> %.1f s (trim start=%.1f s, end=%.1f s)\n', t(1), t(end), t_trim_start, t_trim_end);
end

KPI.meta.t_trim_start = t_trim_start;
KPI.meta.t_trim_end   = t_trim_end;
KPI.meta.t_analysis_start = t(1);
KPI.meta.t_analysis_end   = t(end);

%% ========================================================================
% 3. KPI energetici
% ========================================================================
%
% L'energia viene calcolata con la massa totale effettiva (tara + passeggeri).
% I tre valori vengono salvati separatamente per permettere confronti:
%   E_vehicle_kWh       : energia con massa totale (tara + pax)   [kWh]
%   E_empty_vehicle_kWh : energia con sola tara                   [kWh]
%   E_pax_vehicle_kWh   : contributo energetico dei passeggeri    [kWh]
% ========================================================================

E_vehicle_kWh       = zeros(1,nVeh);
E_empty_vehicle_kWh = zeros(1,nVeh);

for i = 1:nVeh
    % Energia con massa totale (tara + passeggeri)
    P_total  = mass_total(i) .* a(:,i) .* v(:,i);
    E_vehicle_kWh(i) = trapz(t, max(P_total,0)) / 3.6e6;

    % Energia con sola tara (per separare il contributo pax)
    P_empty  = mass_empty(i) .* a(:,i) .* v(:,i);
    E_empty_vehicle_kWh(i) = trapz(t, max(P_empty,0)) / 3.6e6;
end

E_pax_vehicle_kWh = E_vehicle_kWh - E_empty_vehicle_kWh;   % contributo pax

E_total_kWh       = sum(E_vehicle_kWh);
E_empty_total_kWh = sum(E_empty_vehicle_kWh);
E_pax_total_kWh   = sum(E_pax_vehicle_kWh);
N_pax_total       = sum(nPax);

% Meta-dati
KPI.meta.nVeh           = nVeh;
KPI.meta.mass_empty_kg  = mass_empty;
KPI.meta.mass_pax_kg    = mass_pax;
KPI.meta.mass_total_kg  = mass_total;
KPI.meta.pax_mass_kg    = pax_mass_kg;
KPI.meta.nPax_vehicle   = nPax;
KPI.meta.nPax_total     = N_pax_total;
KPI.meta.load_factor    = nPax ./ (nPax + mass_empty./pax_mass_kg);  % [-] approx
if isfield(config,'fascia')
    KPI.meta.fascia = config.fascia;
else
    KPI.meta.fascia = 'N/D';
end

% KPI energetici
KPI.energy.E_vehicle_kWh         = E_vehicle_kWh;
KPI.energy.E_empty_vehicle_kWh   = E_empty_vehicle_kWh;
KPI.energy.E_pax_vehicle_kWh     = E_pax_vehicle_kWh;
KPI.energy.E_total_kWh           = E_total_kWh;
KPI.energy.E_empty_total_kWh     = E_empty_total_kWh;
KPI.energy.E_pax_total_kWh       = E_pax_total_kWh;
KPI.energy.E_pax_kWh_per_pax     = E_total_kWh / max(N_pax_total,1);

%% ========================================================================
% 4. KPI tempo: tempo di viaggio e arrivo
% ========================================================================

T_sim     = t(end) - t(1);
T_vehicle = zeros(1,nVeh);

if isfield(config,'route_target_s') && ~isempty(config.route_target_s)
    s_target = config.route_target_s;
    for i = 1:nVeh
        idx_arr = find(s(:,i) >= s_target - arrival_tol_s, 1, 'first');
        if isempty(idx_arr)
            T_vehicle(i) = T_sim;
        else
            T_vehicle(i) = t(idx_arr) - t(1);
        end
    end
else
    T_vehicle(:) = T_sim;
end

KPI.time.T_vehicle_s      = T_vehicle;
KPI.time.T_travel_mean_s  = mean(T_vehicle);
KPI.time.T_arrival_max_s  = max(T_vehicle);

%% ========================================================================
% 5. KPI comfort: RMS e picco di accelerazione e jerk
% ========================================================================

Ax_RMS_vehicle  = zeros(1,nVeh);
Ax_peak_vehicle = zeros(1,nVeh);
u_RMS_vehicle   = NaN(1,nVeh);
u_peak_vehicle  = NaN(1,nVeh);

for i = 1:nVeh
    Ax_RMS_vehicle(i)  = sqrt(trapz(t, a(:,i).^2) / max(T_sim,eps));
    Ax_peak_vehicle(i) = max(abs(a(:,i)));

    if i <= size(U,2) && any(isfinite(U(:,i)))
        u_i       = U(:,i);
        idx_valid = isfinite(u_i) & isfinite(t_u);
        if nnz(idx_valid) >= 2
            T_u = t_u(find(idx_valid,1,'last')) - t_u(find(idx_valid,1,'first'));
            u_RMS_vehicle(i)  = sqrt(trapz(t_u(idx_valid), u_i(idx_valid).^2) / max(T_u,eps));
            u_peak_vehicle(i) = max(abs(u_i(idx_valid)));
        end
    end
end

KPI.comfort.Ax_RMS_vehicle   = Ax_RMS_vehicle;
KPI.comfort.Ax_peak_vehicle  = Ax_peak_vehicle;
KPI.comfort.Ax_RMS_aggregate = sqrt(sum(trapz(t,a.^2,1)) / (nVeh*max(T_sim,eps)));
KPI.comfort.Ax_peak_max      = max(Ax_peak_vehicle);

KPI.comfort.u_RMS_vehicle    = u_RMS_vehicle;
KPI.comfort.u_peak_vehicle   = u_peak_vehicle;
KPI.comfort.u_RMS_aggregate  = local_rms_aggregate(t_u, U(:,1:nVeh));
KPI.comfort.u_peak_max       = local_nanmax(u_peak_vehicle);

%% ========================================================================
% 6. Gap inter-veicolo minimo (solo nVeh >= 2)
% ========================================================================

if nVeh >= 2
    gap_min_v  = NaN(1,nVeh-1);
    gap_mean_v = NaN(1,nVeh-1);
    for i = 1:nVeh-1
        g = s(:,i) - s(:,i+1) - L_vehicle;
        gap_min_v(i)  = min(g);
        gap_mean_v(i) = mean(g);
    end
    KPI.gap.gap_min_m      = gap_min_v;
    KPI.gap.gap_mean_m     = gap_mean_v;
    KPI.gap.gap_min_global = min(gap_min_v);
else
    KPI.gap.gap_min_m      = NaN;
    KPI.gap.gap_mean_m     = NaN;
    KPI.gap.gap_min_global = NaN;
end

%% ========================================================================
% 7. Stop KPI al netto delle fermate obbligatorie
% ========================================================================

v_stop     = config.v_stop;
t_min_stop = config.t_min_stop;

N_stop_nonmandatory = zeros(1,nVeh);
T_stop_nonmandatory = zeros(1,nVeh);

for i = 1:nVeh
    is_stopped = v(:,i) < v_stop;
    is_mandatory = false(size(is_stopped));
    for j = 1:numel(s_stop)
        is_mandatory = is_mandatory | (abs(s(:,i) - s_stop(j)) <= stop_exclusion_radius);
    end
    is_free_stop = is_stopped & ~is_mandatory;
    T_stop_nonmandatory(i) = trapz(t, double(is_free_stop));
    N_stop_nonmandatory(i) = local_count_stop_events(t, is_free_stop, t_min_stop);
end

KPI.stop.N_stop_vehicle   = N_stop_nonmandatory;
KPI.stop.T_stop_vehicle_s = T_stop_nonmandatory;

%% ========================================================================
% 8. KPI computazionali solver
% ========================================================================

if local_has_field(out,'executionTime')
    texec = squeeze(out.executionTime.signals.values);
    texec = texec(isfinite(texec(:)));
    KPI.solver.execution_mean_s   = local_nanmean(texec);
    KPI.solver.execution_median_s = local_nanmedian(texec);
    KPI.solver.execution_p95_s    = local_percentile(texec,95);
else
    KPI.solver.execution_mean_s   = NaN;
    KPI.solver.execution_median_s = NaN;
    KPI.solver.execution_p95_s    = NaN;
end

end

%% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function tf = local_has_field(s, fname)
% Compatibile con struct e Simulink.SimulationOutput
    if isstruct(s)
        tf = isfield(s, fname);
    else
        try
            tf = ~isempty(s.(fname));
        catch
            tf = false;
        end
    end
end

function x = local_row_vector(x)
    x = x(:).';
end

function y = local_nanmean(x)
    x = x(isfinite(x));
    if isempty(x); y = NaN; else; y = mean(x); end
end

function y = local_nanmedian(x)
    x = x(isfinite(x));
    if isempty(x); y = NaN; else; y = median(x); end
end

function y = local_nanmax(x)
    x = x(isfinite(x));
    if isempty(x); y = NaN; else; y = max(x); end
end

function y = local_percentile(x,p)
    x = sort(x(isfinite(x)));
    if isempty(x); y = NaN; return; end
    if numel(x) == 1; y = x; return; end
    q = 1 + (numel(x)-1)*p/100;
    ql = floor(q); qh = ceil(q);
    if ql == qh; y = x(ql);
    else; y = (1-(q-ql))*x(ql) + (q-ql)*x(qh); end
end

function y = local_rms_aggregate(t,U)
    if isempty(U); y = NaN; return; end
    total = 0; n = 0;
    for i = 1:size(U,2)
        u_i = U(:,i);
        idx = isfinite(u_i) & isfinite(t);
        if nnz(idx) >= 2
            total = total + trapz(t(idx), u_i(idx).^2);
            n = n + 1;
        end
    end
    if n == 0; y = NaN; return; end
    T = t(end) - t(1);
    y = sqrt(total / (n * max(T,eps)));
end

function N = local_count_stop_events(t,isStopped,t_min_stop)
    isStopped = isStopped(:); t = t(:);
    d = diff([false; isStopped; false]);
    idx_s = find(d == 1);
    idx_e = find(d == -1) - 1;
    N = 0;
    for k = 1:numel(idx_s)
        if t(idx_e(k)) - t(idx_s(k)) >= t_min_stop
            N = N + 1;
        end
    end
end

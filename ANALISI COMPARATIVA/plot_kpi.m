%==========================================================================
% PLOT_KPI
%==========================================================================
% Script principale per l'analisi comparativa 1/2/3 veicoli.
%
% WORKFLOW:
%   STEP 1: salvare i risultati Simulink:
%             out_bus  = out;   (simulazione 1 veicolo)
%             out_2veh = out;   (simulazione 2 veicoli)
%             out_3veh = out;   (simulazione 3 veicoli)
%   STEP 2: eseguire questo script.
%
% Lo script calcola i KPI per 3 fasce orarie (critica / media / notturna)
% variando solo il numero di passeggeri per veicolo.
% La simulazione e' unica: si riusa out_* per tutte e tre le fasce.
%
% OUTPUT workspace:
%   KPI_bus_critica,   KPI_bus_media,   KPI_bus_notturna
%   KPI_2veh_critica,  KPI_2veh_media,  KPI_2veh_notturna
%   KPI_3veh_critica,  KPI_3veh_media,  KPI_3veh_notturna
%   Comparison   (tabella MATLAB)
%==========================================================================

clearvars KPI_bus_critica KPI_bus_media KPI_bus_notturna ...
          KPI_2veh_critica KPI_2veh_media KPI_2veh_notturna ...
          KPI_3veh_critica KPI_3veh_media KPI_3veh_notturna Comparison

%% ========================================================================
% 1. Parametri comuni (indipendenti dalla fascia)
% ========================================================================

v_stop               = 0.2;    % [m/s]  soglia veicolo fermo
t_min_stop           = 1.0;    % [s]    durata minima stop conteggiato
stop_exclusion_radius = 3;     % [m]    esclusione fermate obbligatorie
pax_mass_kg          = 70;     % [kg]   massa per passeggero
route_target_s       = [];     % lascia [] per usare durata simulata

% Trimming temporale: secondi da escludere all'inizio e alla fine
% della simulazione prima del calcolo dei KPI.
% Utile per eliminare il transitorio iniziale del solver e la
% decelerazione finale a fine percorso.
% Imposta 0 per non escludere nulla.
t_trim_start = 3;   % [s] escludi i primi 30 s (transitorio NMPC)
t_trim_end   = 3;   % [s] escludi gli ultimi 30 s (decelerazione finale)

% Fermate obbligatorie dal workspace (se disponibili) o default da plot_data
if exist('s_stop','var')
    mandatory_stops = s_stop(:).';
else
    mandatory_stops = [80, 447, 756, 1084, 1304, 1507, 1822]; % [m] 7 fermate linea
end

%% ========================================================================
% 2. Definizione fasce orarie
% ========================================================================
%
% Ogni fascia definisce il numero di passeggeri per ogni tipo di veicolo.
% Il numero di passeggeri influenza la massa totale:
%   mass_total = mass_empty + nPax * pax_mass_kg
% e quindi l'energia inerziale calcolata.
%
% Valori di riferimento (modifica secondo i dati ATM/linea reale):
%   Bus filobus  (capienza max 135 pax):
%     Critica   -> 135 pax  (100% carico, ora di punta mattina)
%     Media     ->  68 pax  ( ~50% carico, pomeriggio)
%     Notturna  ->  14 pax  ( ~10% carico, sera/notte)
%
%   Navetta midibus (capienza max 43 pax)
%     Critica   ->  43 pax  (100% carico)
%     Media     ->  22 pax  ( ~50% carico)
%     Notturna  ->   5 pax  ( ~10% carico)
% ========================================================================

fasce = struct();

% --- Fascia CRITICA (ora di punta mattina) ---
fasce.critica.label          = 'Critica';
fasce.critica.nPax_bus       = 135;   % [pax] filobus pieno
fasce.critica.nPax_shuttle   =  43;   % [pax] navetta piena

% --- Fascia MEDIA (pomeriggio) ---
fasce.media.label            = 'Media';
fasce.media.nPax_bus         =  68;   % [pax] ~50% carico
fasce.media.nPax_shuttle     =  22;   % [pax] ~50% carico

% --- Fascia NOTTURNA (sera/notte) ---
fasce.notturna.label         = 'Notturna';
fasce.notturna.nPax_bus      =  14;   % [pax] ~10% carico
fasce.notturna.nPax_shuttle  =   5;   % [pax] ~10% carico

fascia_list = {fasce.critica, fasce.media, fasce.notturna};
fascia_keys = {'critica',    'media',    'notturna'};

%% ========================================================================
% 3. Parametri fissi dei veicoli (solo tara)
% ========================================================================
%
% Filobus ATM / Solaris Trollino 18 IMC
%   Tara:  19800 kg
% Navetta elettrica midibus 7.8 m
%   Tara:   8500 kg
% ========================================================================

m_empty_bus     = 19800;   % [kg] tara filobus
m_empty_shuttle =  8500;   % [kg] tara navetta

%% ========================================================================
% 4. Calcolo KPI per ogni configurazione e ogni fascia
% ========================================================================

for fi = 1:numel(fascia_list)
    fascia     = fascia_list{fi};
    fkey       = fascia_keys{fi};

    % --- Config BUS singolo ---
    cfg_bus.nVeh                 = 1;
    cfg_bus.mass_empty           = m_empty_bus;
    cfg_bus.nPax                 = fascia.nPax_bus;
    cfg_bus.pax_mass_kg          = pax_mass_kg;
    cfg_bus.v_stop               = v_stop;
    cfg_bus.t_min_stop           = t_min_stop;
    cfg_bus.s_stop               = mandatory_stops;
    cfg_bus.stop_exclusion_radius = stop_exclusion_radius;
    cfg_bus.route_target_s       = route_target_s;
    cfg_bus.L_vehicle            = 18;
    cfg_bus.fascia               = fascia.label;
    cfg_bus.t_trim_start         = t_trim_start;
    cfg_bus.t_trim_end           = t_trim_end;

    % --- Config 2 NAVETTE ---
    cfg_2veh.nVeh                 = 2;
    cfg_2veh.mass_empty           = [m_empty_shuttle m_empty_shuttle];
    cfg_2veh.nPax                 = [fascia.nPax_shuttle fascia.nPax_shuttle];
    cfg_2veh.pax_mass_kg          = pax_mass_kg;
    cfg_2veh.v_stop               = v_stop;
    cfg_2veh.t_min_stop           = t_min_stop;
    cfg_2veh.s_stop               = mandatory_stops;
    cfg_2veh.stop_exclusion_radius = stop_exclusion_radius;
    cfg_2veh.route_target_s       = route_target_s;
    cfg_2veh.L_vehicle            = 12;
    cfg_2veh.fascia               = fascia.label;
    cfg_2veh.t_trim_start         = t_trim_start;
    cfg_2veh.t_trim_end           = t_trim_end;

    % --- Config 3 NAVETTE ---
    cfg_3veh.nVeh                 = 3;
    cfg_3veh.mass_empty           = [m_empty_shuttle m_empty_shuttle m_empty_shuttle];
    cfg_3veh.nPax                 = [fascia.nPax_shuttle fascia.nPax_shuttle fascia.nPax_shuttle];
    cfg_3veh.pax_mass_kg          = pax_mass_kg;
    cfg_3veh.v_stop               = v_stop;
    cfg_3veh.t_min_stop           = t_min_stop;
    cfg_3veh.s_stop               = mandatory_stops;
    cfg_3veh.stop_exclusion_radius = stop_exclusion_radius;
    cfg_3veh.route_target_s       = route_target_s;
    cfg_3veh.L_vehicle            = 12;
    cfg_3veh.fascia               = fascia.label;
    cfg_3veh.t_trim_start         = t_trim_start;
    cfg_3veh.t_trim_end           = t_trim_end;

    % Calcolo KPI
    if exist('out_bus','var')
        assignin('base', ['KPI_bus_' fkey], compute_comparison_KPI(out_bus, cfg_bus));
    else
        warning('out_bus non trovata: caso Bus [%s] non calcolato.', fascia.label);
    end

    if exist('out_2veh','var')
        assignin('base', ['KPI_2veh_' fkey], compute_comparison_KPI(out_2veh, cfg_2veh));
    else
        warning('out_2veh non trovata: caso 2veh [%s] non calcolato.', fascia.label);
    end

    if exist('out_3veh','var')
        assignin('base', ['KPI_3veh_' fkey], compute_comparison_KPI(out_3veh, cfg_3veh));
    else
        warning('out_3veh non trovata: caso 3veh [%s] non calcolato.', fascia.label);
    end
end

%% ========================================================================
% 5. Tabella comparativa
% ========================================================================

comparison_table

%% ========================================================================
% 6. Plot KPI per fascia — energia e comfort
% ========================================================================

fascia_colors = [0.2 0.5 0.9;   % blu  -> Critica
                 0.9 0.6 0.1;   % arancione -> Media
                 0.4 0.7 0.3];  % verde -> Notturna
fascia_labels = {'Critica','Media','Notturna'};

% Raggruppa i casi per tipo veicolo
config_types = {'Bus','Platoon 2v','Platoon 3v'};

figure('Name','KPI per fascia oraria - Energia e Comfort','NumberTitle','off')
tiledlayout(3,3,'TileSpacing','compact','Padding','compact')

kpi_fields   = {'E_total_kWh',     'E_pax_kWh_per_pax', 'E_empty_total_kWh';
                'Ax_RMS_mps2',     'jerk_RMS_mps3',      'Ax_peak_max_mps2';
                'T_travel_mean_s', 'T_arrival_max_s',     'gap_min_global_m'};
kpi_labels   = {'Energia totale [kWh]',           'Energia/pax [kWh/pax]',       'Energia a vuoto [kWh]';
                'Acc. RMS [m/s^2]',                'Jerk RMS [m/s^3]',             'Acc. peak [m/s^2]';
                'Tempo viaggio medio [s]',         'Tempo arrivo max [s]',         'Gap minimo [m]'};

for r = 1:3
    for c = 1:3
        nexttile
        hold on; grid on
        field = kpi_fields{r,c};
        x_pos = 0;
        xtick_pos = zeros(1, 3*numel(fascia_keys));
        xtick_lbl = cell(1,  3*numel(fascia_keys));
        n_xt = 0;
        for fi = 1:3
            fkey = fascia_keys{fi};
            vals = NaN(1,3);
            knames = {['KPI_bus_' fkey],['KPI_2veh_' fkey],['KPI_3veh_' fkey]};
            for ci = 1:3
                if exist(knames{ci},'var')
                    K = eval(knames{ci});
                    % leggi dalla tabella Comparison
                    mask = Comparison.Fascia == fascia_labels{fi} & Comparison.Case == config_types{ci};
                    if any(mask)
                        if ismember(field, Comparison.Properties.VariableNames)
                            vals(ci) = Comparison.(field)(mask);
                        end
                    end
                end
            end
            positions = x_pos + (1:3);
            for ci = 1:3
                if ~isnan(vals(ci))
                    bar(positions(ci), vals(ci), 0.6, 'FaceColor', fascia_colors(fi,:), ...
                        'EdgeColor','k','LineWidth',0.5);
                end
            end
            xtick_pos(n_xt+1:n_xt+3) = positions;
            xtick_lbl(n_xt+1:n_xt+3) = config_types;
            n_xt = n_xt + 3;
            x_pos = x_pos + 4;
        end
        xticks(xtick_pos(1:n_xt))
        xticklabels(xtick_lbl(1:n_xt))
        xtickangle(30)
        title(kpi_labels{r,c})
        if r == 3 && c == 3
            yline(0,'--r','LineWidth',1.5)
        end
    end
end

% Legenda fasce
lgd_h = gobjects(1,3);
lgd_s = cell(1,3);
n_lgd = 0;
for fi = 1:3
    n_lgd = n_lgd + 1;
    lgd_h(n_lgd) = patch(NaN,NaN,fascia_colors(fi,:),'EdgeColor','k');
    lgd_s{n_lgd} = fascia_labels{fi};
end
legend(lgd_h(1:n_lgd), lgd_s(1:n_lgd), 'Location','southoutside','Orientation','horizontal','NumColumns',3)

%% ========================================================================
% 7. Plot stop non obbligatori
% ========================================================================
% Gli stop non dipendono dal carico passeggeri, quindi usiamo la fascia
% critica come riferimento (risultati identici per le 3 fasce).
% ========================================================================

names_base = cell(1,3);
nb = 0;
if exist('KPI_bus_critica','var');  nb=nb+1; names_base{nb}='Bus'; end
if exist('KPI_2veh_critica','var'); nb=nb+1; names_base{nb}='Platoon 2v'; end
if exist('KPI_3veh_critica','var'); nb=nb+1; names_base{nb}='Platoon 3v'; end
names_base = names_base(1:nb);

if ~isempty(names_base)
    kpi_stop = cell(1,3);
    nks = 0;
    vnames = {'KPI_bus_critica','KPI_2veh_critica','KPI_3veh_critica'};
    for ci = 1:3
        if exist(vnames{ci},'var')
            nks = nks + 1;
            kpi_stop{nks} = eval(vnames{ci});
        end
    end
    kpi_stop = kpi_stop(1:nks);

    nC = nks;
    Nstop_mat = NaN(nC,3);
    Tstop_mat = NaN(nC,3);
    for ci = 1:nC
        Ns = kpi_stop{ci}.stop.N_stop_vehicle;
        Ts = kpi_stop{ci}.stop.T_stop_vehicle_s;
        for v = 1:min(numel(Ns),3)
            Nstop_mat(ci,v) = Ns(v);
            Tstop_mat(ci,v) = Ts(v);
        end
    end

    figure('Name','KPI - Stop non obbligatori (fascia critica)','NumberTitle','off')
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact')

    nexttile
    bar(categorical(names_base), Nstop_mat)
    grid on
    ylabel('N. stop [-]')
    legend({'Veh 1','Veh 2','Veh 3'},'Location','best')
    title('Stop non obbligatori per veicolo')

    nexttile
    bar(categorical(names_base), Tstop_mat)
    grid on
    ylabel('Tempo stop [s]')
    legend({'Veh 1','Veh 2','Veh 3'},'Location','best')
    title('Tempo stop non obbligatori per veicolo')
end

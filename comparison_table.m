%==========================================================================
% COMPARISON_TABLE
%==========================================================================
% Costruisce la tabella comparativa KPI a partire dalle strutture
% KPI_* presenti nel workspace.
%
% Prerequisito: eseguire plot_kpi.m, che popola le strutture:
%   KPI_bus_critica, KPI_bus_media, KPI_bus_notturna
%   KPI_2veh_critica, KPI_2veh_media, KPI_2veh_notturna
%   KPI_3veh_critica, KPI_3veh_media, KPI_3veh_notturna
%==========================================================================

%% Raccolta casi disponibili

all_vars = {
    'KPI_bus_critica',   'Bus',        'Critica';
    'KPI_bus_media',     'Bus',        'Media';
    'KPI_bus_notturna',  'Bus',        'Notturna';
    'KPI_2veh_critica',  'Platoon 2v', 'Critica';
    'KPI_2veh_media',    'Platoon 2v', 'Media';
    'KPI_2veh_notturna', 'Platoon 2v', 'Notturna';
    'KPI_3veh_critica',  'Platoon 3v', 'Critica';
    'KPI_3veh_media',    'Platoon 3v', 'Media';
    'KPI_3veh_notturna', 'Platoon 3v', 'Notturna';
};

n_max = size(all_vars,1);
case_names = cell(n_max,1);
KPI_list   = cell(n_max,1);
n_found = 0;

for r = 1:size(all_vars,1)
    vname = all_vars{r,1};
    if exist(vname,'var')
        n_found = n_found + 1;
        case_names{n_found} = sprintf('%s - %s', all_vars{r,2}, all_vars{r,3});
        KPI_list{n_found}   = eval(vname);
    end
end
case_names = case_names(1:n_found);
KPI_list   = KPI_list(1:n_found);

if isempty(KPI_list)
    error('Nessun KPI trovato. Eseguire prima plot_kpi.m.');
end

nCases = numel(KPI_list);

%% Preallocazione colonne

Case           = strings(nCases,1);
Fascia         = strings(nCases,1);
nVeh_col       = NaN(nCases,1);
N_pax_total    = NaN(nCases,1);

mass_empty_tot = NaN(nCases,1);
mass_pax_tot   = NaN(nCases,1);

E_total_kWh        = NaN(nCases,1);
E_empty_total_kWh  = NaN(nCases,1);
E_pax_total_kWh    = NaN(nCases,1);
E_pax_kWh_per_pax  = NaN(nCases,1);

T_travel_mean_s    = NaN(nCases,1);
T_arrival_max_s    = NaN(nCases,1);

Ax_RMS_mps2        = NaN(nCases,1);
Ax_peak_max_mps2   = NaN(nCases,1);
jerk_RMS_mps3      = NaN(nCases,1);
u_peak_max_mps3    = NaN(nCases,1);

N_stop_v1 = NaN(nCases,1); N_stop_v2 = NaN(nCases,1); N_stop_v3 = NaN(nCases,1);
T_stop_v1_s = NaN(nCases,1); T_stop_v2_s = NaN(nCases,1); T_stop_v3_s = NaN(nCases,1);

gap_min_global_m   = NaN(nCases,1);

exec_mean_s   = NaN(nCases,1);
exec_median_s = NaN(nCases,1);
exec_p95_s    = NaN(nCases,1);

%% Riempimento

for i = 1:nCases
    K = KPI_list{i};
    parts = strsplit(case_names{i},' - ');
    Case(i)   = string(parts{1});
    Fascia(i) = string(parts{2});

    nVeh_col(i)    = K.meta.nVeh;
    N_pax_total(i) = K.meta.nPax_total;
    mass_empty_tot(i) = sum(K.meta.mass_empty_kg);
    mass_pax_tot(i)   = sum(K.meta.mass_pax_kg);

    E_total_kWh(i)       = K.energy.E_total_kWh;
    E_empty_total_kWh(i) = K.energy.E_empty_total_kWh;
    E_pax_total_kWh(i)   = K.energy.E_pax_total_kWh;
    E_pax_kWh_per_pax(i) = K.energy.E_pax_kWh_per_pax;

    T_travel_mean_s(i) = K.time.T_travel_mean_s;
    T_arrival_max_s(i) = K.time.T_arrival_max_s;

    Ax_RMS_mps2(i)      = K.comfort.Ax_RMS_aggregate;
    Ax_peak_max_mps2(i) = K.comfort.Ax_peak_max;
    jerk_RMS_mps3(i)    = K.comfort.u_RMS_aggregate;
    u_peak_max_mps3(i)  = K.comfort.u_peak_max;

    Nstop = K.stop.N_stop_vehicle;
    Tstop = K.stop.T_stop_vehicle_s;
    if numel(Nstop)>=1; N_stop_v1(i)=Nstop(1); T_stop_v1_s(i)=Tstop(1); end
    if numel(Nstop)>=2; N_stop_v2(i)=Nstop(2); T_stop_v2_s(i)=Tstop(2); end
    if numel(Nstop)>=3; N_stop_v3(i)=Nstop(3); T_stop_v3_s(i)=Tstop(3); end

    gap_min_global_m(i) = K.gap.gap_min_global;

    exec_mean_s(i)   = K.solver.execution_mean_s;
    exec_median_s(i) = K.solver.execution_median_s;
    exec_p95_s(i)    = K.solver.execution_p95_s;
end

%% Tabella finale

Comparison = table( ...
    Case, Fascia, nVeh_col, N_pax_total, mass_empty_tot, mass_pax_tot, ...
    E_total_kWh, E_empty_total_kWh, E_pax_total_kWh, E_pax_kWh_per_pax, ...
    T_travel_mean_s, T_arrival_max_s, ...
    Ax_RMS_mps2, Ax_peak_max_mps2, jerk_RMS_mps3, u_peak_max_mps3, ...
    N_stop_v1, N_stop_v2, N_stop_v3, T_stop_v1_s, T_stop_v2_s, T_stop_v3_s, ...
    gap_min_global_m, ...
    exec_mean_s, exec_median_s, exec_p95_s);

disp(Comparison)

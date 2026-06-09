%% ============================================================
% PRECOMPUTE k_road FROM PL_jrl_chs MAP
% ============================================================
%
% Output:
%   s_kroad_map      [m]
%   k_road_map       [1/m]
%
% La curvatura viene calcolata tramite Map_loc:
%
%   [~,~,~,C_c,~,~] = Map_loc(map_road, s)
%
% e poi salvata in una lookup table da usare online.
% ============================================================

%% Load road map

data = load('PL_jrl_chs.mat');

% Nel tuo snippet usavi:
%   map_road = coder.load('PL_jrl_chs.mat');
%   Map_loc(map_road.map_road, s)
%
% Quindi qui assumo che il file contenga una variabile/struct
% con campo map_road.

if isfield(data, 'map_road')
    map_data = data.map_road;
else
    error('Il file PL_jrl_chs.mat non contiene la variabile map_road.')
end

if isstruct(map_data) && isfield(map_data, 'map_road')
    road_for_MapLoc = map_data.map_road;
else
    road_for_MapLoc = map_data;
end

%% Route length

% Imposta qui la lunghezza massima della tua mappa.
% Per il tuo scenario il TL più avanti è circa 1823 m.
% Se la mappa è più lunga, aumenta questo valore.

s_start = 0;
s_end   = 1825;       % [m]
ds      = 0.10;       % [m] risoluzione spaziale lookup

s_kroad_map = (s_start:ds:s_end).';
n_pts = length(s_kroad_map);

k_road_map = zeros(n_pts,1);

%% Curvature extraction with Map_loc

for jj = 1:n_pts

    s = s_kroad_map(jj);

    [~,~,~,C_c,~,~] = Map_loc(road_for_MapLoc, s);

    if isfinite(C_c)
        k_road_map(jj) = abs(C_c);
    else
        k_road_map(jj) = 0;
    end

end

%% Numerical cleaning

% Elimina rumore numerico su rettilinei
k_min = 1e-7;             % [1/m]
k_road_map(k_road_map < k_min) = 0;

% Limite superiore anti-outlier
% R_min = 5 m -> k_max = 0.2 1/m
R_min = 5;                % [m]
k_max = 1/R_min;          % [1/m]
k_road_map(k_road_map > k_max) = k_max;

%% Optional smoothing

use_smoothing = true;

if use_smoothing
    win = 11;

    if exist('sgolayfilt','file') == 2
        k_road_map = sgolayfilt(k_road_map, 3, win);
    else
        k_road_map = movmean(k_road_map, win);
    end

    k_road_map = abs(k_road_map);
    k_road_map(k_road_map < k_min) = 0;
    k_road_map(k_road_map > k_max) = k_max;
end

%% Save lookup table

save('k_road_PL_jrl_chs_precomputed.mat', ...
     's_kroad_map', ...
     'k_road_map', ...
     'ds', ...
     's_start', ...
     's_end')

fprintf('k_road map precomputed successfully.\n')
fprintf('Number of points: %d\n', n_pts)
fprintf('s range: %.2f -> %.2f m\n', s_kroad_map(1), s_kroad_map(end))
fprintf('max(k_road): %.6f 1/m\n', max(k_road_map))

if max(k_road_map) > 0
    fprintf('minimum equivalent radius: %.2f m\n', 1/max(k_road_map))
end

%% Plot check
% 
% figure
% plot(s_kroad_map, k_road_map, 'LineWidth', 1.2)
% grid on
% xlabel('s [m]')
% ylabel('k_{road} [1/m]')
% title('Precomputed road curvature')
% 
% Ay_max = 1.5;  % [m/s^2]
% vmax_lat = sqrt(Ay_max ./ max(k_road_map, 1e-9));
% 
% figure
% plot(s_kroad_map, vmax_lat*3.6, 'LineWidth', 1.2)
% grid on
% xlabel('s [m]')
% ylabel('v_{max,lat} [km/h]')
% title('Lateral-acceleration speed limit')
% ylim([0 80])
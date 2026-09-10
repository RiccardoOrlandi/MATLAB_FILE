%% PLOT CHEBYSHEV GRID - PRESENTATION DIAGNOSTICS
% Confronta:
%   1) griglia uniforme;
%   2) Chebyshev-Lobatto pura;
%   3) griglia Chebyshev-Lobatto attenuata implementata nell'NMPC.
%
% Configurazione:
%   N = 35 intervalli
%   N+1 = 36 nodi
%   T_hor = 50 s
%   Ts_ctrl = 1 s
%
% Lo script salva anche due immagini PNG ad alta risoluzione.

clear;
close all;
clc;

%% Parametri
N       = 35;
T_hor   = 50.0;
Ts_ctrl = 1.0;

k  = (0:N)';
xi = k/N;

%% 1. Griglia uniforme normalizzata
tau_uniform = xi;

%% 2. Griglia Chebyshev-Lobatto pura normalizzata
tau_cheb = 0.5*(1 - cos(pi*xi));

%% 3. Calcolo del blending
% Si impone che il primo intervallo della griglia attenuata sia Ts_ctrl.
dtau_uniform = tau_uniform(2) - tau_uniform(1);
dtau_cheb_1  = tau_cheb(2)   - tau_cheb(1);

beta_cheb = (Ts_ctrl/T_hor - dtau_uniform) / ...
            (dtau_cheb_1 - dtau_uniform);

if beta_cheb < 0 || beta_cheb > 1
    error('Il coefficiente beta_cheb non appartiene a [0,1].');
end

tau_blend = (1-beta_cheb)*tau_uniform + beta_cheb*tau_cheb;

%% Conversione in secondi
t_uniform = T_hor*tau_uniform;
t_cheb    = T_hor*tau_cheb;
t_blend   = T_hor*tau_blend;

% Forzatura esatta degli estremi
t_uniform(1) = 0; t_uniform(end) = T_hor;
t_cheb(1)    = 0; t_cheb(end)    = T_hor;
t_blend(1)   = 0; t_blend(end)   = T_hor;

dt_uniform = diff(t_uniform);
dt_cheb    = diff(t_cheb);
dt_blend   = diff(t_blend);

%% Indicatori numerici
density_uniform = 1./dt_uniform;
density_cheb    = 1./dt_cheb;
density_blend   = 1./dt_blend;

node_shift = t_blend - t_uniform;

zones = [0 10; 20 30; 40 50];
zone_labels = {'0-10 s','20-30 s','40-50 s'};

count_uniform = zeros(1,size(zones,1));
count_blend   = zeros(1,size(zones,1));
count_cheb    = zeros(1,size(zones,1));

for ii = 1:size(zones,1)
    a = zones(ii,1);
    b = zones(ii,2);

    count_uniform(ii) = sum(t_uniform >= a & t_uniform <= b);
    count_blend(ii)   = sum(t_blend   >= a & t_blend   <= b);
    count_cheb(ii)    = sum(t_cheb    >= a & t_cheb    <= b);
end

%% Stampa riepilogo
fprintf('\n============================================================\n');
fprintf('GRIGLIA CHEBYSHEV-LOBATTO ATTENUATA\n');
fprintf('============================================================\n');
fprintf('N intervalli             : %d\n',N);
fprintf('N nodi                   : %d\n',N+1);
fprintf('Orizzonte                : %.3f s\n',T_hor);
fprintf('Sample time controllore  : %.3f s\n',Ts_ctrl);
fprintf('beta Chebyshev           : %.9f\n',beta_cheb);
fprintf('Componente uniforme      : %.3f %%\n',100*(1-beta_cheb));
fprintf('Componente Chebyshev     : %.3f %%\n',100*beta_cheb);
fprintf('dt minimo implementato   : %.6f s\n',min(dt_blend));
fprintf('dt massimo implementato  : %.6f s\n',max(dt_blend));
fprintf('dt medio implementato    : %.6f s\n',mean(dt_blend));
fprintf('dt min Chebyshev puro    : %.6f s\n',min(dt_cheb));
fprintf('dt max Chebyshev puro    : %.6f s\n',max(dt_cheb));
fprintf('============================================================\n\n');

summary_table = table( ...
    (0:N-1)', ...
    t_blend(1:end-1), ...
    t_blend(2:end), ...
    dt_blend, ...
    density_blend, ...
    'VariableNames', ...
    {'Intervallo','t_inizio_s','t_fine_s','dt_s','densita_nodi_Hz'});

disp(summary_table);

%% FIGURA 1 - Funzionamento generale
fig1 = figure('Name','Chebyshev grid - overview','Color','w');
tl1 = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% A) Distribuzione dei nodi sull'asse temporale
nexttile;
plot(t_uniform, 3*ones(size(t_uniform)),'o-','LineWidth',1.0,'MarkerSize',4);
hold on;
plot(t_blend,   2*ones(size(t_blend)),  'o-','LineWidth',1.2,'MarkerSize',5);
plot(t_cheb,    1*ones(size(t_cheb)),   'o-','LineWidth',1.0,'MarkerSize',4);
grid on;
xlim([0 T_hor]);
ylim([0.5 3.5]);
yticks([1 2 3]);
yticklabels({'Chebyshev pura','Implementata','Uniforme'});
xlabel('Tempo nell''orizzonte [s]');
title('Distribuzione dei nodi');

% B) Durata degli intervalli
nexttile;
plot(1:N,dt_uniform,'--','LineWidth',1.2);
hold on;
plot(1:N,dt_blend,'o-','LineWidth',1.3,'MarkerSize',4);
plot(1:N,dt_cheb,':','LineWidth',1.3);
yline(Ts_ctrl,'--','T_s controller');
grid on;
xlabel('Indice intervallo k');
ylabel('\Delta t_k [s]');
title('Durata degli intervalli');
legend('Uniforme','Chebyshev attenuata','Chebyshev pura', ...
       'Location','best');

% C) Mappatura indice nodo -> tempo
nexttile;
plot(k,t_uniform,'--','LineWidth',1.2);
hold on;
plot(k,t_blend,'o-','LineWidth',1.3,'MarkerSize',4);
plot(k,t_cheb,':','LineWidth',1.3);
grid on;
xlabel('Indice nodo k');
ylabel('Tempo del nodo t_k [s]');
title('Mappatura dei nodi');
legend('Uniforme','Chebyshev attenuata','Chebyshev pura', ...
       'Location','best');

% D) Spostamento rispetto alla griglia uniforme
nexttile;
plot(k,node_shift,'o-','LineWidth',1.3,'MarkerSize',4);
yline(0,'--');
grid on;
xlabel('Indice nodo k');
ylabel('t_k^{blend} - t_k^{uniforme} [s]');
title('Spostamento dei nodi');

title(tl1,sprintf( ...
    'Griglia Chebyshev-Lobatto attenuata: N=%d, T=%.0f s, \\beta=%.4f', ...
    N,T_hor,beta_cheb));

exportgraphics(fig1,'chebyshev_overview_N35.png','Resolution',300);

%% FIGURA 2 - Densità e quantificazione
fig2 = figure('Name','Chebyshev grid - density','Color','w');
tl2 = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

% A) Densità nodale
nexttile;
plot(1:N,density_uniform,'--','LineWidth',1.2);
hold on;
plot(1:N,density_blend,'o-','LineWidth',1.3,'MarkerSize',4);
plot(1:N,density_cheb,':','LineWidth',1.3);
grid on;
xlabel('Indice intervallo k');
ylabel('Densità 1/\Delta t_k [nodi/s]');
title('Densità temporale');
legend('Uniforme','Chebyshev attenuata','Chebyshev pura', ...
       'Location','best');

% B) Numero di nodi in tre zone dell'orizzonte
nexttile;
zone_counts = [count_uniform(:),count_blend(:),count_cheb(:)];
bar(zone_counts);
grid on;
xticks(1:numel(zone_labels));
xticklabels(zone_labels);
ylabel('Numero di nodi');
title('Concentrazione dei nodi nelle zone');
legend('Uniforme','Chebyshev attenuata','Chebyshev pura', ...
       'Location','best');

title(tl2,'Quantificazione della distribuzione non uniforme');

exportgraphics(fig2,'chebyshev_density_N35.png','Resolution',300);

%% Salvataggio tabella
writetable(summary_table,'chebyshev_grid_N35.csv');

fprintf('File generati:\n');
fprintf('  - chebyshev_overview_N35.png\n');
fprintf('  - chebyshev_density_N35.png\n');
fprintf('  - chebyshev_grid_N35.csv\n');

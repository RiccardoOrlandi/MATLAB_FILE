%==========================================================================
% PLOT_PARAMETRI_SIMULAZIONE.m  –  B-GLOSA 2 VEH
% Legge TUTTI i parametri direttamente dal workspace.
% Esegui DOPO aver caricato out.mat (o dopo la simulazione).
%==========================================================================
close all; clc;

%% --------------------------------------------------------
%  CONTROLLO VARIABILI OBBLIGATORIE
%% --------------------------------------------------------
required = {'N','N_near','N_far','Ts_near','Ts_far','Ts','T_hor', ...
            'Ax_min','Ax_max','Ay_max','jerk_min','jerk_max','V_max', ...
            'v_eps','L_platoon','d_gap','gap_min','eps_aero', ...
            't_dwell','eps_stop_back','eps_stop_front','eps_stop_v','eps_stop_a', ...
            'eps_stop_num','d_safe_TL','s_max','Qx1','QN1','time_grid'};
missing = {};
for ri = 1:length(required)
    if ~exist(required{ri}, 'var')
        missing{end+1} = required{ri};
    end
end
if ~isempty(missing)
    warning('Variabili mancanti nel workspace: %s', strjoin(missing, ', '));
end

%% --------------------------------------------------------
%  RECUPERO GRIGLIA E PARAMETRI DERIVATI
%% --------------------------------------------------------
dt_grid   = diff(time_grid(:));
V_max_kmh = V_max * 3.6;

% Pesi dalla diagonale di W1 (Qx1, 17 termini per 2VEH):
% [1-2]=dist, [3-4]=jerk, [5-6]=Ax, [7-8]=stop_tgt,
% [9-10]=dwell_pos, [11-12]=dwell_vel, [13-14]=dwell_acc,
% [15]=aero_12, [16-17]=slack_Jmin
w_dist      = Qx1(1);
w_jerk      = Qx1(3);
w_Ax        = Qx1(5);
w_stop_tgt  = Qx1(7);
w_dwell_pos = Qx1(9);
w_dwell_vel = Qx1(11);
w_dwell_acc = Qx1(13);
w_aero      = Qx1(15);
w_slack     = Qx1(16);
w_dist_N    = QN1(1);

if exist('Levenberg_Marquardt','var')
    Lev_Marq = Levenberg_Marquardt;
else
    Lev_Marq = 1e-4;
end

%% --------------------------------------------------------
%  PALETTE
%% --------------------------------------------------------
cB=[0.12,0.47,0.71]; cO=[1.00,0.50,0.05]; cG=[0.17,0.63,0.17];
cR=[0.84,0.15,0.16]; cP=[0.58,0.40,0.74]; cK=[0.50,0.50,0.50];
cA=[0.91,0.63,0.00];

%% ========================================================
%  STAMPA COMMAND WINDOW
%% ========================================================
sep  = repmat('=',1,72);
sep2 = repmat('-',1,72);

fprintf("\n%s\n", sep);
fprintf("  PARAMETRI SIMULAZIONE  -  B-GLOSA  2VEH\n");
fprintf("  Griglia non uniforme: %d near (%.1f s) + %d far (%.1f s)\n", N_near, Ts_near, N_far, Ts_far);
fprintf("%s\n\n", sep);

fprintf("%s\n [1] ORIZZONTE PREDITTIVO\n%s\n", sep2, sep2);
fprintf("  N              = %d          intervalli OCP (nodi = %d)\n", N, N+1);
fprintf("  N_near/N_far   = %d / %d    passi near/far\n", N_near, N_far);
fprintf("  Ts_near/far    = %.1f / %.1f s   passo temporale near / far\n", Ts_near, Ts_far);
fprintf("  T_hor          = %.1f s      orizzonte totale\n", T_hor);
fprintf("  Ts_ctrl        = %.1f s      sample time controller\n", Ts);
fprintf("\n");

fprintf("%s\n [2] VINCOLI CINEMATICI\n%s\n", sep2, sep2);
fprintf("  Ax_min/max     = %.2f / %.2f m/s^2   acc. longitudinale\n", Ax_min, Ax_max);
fprintf("  Ay_max         = %.2f m/s^2           acc. laterale (v^2*k <= Ay_max)\n", Ay_max);
fprintf("  jerk_min/max   = %.2f / %.2f m/s^3   ingresso di controllo\n", jerk_min, jerk_max);
fprintf("  V_max          = %.0f km/h (%.4f m/s)  limite velocita'\n", V_max_kmh, V_max);
fprintf("  v_eps          = %.3f m/s             tolleranza velocita' negativa\n", v_eps);
fprintf("\n");

fprintf("%s\n [3] GAP INTER-VEICOLARE\n%s\n", sep2, sep2);
fprintf("  L_platoon      = %.1f m     lunghezza veicolo\n", L_platoon);
fprintf("  d_gap          = %.1f m     gap netto desiderato\n", d_gap);
fprintf("  gap_min        = %.1f m     gap netto minimo (vincolo hard OCP)\n", gap_min);
fprintf("  eps_aero       = %.2f m^2   smoothing costo spacing\n", eps_aero);
if exist('gap_off','var')
    fprintf("  gap_off        = %.1f m     soglia spegnimento costo spacing\n", gap_off);
end
fprintf("\n");

fprintf("%s\n [4] FERMATE BUS\n%s\n", sep2, sep2);
fprintf("  t_dwell        = %d s       sosta obbligatoria\n", t_dwell);
fprintf("  eps_stop_back  = %.1f m     tolleranza dietro la fermata\n", eps_stop_back);
fprintf("  eps_stop_front = %.1f m     tolleranza davanti la fermata\n", eps_stop_front);
fprintf("  eps_stop_v     = %.2f m/s   velocita' max durante sosta\n", eps_stop_v);
fprintf("  eps_stop_a     = %.2f m/s^2 acc. max durante sosta\n", eps_stop_a);
fprintf("  eps_stop_num   = %.2f m     tolleranza numerica vincolo approccio\n", eps_stop_num);
fprintf("\n");

fprintf("%s\n [5] SEMAFORI\n%s\n", sep2, sep2);
fprintf("  d_safe_TL      = %.1f m     margine sicurezza stop line\n", d_safe_TL);
fprintf("  s_max          = %d m       lunghezza percorso\n", s_max);
fprintf("\n");

fprintf("%s\n [6] PESI COST FUNCTION  (letti da Qx1, 2VEH)\n%s\n", sep2, sep2);
fprintf("  w_dist         = %7.2f    avanzamento (term. principale)\n", w_dist);
fprintf("  w_jerk         = %7.2f    jerk (comfort)\n", w_jerk);
fprintf("  w_Ax           = %7.2f    accelerazione longitudinale\n", w_Ax);
fprintf("  w_stop_tgt     = %7.2f    attrazione verso fermata\n", w_stop_tgt);
fprintf("  w_dwell_pos    = %7.2f    posizione durante sosta\n", w_dwell_pos);
fprintf("  w_dwell_vel    = %7.2f    velocita' durante sosta\n", w_dwell_vel);
fprintf("  w_dwell_acc    = %7.2f    acc. durante sosta\n", w_dwell_acc);
fprintf("  w_aero         = %7.2f    spacing/aero coppia 1-2\n", w_aero);
fprintf("  w_slack_Jmin   = %7.2f    slack jerk min (rilassamento feasibility)\n", w_slack);
fprintf("  [terminal]  w_dist_N = %.2f  (da QN1)\n", w_dist_N);
fprintf("\n");

fprintf("%s\n [7] SOLVER ACADO\n%s\n", sep2, sep2);
fprintf("  Levenberg-Marquardt = %.0e\n", Lev_Marq);
fprintf("  Integratore         = RK4  (MULTIPLE_SHOOTING)\n");
fprintf("  N_integrator_steps  = %d\n", N);
fprintf("  QP solver           = qpOASES3  (FULL_CONDENSING_N2)\n");
fprintf("  Appross. Hessiano   = GAUSS_NEWTON\n");
fprintf("\n%s\n\n", sep);

%% ========================================================
%  FIGURA 1 - Griglia predittiva
%% ========================================================
figure("Name","Fig1 - Griglia predittiva","NumberTitle","off","Position",[30 490 660 260]);
bar(1:N, dt_grid, 1, "FaceColor", cB, "EdgeColor", "none"); hold on;
yline(Ts_near, "--", "Color", cR, "LineWidth", 1.4);
yline(Ts_far,  "--", "Color", cA, "LineWidth", 1.4);
text(N_near/2,      Ts_near+0.06, sprintf("Ts_{near} = %.1f s", Ts_near), "Color", cR, "FontSize", 9, "HorizontalAlignment", "center");
text(N_near+N_far/2, Ts_far+0.06, sprintf("Ts_{far}  = %.1f s", Ts_far),  "Color", cA, "FontSize", 9, "HorizontalAlignment", "center");
xlabel("Indice intervallo k  [-]"); ylabel("\Delta t_k  [s]");
title(sprintf("Griglia predittiva non uniforme    N=%d,  T_{hor}=%.1f s,  Ts_{ctrl}=%.1f s", N, T_hor, Ts), "FontSize", 10);
grid on; box on; xlim([0.5 N+0.5]); ylim([0 1.6]);

%% ========================================================
%  FIGURA 2 - Vincoli cinematici
%% ========================================================
figure("Name","Fig2 - Vincoli cinematici","NumberTitle","off","Position",[700 490 660 260]);
lbls = {"Ax_{min}","Ax_{max}","Ay_{max}","j_{min}","j_{max}","V_{max}","v_{eps}"};
vals = [Ax_min, Ax_max, Ay_max, jerk_min, jerk_max, V_max, v_eps];
unts = {"m/s^2","m/s^2","m/s^2","m/s^3","m/s^3","m/s","m/s"};
clrs = [cR;cR;cA;cP;cP;cB;cK];
bh = bar(abs(vals), 0.65, "FaceColor", "flat"); bh.CData = clrs;
axv = gca; axv.XTick = 1:length(lbls); axv.XTickLabel = lbls;
ylabel("|valore|"); title("Vincoli cinematici", "FontSize", 10); grid on; box on;
for i = 1:length(vals)
    text(i, abs(vals(i))+0.04, sprintf("%.3g\n%s", vals(i), unts{i}), "HorizontalAlignment", "center", "FontSize", 7.5);
end

%% ========================================================
%  FIGURA 3 - Pesi cost function (letti da Qx1)
%% ========================================================
figure("Name","Fig3 - Pesi cost function","NumberTitle","off","Position",[30 170 660 270]);
lww = {"dist","jerk","Ax","stop tgt","dwell pos","dwell vel","dwell acc","aero","slack Jmin"};
wws = [w_dist, w_jerk, w_Ax, w_stop_tgt, w_dwell_pos, w_dwell_vel, w_dwell_acc, w_aero, w_slack];
clw = [cB;cP;cA;cG;cR;cR;cR;cK;[0.5 0.5 0.5]];
bw = bar(wws, 0.7, "FaceColor", "flat"); bw.CData = clw;
axw = gca; axw.XTick = 1:length(lww); axw.XTickLabel = lww; axw.XTickLabelRotation = 25;
axw.YScale = "log";
ylabel("Peso Q  [-]  (scala log)"); title("Pesi cost function  (da Qx1 e QN1 del workspace)", "FontSize", 10);
grid on; box on;
for i = 1:length(wws)
    text(i, wws(i)*1.1, num2str(wws(i)), "HorizontalAlignment", "center", "FontSize", 8);
end

%% ========================================================
%  FIGURA 4 - Costo gap inter-veicolare
%% ========================================================
figure("Name","Fig4 - Costo gap","NumberTitle","off","Position",[700 170 660 270]);
gvec = linspace(0, 12, 600);
ph_v   = gvec - d_gap;
huber  = sqrt(ph_v.^2 + eps_aero) - sqrt(eps_aero);
if exist('gap_off','var') && exist('eps_win','var')
    delta_w   = gap_off - gvec;
    w_r       = (delta_w + sqrt(delta_w.^2 + eps_win)) / 2;
    delta_w_0 = gap_off - d_gap;
    w_r_0     = (delta_w_0 + sqrt(delta_w_0.^2 + eps_win)) / 2;
    w_n       = (w_r ./ w_r_0).^2;
    ca        = huber .* w_n / d_gap;
    label_gap = sprintf("Costo con finestra  (gap_{off}=%.0f m)", gap_off);
else
    ca    = huber / d_gap;
    label_gap = "Costo pseudo-Huber simmetrico";
end
plot(gvec, ca, "Color", cB, "LineWidth", 2); hold on;
fill([0 gap_min gap_min 0], [min(ca)-0.05 min(ca)-0.05 max(ca)+0.05 max(ca)+0.05], cR, "FaceAlpha", 0.10, "EdgeColor", "none");
xline(gap_min, ":", "Color", cR,  "LineWidth", 1.5);
xline(d_gap,   "--", "Color", cA, "LineWidth", 1.5);
if exist('gap_off','var')
    xline(gap_off, "--", "Color", cK, "LineWidth", 1.2);
    text(gap_off+0.2, max(ca)*0.3, sprintf("gap_{off}=%.0f m", gap_off), "Color", cK, "FontSize", 9);
end
text(gap_min+0.15, max(ca)*0.6, sprintf("gap_{min}=%.0f m", gap_min), "Color", cR, "FontSize", 9);
text(d_gap+0.15,   max(ca)*0.8, sprintf("d_{gap}=%.0f m",   d_gap),   "Color", cA, "FontSize", 9);
xlabel("Gap netto coppia 1-2  [m]"); ylabel("cost\_aero  [-]");
title(label_gap, "FontSize", 10);
grid on; box on;

fprintf(">> Script completato: output in Command Window + 4 figure.\n");

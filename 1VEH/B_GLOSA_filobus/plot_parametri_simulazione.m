%==========================================================================
% PLOT_PARAMETRI_SIMULAZIONE.m  –  B-GLOSA  1VEH  4TL
% Legge i parametri direttamente dal workspace.
% Esegui DOPO aver eseguito NMPC_init_1veh_tl4.m
% (o dopo aver caricato out.mat che include le variabili di init).
%==========================================================================
close all; clc;

%% --------------------------------------------------------
%  CONTROLLO VARIABILI OBBLIGATORIE
%% --------------------------------------------------------
required = {'N','Ts','Ax_min','Ax_max','Ay_max', ...
            'jerk_min','jerk_max','V_max', ...
            'v_eps','L_platoon','t_dwell', ...
            'eps_stop_back','eps_stop_front','eps_stop_v','eps_stop_a', ...
            'd_safe_TL','s_max','Qx1','QN1'};
n_req = length(required);  missing_flag = false(1,n_req);
for ri = 1:length(required)
    missing_flag(ri) = ~exist(required{ri},'var');
end
if any(missing_flag)
    warning('Variabili mancanti nel workspace: %s', strjoin(required(missing_flag),', '));
end

%% --------------------------------------------------------
%  PARAMETRI DERIVATI
%% --------------------------------------------------------
T_hor     = N * Ts;
V_max_kmh = V_max * 3.6;

% Pesi da Qx1 (7 termini):
%   1 dist, 2 jerk, 3 Ax, 4 stop_tgt, 5 dwell_pos, 6 dwell_vel, 7 dwell_acc
w_dist      = Qx1(1);
w_jerk      = Qx1(2);
w_Ax        = Qx1(3);
w_stop_tgt  = Qx1(4);
w_dwell_pos = Qx1(5);
w_dwell_vel = Qx1(6);
w_dwell_acc = Qx1(7);
w_dist_N    = QN1(1);

if ~exist('Levenberg_Marquardt','var'); Lev_Marq = 1e-4; else; Lev_Marq = Levenberg_Marquardt; end

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
fprintf("  PARAMETRI SIMULAZIONE  -  B-GLOSA  1VEH  4TL\n");
fprintf("  Griglia uniforme: N=%d passi da Ts=%.1f s  ->  T_hor=%.0f s\n", N, Ts, T_hor);
fprintf("%s\n\n", sep);

fprintf("%s\n [1] ORIZZONTE PREDITTIVO\n%s\n", sep2, sep2);
fprintf("  N         = %d         intervalli OCP (nodi = %d)\n", N, N+1);
fprintf("  Ts        = %.1f s     passo temporale uniforme\n", Ts);
fprintf("  T_hor     = %.0f s     orizzonte totale\n", T_hor);
fprintf("\n");

fprintf("%s\n [2] VINCOLI CINEMATICI\n%s\n", sep2, sep2);
fprintf("  Ax_min/max  = %.2f / %.2f m/s^2\n", Ax_min, Ax_max);
fprintf("  Ay_max      = %.2f m/s^2\n", Ay_max);
fprintf("  jerk_min/max= %.2f / %.2f m/s^3\n", jerk_min, jerk_max);
fprintf("  V_max       = %.0f km/h (%.4f m/s)\n", V_max_kmh, V_max);
fprintf("  v_eps       = %.3f m/s\n", v_eps);
fprintf("\n");

fprintf("%s\n [3] VEICOLO\n%s\n", sep2, sep2);
fprintf("  L_platoon   = %.1f m   lunghezza rigida equivalente\n", L_platoon);
fprintf("\n");

fprintf("%s\n [4] FERMATE BUS\n%s\n", sep2, sep2);
fprintf("  t_dwell        = %d s\n", t_dwell);
fprintf("  eps_stop_back  = %.1f m\n", eps_stop_back);
fprintf("  eps_stop_front = %.1f m\n", eps_stop_front);
fprintf("  eps_stop_v     = %.2f m/s\n", eps_stop_v);
fprintf("  eps_stop_a     = %.2f m/s^2\n", eps_stop_a);
fprintf("\n");

fprintf("%s\n [5] SEMAFORI\n%s\n", sep2, sep2);
fprintf("  d_safe_TL   = %.1f m\n", d_safe_TL);
fprintf("  s_max       = %d m\n", s_max);
fprintf("\n");

fprintf("%s\n [6] PESI COST FUNCTION  (da Qx1)\n%s\n", sep2, sep2);
fprintf("  w_dist      = %7.2f    avanzamento\n", w_dist);
fprintf("  w_jerk      = %7.2f    jerk (comfort)\n", w_jerk);
fprintf("  w_Ax        = %7.2f    accelerazione longitudinale\n", w_Ax);
fprintf("  w_stop_tgt  = %7.2f    attrazione verso fermata\n", w_stop_tgt);
fprintf("  w_dwell_pos = %7.2f    posizione durante sosta\n", w_dwell_pos);
fprintf("  w_dwell_vel = %7.2f    velocita durante sosta\n", w_dwell_vel);
fprintf("  w_dwell_acc = %7.2f    acc. durante sosta\n", w_dwell_acc);
fprintf("  [terminal]  w_dist_N = %.2f\n", w_dist_N);
fprintf("\n");

fprintf("%s\n [7] SOLVER ACADO\n%s\n", sep2, sep2);
fprintf("  Levenberg-Marquardt = %.0e\n", Lev_Marq);
fprintf("  Integratore         = RK4  (MULTIPLE_SHOOTING)\n");
fprintf("  N_integrator_steps  = %d\n", N);
fprintf("  QP solver           = qpOASES3  (FULL_CONDENSING_N2)\n");
fprintf("  Appross. Hessiano   = GAUSS_NEWTON\n");
fprintf("\n%s\n\n", sep);

%% ========================================================
%%  FIGURA 1 – Griglia predittiva (uniforme)
%% ========================================================
figure("Name","Fig1 - Griglia predittiva","NumberTitle","off","Position",[30 490 660 240]);
bar(1:N, ones(N,1)*Ts, 1, "FaceColor", cB, "EdgeColor", "none");
hold on;
yline(Ts, "--", "Color", cR, "LineWidth", 1.4);
text(N/2, Ts+0.04, sprintf("Ts = %.1f s  (uniforme)", Ts), ...
     "Color", cR, "FontSize", 9, "HorizontalAlignment", "center");
xlabel("Indice intervallo k  [-]"); ylabel("\Delta t_k  [s]");
title(sprintf("Griglia predittiva uniforme    N=%d,  T_{hor}=%.0f s,  Ts=%.1f s", N, T_hor, Ts), "FontSize", 10);
grid on; box on; xlim([0.5 N+0.5]); ylim([0 Ts*1.5]);

%% ========================================================
%%  FIGURA 2 – Vincoli cinematici
%% ========================================================
figure("Name","Fig2 - Vincoli cinematici","NumberTitle","off","Position",[700 490 660 260]);
lbls = ["Ax_{min}","Ax_{max}","Ay_{max}","j_{min}","j_{max}","V_{max}","v_{eps}"];
vals = [Ax_min, Ax_max, Ay_max, jerk_min, jerk_max, V_max, v_eps];
unts = ["m/s^2","m/s^2","m/s^2","m/s^3","m/s^3","m/s","m/s"];
clrs = [cR;cR;cA;cP;cP;cB;cK];
bh = bar(abs(vals), 0.65, "FaceColor", "flat"); bh.CData = clrs;
axv = gca; axv.XTick = 1:length(lbls); axv.XTickLabel = lbls;
ylabel("|valore|"); title("Vincoli cinematici","FontSize",10); grid on; box on;
for i = 1:length(vals)
    text(i, abs(vals(i))+0.04, sprintf("%.3g\n%s", vals(i), unts{i}), ...
         "HorizontalAlignment","center","FontSize",7.5);
end

%% ========================================================
%%  FIGURA 3 – Pesi cost function
%% ========================================================
figure("Name","Fig3 - Pesi cost function","NumberTitle","off","Position",[30 170 660 270]);
lww = ["dist","jerk","Ax","stop tgt","dwell pos","dwell vel","dwell acc","dist(N)"];
wws = [w_dist, w_jerk, w_Ax, w_stop_tgt, w_dwell_pos, w_dwell_vel, w_dwell_acc, w_dist_N];
clw = [cB;cP;cA;cG;cR;cR;cR;cB];
bw = bar(wws, 0.7, "FaceColor", "flat"); bw.CData = clw;
axw = gca; axw.XTick = 1:length(lww); axw.XTickLabel = lww; axw.XTickLabelRotation = 25;
ylabel("Peso Q  [-]");
title("Pesi cost function  (da Qx1 e QN1 del workspace)","FontSize",10);
grid on; box on;
for i = 1:length(wws)
    text(i, wws(i)+0.4, num2str(wws(i)), "HorizontalAlignment","center","FontSize",8.5);
end

fprintf(">> Script completato: output in Command Window + 3 figure.\n");

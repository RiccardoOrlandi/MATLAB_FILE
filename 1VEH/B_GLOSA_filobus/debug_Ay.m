%% ============================================================
% DEBUG AY SLACKS u3/u8
% ============================================================
 
s_TL = [45.5, 203.2, 384.2, 589.4, 773.6, ...
        1004.2, 1225.3, 1419.7, 1507.8, 1739.0, 1823.0];
 
t = out.states.time(:);
x = out.states.signals.values;
 
s1 = x(:,1);
v1 = x(:,2);
 
%% Ricostruzione curvatura da mappa
map_road = load('PL_jrl_chs.mat');
 
k1 = zeros(length(t),1);
 
for kk = 1:length(t)
    [~,~,~,C_c1,~,~] = Map_loc(map_road.map_road, s1(kk));
 
    k1(kk) = abs(C_c1);
 
end
 
%% Accelerazione laterale corretta
Ay1_rebuilt = v1.^2 .* k1;
 
%% Controlli input
t_u = out.u.time(:);
 
u = squeeze(out.u.signals.values);
 
u = u(:);
 
%% Se hai ancora il vettore completo u con 10 ingressi
if isfield(out,'u')
    U = squeeze(out.u.signals.values);
 
    if size(U,1) ~= length(out.u.time)
        U = U';
    end
 
    t_U = out.u.time(:);
 
    slack_Ay1 = U(:,3);
else
    t_U = t_u;
    slack_Ay1 = [];
end
 
%% Plot debug
figure;
subplot(4,1,1)
plot(t, v1.*3.6, 'LineWidth', 1.5); hold on
grid on
ylabel('v [km/h]')
legend('v_1')
title('Velocities')
 
subplot(4,1,2)
plot(t, k1, 'LineWidth', 1.5); hold on
grid on
ylabel('k road [1/m]')
legend('k_1')
title('Road curvature from map')
 
subplot(4,1,3)
plot(t, Ay1_rebuilt, 'LineWidth', 1.5); hold on
grid on
ylabel('Ay [m/s^2]')
legend('Ay_1')
title('Rebuilt lateral acceleration: Ay = v^2 k')
 
subplot(4,1,4)
if ~isempty(slack_Ay1)
    plot(t_U, slack_Ay1, 'LineWidth', 1.5); hold on
    legend('u_3 slack Ay1')
else
    plot(t_u, u1, 'LineWidth', 1.5); hold on
    legend('u_1')
end
grid on
xlabel('Time [s]')
ylabel('slack / input')
title('Control inputs / Ay slacks')
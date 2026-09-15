function h = plot_covDist(out,i_fig,s_TL,s_stop)
% plot the covered distance by the vehicle along with the Traffic Light
% status

figure(i_fig)
hold on
for kk = 1:length(out.x_tl_hor.time)
    for ii = 1:length(s_TL)
        if out.x_tl_hor.signals.values(1,ii,kk) == 1
            h = plot(out.x_tl_hor.time(kk),s_TL(ii),'.g');
            h.Annotation.LegendInformation.IconDisplayStyle = 'off';
        else
            h = plot(out.x_tl_hor.time(kk),s_TL(ii),'.r');
            h.Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    end
end
for jj = 1:length(s_stop)
    h = plot([out.states.time(1) out.states.time(end)], [s_stop(jj) s_stop(jj)],'--k');
    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
end
plot(out.states.time,out.states.signals.values(:,1),'LineWidth',2)
grid on
xlabel('Time [s]')
ylabel('Covered distance [m]')


figure(2*i_fig)
subplot 311
g = plot([0 out.states.time(end)],[15 15],'--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';
hold on
g = plot([0 out.states.time(end)],[50 50],'--k');
g.Annotation.LegendInformation.IconDisplayStyle = 'off';
plot(out.states.time,out.states.signals.values(:,2).*3.6,'LineWidth',2)
grid on
xlabel('Time [s]')
ylabel('V [km/h]')
ylim([0 55])

subplot 312
plot(out.states.time,out.states.signals.values(:,3),'LineWidth',2)
hold on
grid on
xlabel('Time [s]')
ylabel('A_x [m/s^2]')

subplot 313
plot(out.u.time,out.u.signals.values,'LineWidth',2)
hold on
grid on
xlabel('Time [s]')
ylabel('u [m/s^3]')




end
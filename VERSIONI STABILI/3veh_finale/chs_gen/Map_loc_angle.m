%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                     %
%           ---  Map Localization   ---               v 0.1           %
%   - localization for mapping -                     11/12/2019       %
%                                                (Stefano Arrigoni)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [X,Y,Xs,Ys,m_pos] = Map_loc_angle(map_road,s)


%%% check consistency

%%% riazzero su circuito chiuso
if s >= map_road.s(end)
    s = s - map_road.s(end);
%     disp('s sup')
%     disp(s)
end
while s>map_road.s(end)
    s = s - map_road.s(end);
    disp('grosso problema 2!!!!')
    disp(s)
end

%%% circuito chiuso negativi
if s<0
    s = map_road.s(end) + s;
%     disp('s negativa')
end
while s<0
    s = map_road.s(end) + s;
    disp('grosso problema !!!!')
    disp(s)
end


%%% map ego-localization


i = 1;
%%% identifico indice sx più vicino

while(s >= map_road.s(i))

    i = i+1;
end

i=i-1;


%%% parameter calculation

%%% X Y teta_c C_c 
[X,Y,Xs,Ys] = CHS_fn_angle([map_road.X(i) map_road.Y(i)], [map_road.X(i+1) map_road.Y(i+1)],...
    [map_road.Xp(i) map_road.Yp(i)], [map_road.Xp(i+1) map_road.Yp(i+1)],...
    map_road.s(i), map_road.s(i+1), s);



g_c=0;
m_pos =0;
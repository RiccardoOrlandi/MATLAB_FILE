%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                     %
%           ---  CHS M  ---                           v 0.1           %
%   - Cubic Hermite Spline complessiva -             15/02/2017       %
%                                                (Stefano Arrigoni)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [X,Y,teta_c,C_c] = CHS_fn(p0, p1, v0, v1, s0, s1, s)

% if min(s)<s0 || max(s)>s1
%     errordlg('wrong input u!');
% end


uu = (s - s0) / (s1-s0);                                 % normalize u into [0, 1]


X = (2*uu^3 - 3*uu^2 + 1)* p0(1) + (-2*uu^3 + 3*uu^2)* p1(1) +...
    (uu^3 - 2*uu^2 + uu)* v0(1) + (uu^3 - uu^2)* v1(1);
Y = (2*uu^3 - 3*uu^2 + 1)* p0(2) + (-2*uu^3 + 3*uu^2)* p1(2) +...
    (uu^3 - 2*uu^2 + uu)* v0(2) + (uu^3 - uu^2)* v1(2);


Xs = (6*uu^2 - 6*uu)* p0(1) + (-6*uu^2 + 6*uu)* p1(1) +...
    (3*uu^2 - 4*uu + 1)* v0(1) + (3*uu^2 - 2*uu)* v1(1);
Ys = (6*uu^2 - 6*uu)* p0(2) + (-6*uu^2 + 6*uu)* p1(2) +...
    (3*uu^2 - 4*uu + 1)* v0(2) + (3*uu^2 - 2*uu)* v1(2);


Xss = (12*uu - 6)* p0(1) + (-12*uu + 6)* p1(1) +...
    (6*uu - 4)* v0(1) + (6*uu - 2)* v1(1);
Yss = (12*uu - 6)* p0(2) + (-12*uu + 6)* p1(2) +...
    (6*uu - 4)* v0(2) + (6*uu - 2)* v1(2);


teta_c = atan2(Ys,Xs);
if teta_c < 0
    teta_c = teta_c + 2*pi;
end


C_c = (Xs*Yss - Ys*Xss)/(Xs^2 + Ys^2)^(3/2);
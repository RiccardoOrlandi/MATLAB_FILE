%------------------------------------------------
%  Cubic Hermite Spline
%
%  Input:     p0, p1 = [x, y, z, ...]'
%                v0 = dp0/du
%  Output:  p = f(u)
%------------------------------------------------
function p = CHS(p0, v0, p1, v1, us, ue, u)
if min(u)<us || max(u)>ue
    errordlg('wrong input u!');
end

A = [2  -2  1  1
        -3  3 -2 -1
        0   0   1 0
        1   0   0 0];
uu = (u - us) / (ue-us);                                 % normalize u into [0, 1]
n = length(u);
for i = 1 : n
    p_temp = [uu(i)^3, uu(i)^2, uu(i), 1] * A * [p0'; p1'; v0'; v1'];
    p(: , i) = p_temp';
end


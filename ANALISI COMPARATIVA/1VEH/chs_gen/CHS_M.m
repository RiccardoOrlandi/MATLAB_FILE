%------------------------------------------------
%  Multi-segment CHS
%
%  Input:     p = [p0, p1, ...]
%                v = [v0, v1,...]
%                knot = [u0, u1, ...]
%  Output:  c = f(u)
%------------------------------------------------
function c = CHS_M(p, v, knot, u)
if (length(p)~=length(v)) || (length(v)~=length(knot)) || (length(p)~=length(knot))
    errordlg('wrong input data!');
end

n = length(knot);
if min(u)<knot(1) || max(u)>knot(n)
    errordlg('wrong input u!');
end
    
c = [];
for i = 1 : length(u)
    k = fix(sum( sign(u(i) - knot)+1) / 2);               % u(i) falls in the kth segment of knots
    k = (k==0)*1 + (k>0)*k;                                  % in case that u(i) is at one knot
    p0 = p(:,k); p1 = p(:,k+1);
    v0 = v(:,k); v1 = v(:,k+1);
    c_i = CHS(p0, v0, p1, v1, knot(k), knot(k+1), u(i));
    c = [c, c_i];
end
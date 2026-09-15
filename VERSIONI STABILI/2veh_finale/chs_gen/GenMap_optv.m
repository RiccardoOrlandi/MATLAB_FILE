%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                     %
%        -- GenMap_optv --                            v 0.1           %
%   - map simplify and trasform CHS -                13/02/2017       %
%                                                (Stefano Arrigoni)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function [v_r, res] = GenMap_optv(id_rs, u_rs, xy_rs, u_raw, xy_raw)
n_r = size(id_rs,1);
% n_raw = size(u_raw,1);
A = zeros(2*n_r, 2*n_r);
B = zeros(2*n_r, 1);


for i=1:n_r-1    id_j = id_rs(i):id_rs(i+1); % intervallo resampled
    u_j = (u_raw(id_j)-u_rs(i))/(u_rs(i+1)-u_rs(i)); % u rinormalizzate
    xy_j = xy_raw(id_j,:); % misure
    %%% coeff secondo CHS
    a = 2*u_j.^3-3*u_j.^2+1;
    b = -2*u_j.^3+3*u_j.^2;
    c = u_j.^3-2*u_j.^2 + u_j;
    d = u_j.^3-u_j.^2;
    %% matrici Ax=b (del problema min quad)
    %%%x
    A(2*(i-1)+1,2*(i-1)+1) = A(2*(i-1)+1,2*(i-1)+1)+sum(c.*c); %[1,1]
    A(2*(i-1)+3,2*(i-1)+3) = A(2*(i-1)+3,2*(i-1)+3)+sum(d.*d); %[3,3]
    A(2*(i-1)+1,2*(i-1)+3) = A(2*(i-1)+1,2*(i-1)+3)+sum(c.*d); %[1,3]
    A(2*(i-1)+3,2*(i-1)+1) = A(2*(i-1)+1,2*(i-1)+3); %[3,1]
    %%%y
    A(2*(i-1)+2,2*(i-1)+2) = A(2*(i-1)+2,2*(i-1)+2)+sum(c.*c); %[2,2]
    A(2*(i-1)+4,2*(i-1)+4) = A(2*(i-1)+4,2*(i-1)+4)+sum(d.*d); %[4,4]
    A(2*(i-1)+2,2*(i-1)+4) = A(2*(i-1)+2,2*(i-1)+4)+sum(c.*d); %[2,4]
    A(2*(i-1)+4,2*(i-1)+2) = A(2*(i-1)+2,2*(i-1)+4); %[4,2]
    
    %%%x
    B(2*(i-1)+1,1) = B(2*(i-1)+1,1) + sum((xy_j(:,1) - a*xy_rs(i,1) - b*xy_rs(i+1,1)).*c);
    B(2*(i-1)+2,1) = B(2*(i-1)+2,1) + sum((xy_j(:,2) - a*xy_rs(i,2) - b*xy_rs(i+1,2)).*c);
    B(2*(i-1)+3,1) = B(2*(i-1)+3,1) + sum((xy_j(:,1) - a*xy_rs(i,1) - b*xy_rs(i+1,1)).*d);
    B(2*(i-1)+4,1) = B(2*(i-1)+4,1) + sum((xy_j(:,2) - a*xy_rs(i,2) - b*xy_rs(i+1,2)).*d);
    
end


v = A\B;


v_r = [v(1:2:2*n_r)';  v(2:2:2*n_r)'];
c = CHS_M(xy_rs', v_r, u_rs', u_raw');
d = c - xy_raw';
res = sum(d(1,:).^2 + d(2,:).^2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                     %
%        -- GenMap_rsample --                         v 0.2           %
%   - map simplify and trasform CHS -                13/11/2019       %
%     NB da verificare per i dati non ideali!!   (Stefano Arrigoni)   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [out_id, u, xy, s, s_raw] = GenMap_rsample(xy_raw, step)

n = size(xy_raw, 1);

dx = diff(xy_raw(:,1));
dy = diff(xy_raw(:,2));
s_raw = [0; cumsum(sqrt(dx.^2 + dy.^2))];      % total length from p1 to pi

i_nmax = round(s_raw(n)/step) ; % se aggiungo l'ultimo altrimenti floor

% out_id = zeros(i_nmax,1);

s_temp=0;
i_r=1;
i_n=1;
out_id(i_n) = i_r;

while s_raw(out_id(i_n))+step <= s_raw(end)
    while s_raw(i_r)< s_raw(out_id(i_n))+step
        i_r = i_r + 1;
    end
    i_n = i_n +1;
    out_id(i_n) = i_r;    
end

% last point (if not considered yet)

if rem(s_raw(n),step) ~= 0
    i_n = i_n +1;
    out_id(i_n) = n;
end
out_id = out_id';
%%% e circuito chiuso?
s=s_raw(out_id);
u = s/s_raw(out_id(i_n));
xy = xy_raw(out_id,:);


function [timepoints, dt_grid, beta_cheb] = NMPC_chebyshev_grid(N,T_hor,Ts_ctrl)
%NMPC_CHEBYSHEV_GRID Griglia uniforme/Chebyshev-Lobatto attenuata.
%
%   [timepoints,dt_grid,beta_cheb] = NMPC_chebyshev_grid(N,T_hor,Ts_ctrl)
%
% Restituisce N+1 nodi su [0,T_hor]. Il coefficiente beta_cheb viene
% scelto automaticamente affinche' il primo e l'ultimo intervallo siano
% pari a Ts_ctrl. Gli intervalli crescono verso il centro dell'orizzonte
% e poi decrescono simmetricamente.

if nargin < 1
    N = 35;
end
if nargin < 2
    T_hor = 50.0;
end
if nargin < 3
    Ts_ctrl = 1.0;
end

if N < 2 || fix(N) ~= N
    error('N deve essere un intero >= 2.');
end
if T_hor <= 0 || Ts_ctrl <= 0
    error('T_hor e Ts_ctrl devono essere positivi.');
end

k = 0:N;
tau_uniform = k/N;
tau_cheb = 0.5*(1-cos(pi*k/N));

dtau_uniform = 1/N;
dtau_cheb_1 = tau_cheb(2)-tau_cheb(1);

beta_cheb = (Ts_ctrl/T_hor-dtau_uniform) / ...
            (dtau_cheb_1-dtau_uniform);

if beta_cheb < 0 || beta_cheb > 1
    error(['N, T_hor e Ts_ctrl non consentono un blending compreso ' ...
           'tra griglia uniforme e Chebyshev-Lobatto.']);
end

tau_grid = (1-beta_cheb)*tau_uniform + beta_cheb*tau_cheb;
timepoints = T_hor*tau_grid;
timepoints(1) = 0.0;
timepoints(end) = T_hor;
dt_grid = diff(timepoints);

if any(dt_grid <= 0)
    error('La griglia non e'' strettamente crescente.');
end
end

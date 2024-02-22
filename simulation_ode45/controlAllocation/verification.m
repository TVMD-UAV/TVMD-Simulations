addpath('system_func')

%% Verification for full allocation
% inputs
eta = ones(10, 1) * -0.1;
xi = ones(10, 1) * 0.1;
F = ones(10, 1);

% output control
vec = full_dof_mixing(P, psi, eta, xi, F)

% inverse
n = 10;
Fi = [(sin(xi) .* F)'
    (-sin(eta) .* cos(xi) .* F)'
    (cos(eta) .* cos(xi) .* F)']
Fi = reshape(Fi, [3 * n 1]);
[F, a, b] = inverse_input(Fi)

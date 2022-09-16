%% ===================================
%
% Generate full DOF commands from agents
% P: configuration coordinates in body frame
% psi: orientations of agents
% eta: y-axis angles for each agent (expected to be a column vector)
% xi: x-axis angles for each agent (expected to be a column vector)
% F: thrust magnitude for each agent (expected to be a column vector)
%
%% ===================================
function vec = full_dof_mixing(P, psi, xi, eta, F)
    n = length(psi);
    R = zeros([n, 3, 3]);

    for i = 1:n
        R(i, :, :) = Rx(eta(i)) * Ry(xi(i));
    end

    M = get_M(n, psi, P);

    Fi = [(sin(xi) .* F) (-sin(eta) .* cos(xi) .* F) (cos(eta) .* cos(xi) .* F)];
    Fi = reshape(Fi', [3 * n 1]);
    vec = M * Fi;
end

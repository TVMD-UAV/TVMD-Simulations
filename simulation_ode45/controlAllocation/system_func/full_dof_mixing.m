%% ===================================
%
% Generate full DOF commands from agents
% P: configuration coordinates in body frame
% psi: orientations of agents
% a: x-axis angles for each agent (expected to be a column vector)
% b: y-axis angles for each agent (expected to be a column vector)
% tf: thrust magnitude for each agent (expected to be a column vector)
%
%% ===================================
function vec = full_dof_mixing(P, psi, a, b, tf)
    n = length(psi);
    M = get_M(n, psi, P);
    fi = get_f(a, b, tf);
    vec = M * fi;
end

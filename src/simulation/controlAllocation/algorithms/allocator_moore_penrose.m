function [a, b, F, vecs] = allocator_moore_penrose(u, conf)
    % Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    n = length(psi);
    M = get_M(n, psi, P);

    Fi = pseudo_inverse(M) * u;
    [F, a, b] = inverse_input(Fi);

    vecs = full_dof_mixing(P, psi, a, b, F);
end

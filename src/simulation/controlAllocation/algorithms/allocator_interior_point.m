function [eta, xi, R, F, vecs] = allocator_interior_point(u, W, conf)
    % Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    n = length(psi);
    M = get_M(n, psi, P);

    x0 = zeros(n * 3, 1);

    for i = 1:n
        x0(i * 3) = 10;
    end

    % x = fmincon(fun,    x0,  A,  b,Aeq,beq, lb, ub, nonlcon)
    options = optimoptions(@fmincon, 'Algorithm', 'interior-point', 'Display', 'off');
    Fi = fmincon(@(x)costfun(x, W), x0, [], [], M, u, [], [], @(x)mycon(x, f_max, sigma_a, sigma_b), options);
    [F, a, b] = inverse_input(Fi);

    eta = a;
    xi = b;

    R = zeros([n, 3, 3]);

    for i = 1:n
        R(i, :, :) = Rx(eta(i)) * Ry(xi(i));
    end

    vecs = full_dof_mixing(P, psi, eta, xi, F);
end

function y = costfun(x, W)
    n = length(x') / 3;
    Fi = reshape(x, [3, n]);
    F = sqrt(sum(Fi.^2, 1))';
    y = F' * W * F;
end

% constrain on angles
function [c, ceq] = mycon(x, f_max, sigma_a, sigma_b)
    [F, a, b] = inverse_input(x);
    c = [-F;
        F - f_max;
        -sigma_a - a;
        a - sigma_a;
        -sigma_b - b;
        b - sigma_b];
    ceq = [];
end

% constrain on vector
function [c, ceq] = mycon2(x, f_max, sigma_a, sigma_b)
    %[F, a, b] = inverse_input(x);
    Fi = x;
    n = length(Fi') / 3;
    Fi = reshape(Fi, [3, n]);
    F = sqrt(sum(Fi.^2, 1))';

    ad = -Fi(2, :) / Fi(3, :);
    bd = Fi(1, :) / F';
    c = [-F;
        F - f_max;
        tan(-sigma_a) - ad;
        ad - tan(sigma_a);
        sin(-sigma_b) - bd;
        bd - sin(sigma_b)];
    ceq = [];
end

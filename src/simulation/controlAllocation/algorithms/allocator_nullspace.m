function [a, b, F, x, u] = allocator_nullspace(u_d, conf, f0, a0, b0, dt)
    %% Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    r_sigma_a = conf('r_sigma_a');
    r_sigma_b = conf('r_sigma_b');
    f_max = conf('f_max');
    r_f = conf('r_f');

    n = length(psi);
    W = get_M(n, psi, P);

    persistent F_partial

    if length(F_partial) <= 0
        [F_partial] = allocator_nullspace_cal_jacobian(conf);
    end

    F0 = get_f(a0, b0, f0);

    % Nullspace of W
    Nw = null(W);

    % Equality constraints
    A_Dx = F_partial(a0, b0, f0);
    Aeq = [A_Dx eye(3 * n) -Nw];
    beq = pseudo_inverse(W) * u_d - F0;

    % Cost function weights
    Pw = ones([3 * n 1]);
    Rw = 1000 * ones([3 * n 1]);
    Qw = 100 * ones([3 * n - 6 1]);
    H = diag([Pw; Rw; Qw]);

    % Bounds and inequality constraints
    x0 = [f0; a0; b0];
    x_min = -reshape((ones([1 n]) .* [0; sigma_a; sigma_b])', [3 * n 1]);
    x_max = reshape((ones([1 n]) .* [f_max; sigma_a; sigma_b])', [3 * n 1]);
    r_x_min = -dt * reshape((ones([1 n]) .* [r_f; r_sigma_a; r_sigma_b])', [3 * n 1]);
    r_x_max = dt * reshape((ones([1 n]) .* [r_f; r_sigma_a; r_sigma_b])', [3 * n 1]);

    % Combining the bounds
    dx_lb = max(x_min - x0, r_x_min);
    dx_ub = min(x_max - x0, r_x_max);
    dx_lb = r_x_min;
    dx_ub = r_x_max;
    lb = [dx_lb; -2 * ones([3 * n 1]); -10 * ones([3 * n - 6 1])];
    ub = [dx_ub; 2 * ones([3 * n 1]); 10 * ones([3 * n - 6 1])];

    % QP Solver
    options = optimoptions(@quadprog, 'Algorithm', 'interior-point-convex', 'Display', 'off');
    x = quadprog(H, [], [], [], Aeq, beq, lb, ub, [], options);

    df = x(1:n);
    da = x(n + 1:2 * n);
    db = x(2 * n + 1:3 * n);
    s = x(3 * n + 1:6 * n);
    z = x(6 * n + 1:end);

    % Solutions given by paper
    Fss = get_f(a0 + da, b0 + db, f0 + df);
    zs = pseudo_inverse_tall(Nw) * (Fss - pseudo_inverse(W) * u_d);
    Fs = pseudo_inverse(W) * u_d + Nw * zs;
    [F, a, b] = inverse_input(Fs);

    w = min(x_max, max(x_min, [F; a; b]));
    F = w(1:10);
    a = w(11:20);
    b = w(21:30);

    % fprintf("RMSE equality error: %.4x\n", sqrt(mean((beq - Aeq * x).^2)))
    % fprintf("mean slack error: %.4x\n", mean(s))
    % fprintf("cost: %.4x\n", x' * H * x)
    u = full_dof_mixing(P, psi, a, b, F);
end

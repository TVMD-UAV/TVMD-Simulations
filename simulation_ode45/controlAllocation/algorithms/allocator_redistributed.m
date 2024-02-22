function [a, b, F] = allocator_redistributed(t_d, conf, t0, W)
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
    M = get_M(n, psi, P);
    pinv_tol = 0.0001;

    % Initialization
    N_esp = ones([n 1]);
    M_esp = M;
    M_esp_dag = pinv(M, pinv_tol);
    c = 0;

    % Assuming u0 is feasible
    u0 = M_esp_dag * t0;
    %tf0 = get_tf_from_u(u0, n);
    t_delta = t_d - t0;

    % Iteration
    while (c < 1) && (rank(M_esp) >= 6)
        % Calculate
        M_esp = M_esp * kron(diag(N_esp), eye(3)); % masking
        M_esp_dag = pinv(M_esp, pinv_tol);
        u_delta = M_esp_dag * t_delta;

        % --Determine direction
        % Transform to spherical coordinate
        [rho0, theta0, phi0] = to_spherical(u0);
        disp('u')
        u0'
        u_delta'
        [rho_delta, theta_delta, phi_delta] = to_spherical(u_delta);

        % Limiting on the smallest violation angle
        theta_bound = (theta0 + theta_delta) - sigma_a;
        idx = (sigma_a >= theta0 + theta_delta);
        theta_bound(idx) = inf;
        theta_bound(N_esp == 0) = inf;

        [theta_max, i_star] = min(theta_bound);

        theta0
        theta_delta
        theta_bound
        theta_max

        if theta_max ~= inf
            fprintf("Direction modification")
            theta_delta(i_star) = sigma_a - theta0(i_star);
            % Transform back to cartesian coordinate
            u = to_cartesian(rho0 + rho_delta, theta0 + theta_delta, phi0 + phi_delta);
            u_delta = u - u0;
        end

        % Update marked set
        N_esp(i_star) = 0;

        % --Determine magnitude
        M_sigma = zeros([6, n]);

        for i = 1:n
            M_sigma(:, i) = M_esp(:, 3 * i - 2:3 * i) * u_delta(3 * i - 2:3 * i);
        end

        s_delta = pinv(M_sigma, pinv_tol) * t_delta;
        tf0 = get_tf_from_u(u0, n);
        s_bound = (f_max - tf0) ./ s_delta;
        s_l_bound = (tf0 ./ -s_delta);
        s_bound(s_delta < 0) = s_l_bound(s_delta < 0);
        s_bound(s_delta == 0) = inf;
        s_bound(s_bound <= 0) = inf;
        s_bound
        [d_max, i_sigma] = min(s_bound);

        % Marked and reach bound
        i_star
        i_sigma

        if d_max ~= inf %&& N_esp(i_sigma) == 0
            d = d_max;

            if d_max > 1 - c
                d = 1 - c;
            end

            u0 = u0 + d * u_delta;
            c = c + d;
            c
            d
        end

    end

    [F, a, b] = inverse_input(u0);
end

function tf = get_tf_from_u(u, n)
    % u: column vector
    tf = vecnorm(reshape(u, [3 n]), 2, 1)';
end

function [rho, theta, phi] = to_spherical(u)
    n = length(u) / 3;
    rho = get_tf_from_u(u, n);
    fi = reshape(u, [3 n])';
    phi = atan2(fi(:, 2), fi(:, 1));
    theta = atan2(fi(:, 1), fi(:, 3) .* cos(phi));
end

function u = to_cartesian(rho, theta, phi)
    n = length(rho);
    fx = rho .* cos(phi) .* tan(theta);
    fy = rho .* sin(phi) .* sin(theta);
    fz = rho .* cos(theta);
    u = reshape([fx fy fz]', [3 * n, 1]);
end

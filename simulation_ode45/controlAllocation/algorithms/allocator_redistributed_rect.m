function [a, b, F] = allocator_redistributed_rect(t_d, conf, t0, W, dt)
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
    % u0 = M_esp_dag * t0;
    u0 = ones([1 n]) .* [0; 0; 1];
    u0 = reshape(u0, [3 * n 1]);
    %tf0 = get_tf_from_u(u0, n);
    t_delta = t_d - t0;
    moore_delta = M_esp_dag * t_delta;

    % Calculating boundaries
    u_xyz = reshape(u0, [3 n]);
    % a0 = (atan2(-u_xyz(2, :), u_xyz(3, :)))';
    % b0 = (atan2(u_xyz(1, :), u_xyz(3, :) ./ cos(a0)'))';
    [F0, a0, b0] = inverse_input(u0);

    dap = min(r_sigma_a * dt, sigma_a - a0);
    dan = max(-r_sigma_a * dt, -sigma_a - a0);
    dbp = min(r_sigma_b * dt, sigma_b - b0);
    dbn = max(-r_sigma_b * dt, -sigma_b - b0);

    % region [moore allocation]
    % moore allocation saturation
    % u_moore = u_xyz + moore_xyz;
    % a_m = (atan2(-u_moore(2, :), u_moore(3, :)))';
    % b_m = (atan2(u_moore(1, :), u_moore(3, :) ./ cos(a_m)'))';
    % tf_m = get_tf_from_u(u_moore, n);

    [tf_m, a_m, b_m] = inverse_input(u0 + moore_delta);
    a_m_max = min(sigma_a, a_m + r_sigma_a * dt);
    a_m_min = max(-sigma_a, a_m - r_sigma_a * dt);
    b_m_max = min(sigma_b, b_m + r_sigma_b * dt);
    b_m_min = max(-sigma_b, b_m - r_sigma_b * dt);
    tf_m_max = min(f_max, tf_m + r_f * dt);
    tf_m_min = max(0, tf_m - r_f * dt);
    a_m = min(a_m_max, max(a_m_min, a_m));
    b_m = min(b_m_max, max(b_m_min, b_m));
    tf_m = min(tf_m_max, max(tf_m_min, tf_m));
    u_moore = get_f(a_m, b_m, tf_m);
    moore_delta = u_moore - u0;
    moore_xyz = reshape(moore_delta, [3 n]);
    vecs = full_dof_mixing(P, psi, a_m, b_m, tf_m);
    disp('moore')
    vecs
    % end region [moore allocation]

    % region [boundary visualization]
    figure('Position', [10 10 1600 800])
    aa = reshape([a0 + dap a0 - dap a0 - dap a0 + dap]', [4 * n 1]);
    bb = reshape([b0 - dbp b0 - dbp b0 + dbp b0 + dbp]', [4 * n 1]);
    fs = reshape(get_f(aa, bb, f_max), [3, 4, n]);

    for i = 1:n
        subplot(2, n / 2, i);
        quiver3(0, 0, 0, u_xyz(1, i), u_xyz(2, i), u_xyz(3, i), 'Color', '#000000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
        quiver3(u_xyz(1, i), u_xyz(2, i), u_xyz(3, i), ...
            moore_xyz(1, i), moore_xyz(2, i), moore_xyz(3, i), 'Color', '#0000AA', 'LineWidth', 1, 'AutoScale', 'off'); hold on

        % roof
        funcx = @(a, b)f_max .* sin(b);
        funcy = @(a, b) - f_max .* sin(a) .* cos(b);
        funcz = @(a, b)f_max .* cos(a) .* cos(b);
        fsurf(funcx, funcy, funcz, [a0(i) + dan(i) a0(i) + dap(i) b0(i) + dbn(i) b0(i) + dbp(i)], 'FaceColor', '#77AC30', 'FaceAlpha', 0.2, 'EdgeColor', 'none')

        % positive x
        funcx = @(r, b)r .* tan(sigma_b);
        funcy = @(r, b)r .* sin(b);
        funcz = @(r, b)r .* cos(b);
        fsurf(funcx, funcy, funcz, [0 f_max * 0.85 -sigma_a sigma_a], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none')

        % negative x
        funcx = @(r, b) - r .* tan(sigma_b);
        funcy = @(r, b)r .* sin(b);
        funcz = @(r, b)r .* cos(b);
        fsurf(funcx, funcy, funcz, [0 f_max * 0.85 -sigma_a sigma_a], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none')

        for j = 1:4
            plot3([0 fs(1, j, i)], [0 fs(2, j, i)], [0 fs(3, j, i)], 'Color', '#0072BD'); hold on
            plot3([fs(1, mod(j, 4) + 1, i), fs(1, j, i)], ...
                [fs(2, mod(j, 4) + 1, i), fs(2, j, i)], ...
                [fs(3, mod(j, 4) + 1, i), fs(3, j, i)], 'Color', '#0072BD'); hold on
        end

        title(i);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        axis equal
    end

    % end region [boundary visualization]

    % Iteration
    while (c < 1) && (rank(M_esp) >= 6)
        % Calculate
        N_esp
        M_esp = M_esp * kron(diag(N_esp), eye(3)); % masking
        M_esp_dag = pinv(M_esp, pinv_tol);
        u_delta = M_esp_dag * t_delta;
        v_norm = get_tf_from_u(u_delta, n);
        tf0 = get_tf_from_u(u0, n);
        v_delta = reshape(u_delta, [3, n]);
        v_delta(:, v_norm == 0) = 0; % disabled actuators

        % --Determine direction
        da = dap;
        da(v_delta(2, :) > 0) = dan(v_delta(2, :) > 0);

        db = dbp;
        db(v_delta(1, :) < 0) = dbn(v_delta(1, :) < 0);

        % Finding nearest
        k_b = tan(b0 + db);
        k_a = -tan(a0 + da);

        u_xyz = reshape(u0, [3 n]);

        tx = (u_xyz(3, :)' .* k_b - u_xyz(1, :)') ./ (v_delta(1, :)' - k_b .* v_delta(3, :)');
        ty = (u_xyz(3, :)' .* k_a - u_xyz(2, :)') ./ (v_delta(2, :)' - k_a .* v_delta(3, :)');
        kkb = dot(u_xyz, v_delta, 1)' ./ v_norm.^2;
        kkc = (tf0 .* tf0 - f_max^2) ./ v_norm.^2;
        tz = -kkb + sqrt(kkb .* kkb - kkc);

        tw = [tx ty tz]
        tw(tw < 0) = inf;
        ti = min(tw, [], 2);
        ti(N_esp <= 0) = inf;
        [d_max, i_star] = min(ti);

        % draw boundaries
        disp('ub')
        ub = u_xyz + v_delta .* ([1 1 1]' * ti');
        ub_x = u_xyz + v_delta .* ([1 1 1]' * tx');
        ub_y = u_xyz + v_delta .* ([1 1 1]' * ty');
        ub_z = u_xyz + v_delta .* ([1 1 1]' * tz');

        if d_max == inf
            break;
        end

        ti

        % Update marked set
        N_esp(i_star) = 0;
        d = d_max;

        if d_max > 1 - c
            d = 1 - c;
        end

        d

        for i = 1:n
            subplot(2, n / 2, i);
            quiver3(u0(3 * i - 2), u0(3 * i - 1), u0(3 * i), ...
                d * v_delta(1, i), d * v_delta(2, i), d * v_delta(3, i), 'Color', '#AA0000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
            scatter3(ub(1, i), ub(2, i), ub(3, i), 10, 'black'); hold on
            % scatter3(ub_x(1, i), ub_x(2, i), ub_x(3, i), 10, 'red'); hold on
            % scatter3(ub_y(1, i), ub_y(2, i), ub_y(3, i), 10, 'green'); hold on
            % scatter3(ub_z(1, i), ub_z(2, i), ub_z(3, i), 10, 'blue'); hold on
        end

        u0 = u0 + d * reshape(v_delta, [3 * n 1]);

        c = c + d;
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

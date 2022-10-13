function [a, b, F] = allocator_redistributed_nonlinear(t_d, conf, a0, b0, f0, W, dt)
    draw = false;
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
    u0 = get_f(a0, b0, f0);
    t0 = full_dof_mixing(P, psi, a0, b0, f0);
    t_delta = t_d - t0;
    moore_delta = M_esp_dag * t_delta;

    % Calculating boundaries
    u_xyz = reshape(u0, [3 n]);

    dap = min(r_sigma_a * dt, sigma_a - a0);
    dan = max(-r_sigma_a * dt, -sigma_a - a0);
    dbp = min(r_sigma_b * dt, sigma_b - b0);
    dbn = max(-r_sigma_b * dt, -sigma_b - b0);

    % region [moore allocation]
    % moore allocation saturation
    [tf_m, a_m, b_m] = inverse_input(u0 + moore_delta);
    a_m_max = min(sigma_a, a0 + r_sigma_a * dt);
    a_m_min = max(-sigma_a, a0 - r_sigma_a * dt);
    b_m_max = min(sigma_b, b0 + r_sigma_b * dt);
    b_m_min = max(-sigma_b, b0 - r_sigma_b * dt);
    tf_m_max = min(f_max, f0 + r_f * dt);
    tf_m_min = max(0, f0 - r_f * dt);

    a_m = min(a_m_max, max(a_m_min, a_m));
    b_m = min(b_m_max, max(b_m_min, b_m));
    tf_m = min(tf_m_max, max(tf_m_min, tf_m));

    u_moore = get_f(a_m, b_m, tf_m);
    moore_delta = u_moore - u0;
    moore_xyz = reshape(moore_delta, [3 n]);
    % end region [moore allocation]

    % region [boundary visualization]
    if draw
        figure('Position', [10 10 1600 800])
        aa = reshape([a0 + dap a0 + dan a0 + dan a0 + dap]', [4 * n 1]);
        bb = reshape([b0 + dbn b0 + dbn b0 + dbp b0 + dbp]', [4 * n 1]);
        fs = reshape(get_f(aa, bb, f_max), [3, 4, n]);

        for i = 1:n
            subplot(2, n / 2, i);
            quiver3(0, 0, 0, u_xyz(1, i), u_xyz(2, i), u_xyz(3, i), 'Color', '#000000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
            quiver3(u_xyz(1, i), u_xyz(2, i), u_xyz(3, i), ...
                moore_xyz(1, i), moore_xyz(2, i), moore_xyz(3, i), 'Color', '#0000AA', 'LineWidth', 1, 'AutoScale', 'off'); hold on

            % Plot original moore penrose result
            mdo = M_esp_dag * t_delta + u0;
            mdo = reshape(mdo, [3 n]);
            quiver3(0, 0, 0, ...
                mdo(1, i), mdo(2, i), mdo(3, i), 'Color', '#00AA00', 'LineWidth', 1, 'AutoScale', 'off'); hold on

            % roof
            funcx = @(a, b)f_max .* sin(b);
            funcy = @(a, b) - f_max .* sin(a) .* cos(b);
            funcz = @(a, b)f_max .* cos(a) .* cos(b);
            fsurf(funcx, funcy, funcz, [a0(i) + dan(i) a0(i) + dap(i) b0(i) + dbn(i) b0(i) + dbp(i)], 'FaceColor', '#77AC30', 'FaceAlpha', 0.2, 'EdgeColor', 'none')

            % positive x
            funcx = @(r, b)r .* tan(sigma_b);
            funcy = @(r, b)r .* sin(b);
            funcz = @(r, b)r .* cos(b);
            %fsurf(funcx, funcy, funcz, [0 f_max * 0.45 -sigma_a sigma_a], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none')
            fsurf(funcx, funcy, funcz, [0 f_max * 0.45 a0(i) + dan(i) a0(i) + dap(i)], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none')

            % negative x
            funcx = @(r, b) - r .* tan(sigma_b);
            funcy = @(r, b)r .* sin(b);
            funcz = @(r, b)r .* cos(b);
            fsurf(funcx, funcy, funcz, [0 f_max * 0.45 a0(i) + dan(i) a0(i) + dap(i)], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none')

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

    end

    % end region [boundary visualization]

    % Iteration
    while (c < 1) && (rank(M_esp) >= 6)
        % Calculate
        M_esp = M_esp * kron(diag(N_esp), eye(3)); % masking
        M_esp_dag = pinv(M_esp, pinv_tol);
        u_delta = M_esp_dag * t_delta;
        v_norm = get_tf_from_u(u_delta, n);
        tf0 = get_tf_from_u(u0, n);
        v_delta = reshape(u_delta, [3, n]);
        v_delta(:, v_norm == 0) = 0; % disabled actuators

        % --Determine direction
        [f1, a1, b1] = inverse_input(u0 + u_delta);
        da = dap;
        db = dbp;
        da(a1 - a0 < 0) = dan(a1 - a0 < 0);
        db(b1 - b0 < 0) = dbn(b1 - b0 < 0);

        % Finding nearest
        u_xyz = reshape(u0, [3 n]);
        v_delta_x = v_delta(1, :) ./ (tan(b0 + db)');
        u_xyz_x = u_xyz(1, :) ./ (tan(b0 + db)');
        txa = v_delta(2, :)'.^2 + v_delta(3, :)'.^2 - v_delta_x'.^2;
        txb = u_xyz(2, :)' .* v_delta(2, :)' + u_xyz(3, :)' .* v_delta(3, :)' - u_xyz_x' .* v_delta_x';
        txc = u_xyz(2, :)'.^2 + u_xyz(3, :)'.^2 - u_xyz_x'.^2;
        tx = (-txb - sqrt(txb.^2 - txa .* txc)) ./ txa;
        % tx2 = (-txb - sqrt(txb.^2 - txa .* txc)) ./ txa;
        tx(imag(tx) ~= 0) = inf;

        ty = (-u_xyz(2, :)' - u_xyz(3, :)' .* tan(a0 + da)) ./ (v_delta(2, :)' + v_delta(3, :)' .* tan(a0 + da));

        kkb = dot(u_xyz, v_delta, 1)' ./ v_norm.^2;
        kkc = (tf0 .* tf0 - f_max^2) ./ v_norm.^2;
        tz = -kkb + sqrt(kkb .* kkb - kkc);

        tw = [tx ty tz];
        tw(tw < 0) = inf;
        ti = min(tw, [], 2);
        ti(N_esp <= 0) = inf;
        [d_max, i_star] = min(ti);

        if d_max == inf
            break;
        end

        % Update marked set
        N_esp(i_star) = 0;
        d = d_max;

        if d_max > 1 - c
            d = 1 - c;
        end

        if draw
            % draw boundaries
            ub = u_xyz + v_delta .* ([1 1 1]' * ti');
            ub_x = u_xyz + v_delta .* ([1 1 1]' * tx');
            % ub_x2 = u_xyz + v_delta .* ([1 1 1]' * tx2');
            ub_y = u_xyz + v_delta .* ([1 1 1]' * ty');
            ub_z = u_xyz + v_delta .* ([1 1 1]' * tz');

            for i = 1:n
                subplot(2, n / 2, i);
                quiver3(u0(3 * i - 2), u0(3 * i - 1), u0(3 * i), ...
                    d * v_delta(1, i), d * v_delta(2, i), d * v_delta(3, i), 'Color', '#AA0000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
                scatter3(ub(1, i), ub(2, i), ub(3, i), 10, 'black'); hold on
                % scatter3(ub_x(1, i), ub_x(2, i), ub_x(3, i), 10, 'red'); hold on
                % scatter3(ub_x2(1, i), ub_x2(2, i), ub_x2(3, i), 10, 'red'); hold on
                % scatter3(ub_y(1, i), ub_y(2, i), ub_y(3, i), 10, 'green'); hold on
                % scatter3(ub_z(1, i), ub_z(2, i), ub_z(3, i), 10, 'blue'); hold on
            end

        end

        u0 = u0 + d * u_delta;

        c = c + d;
        % return
    end

    [F, a, b] = inverse_input(u0);
end

function tf = get_tf_from_u(u, n)
    % u: column vector
    tf = vecnorm(reshape(u, [3 n]), 2, 1)';
end

function [a, b, F, u] = allocator_redistributed_nonlinear(t_d, conf, a0, b0, f0, W, dt)
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
    c = 0;

    a_m_max = sigma_a;
    a_m_min = -sigma_a;
    b_m_max = sigma_b;
    b_m_min = -sigma_b;
    tf_m_max = f_max;
    tf_m_min = 0;

    dap = sigma_a - a0;
    dan = -sigma_a - a0;
    dbp = sigma_b - b0;
    dbn = -sigma_b - b0;
   
    % Initial allocation 
    M_dag = pinv(M, pinv_tol);
    u_delta = M_dag * t_d;

    % Checking constraint violations
    [fl0, al0, bl0] = inverse_input(u_delta);
    al0 = min(a_m_max, max(a_m_min, al0));
    bl0 = min(b_m_max, max(b_m_min, bl0));

    % Scaling magnitude
    t_max = max(fl0) / f_max;
    if t_max > 1
        fl0 = fl0 / t_max;
    end

    u0 = get_f(al0, bl0, fl0);
    % t0 = full_dof_mixing(P, psi, al0, bl0, fl0);
    t0 = M * u0;
    t_delta = t_d - t0;

    % region [boundary visualization]
    if draw

        % region [moore allocation]
        % moore allocation saturation
        M_esp_dag = pinv(M, pinv_tol);
        moore_delta = M_esp_dag * t_delta;
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

        figure('Position', [10 10 1600 800])
        aa = reshape([a0 + dap a0 + dan a0 + dan a0 + dap]', [4 * n 1]);
        bb = reshape([b0 + dbn b0 + dbn b0 + dbp b0 + dbp]', [4 * n 1]);
        fs = reshape(get_f(aa, bb, f_max), [3, 4, n]);

        [eta_int, xi_int, R_int, F_int] = allocator_interior_point(t_d, W, conf, a0, b0, f0);
        u_int = get_f(eta_int, xi_int, F_int);
        u_int = reshape(u_int, [3 n]);

        for i = 1:n
            subplot(2, n / 2, i);
            quiver3(0, 0, 0, u_xyz(1, i), u_xyz(2, i), u_xyz(3, i), 'Color', '#000000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
            quiver3(u_xyz(1, i), u_xyz(2, i), u_xyz(3, i), ...
                moore_xyz(1, i), moore_xyz(2, i), moore_xyz(3, i), 'Color', '#0000AA', 'LineWidth', 1, 'AutoScale', 'off'); hold on

            quiver3(0, 0, 0, u_int(1, i), u_int(2, i), u_int(3, i), 'Color', '#00AAAA', 'LineWidth', 1, 'AutoScale', 'off'); hold on

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

    c_idx = 1;
    cmap = jet(8);
    % Iteration
    while (c < 1) && (rank(M_esp) >= 6)
        % Calculate
        M_esp = M * kron(diag(N_esp), eye(3)); % masking
        M_esp_dag = pinv(M_esp, pinv_tol);
        u_delta = M_esp_dag * t_delta;
        tf0 = get_tf_from_u(u0, n);

        [d_max, i_star] = calc_nearest_c2(N_esp, u0, u0+u_delta, ...
            a0, b0, tf0, dan, dap, dbn, dbp, f_max);

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
                    d * v_delta(1, i), d * v_delta(2, i), d * v_delta(3, i), 'Color', cmap(c_idx, :), 'LineWidth', 2, 'AutoScale', 'off'); hold on
                scatter3(ub(1, i), ub(2, i), ub(3, i), 10, 'black'); hold on
                % scatter3(ub_x(1, i), ub_x(2, i), ub_x(3, i), 10, 'red'); hold on
                % scatter3(ub_x2(1, i), ub_x2(2, i), ub_x2(3, i), 10, 'red'); hold on
                % scatter3(ub_y(1, i), ub_y(2, i), ub_y(3, i), 10, 'green'); hold on
                % scatter3(ub_z(1, i), ub_z(2, i), ub_z(3, i), 10, 'blue'); hold on
            end

        end

        u0 = u0 + d * u_delta;
        c = c + d;

        % Checking
        if draw
            uo = M * u0;
            fprintf("d=%.2f, c=%.2f, sat=%#d,\t", d, c, i_star);
            [ef, em, df, dm] = output_error(t_d, uo);
            [f1, a1, b1] = inverse_input(uo);
            tef = thrust_efficiency(a1, b1, f1);

            fprintf("[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f],", uo(1), uo(2), uo(3), uo(4), uo(5), uo(6));
            fprintf(" \ttef=%.4f,\tef=%.4f,\tem=%.4f,\tdf=%.4f,\tdm=%.4f \n", tef, ef, em, df, dm);
        end

        c_idx = c_idx + 1;
        % pause
    end

    [F, a, b] = inverse_input(u0);
    u = full_dof_mixing(P, psi, a, b, F);
end

function [c_star, i_star] = calc_nearest_c(N_esp, u0, ud, a0, b0, tf0, dan, dap, dbn, dbp, f_max)
    % a0, b0, tf0: previous state
    % dan, dbn: negtive bound of rate
    % dap, dbp: positive bound of rate

    % u0: allocation result in previous step
    % ud: temporary target

    % u is 3n x 1 column vector
    % u_xyz is 3 x n matrix
    % v is alias for u / tan
    n = length(N_esp);
    u_xyz = reshape(u0, [3 n]);
    u_delta = ud - u0;
    u_delta_xyz = reshape(u_delta, [3, n]);
    u_delta_xyz(:, ~N_esp) = 0; % disabled actuators
    v_norm = get_tf_from_u(u_delta, n);

    % Determine direction
    [f1, a1, b1] = inverse_input(ud);
    da = dap;
    db = dbp;
    da(a1 - a0 < 0) = dan(a1 - a0 < 0);
    db(b1 - b0 < 0) = dbn(b1 - b0 < 0);

    % Finding nearest
    v_delta_x = u_delta_xyz(1, :) ./ (tan(b0 + db)');
    v_xyz_x = u_xyz(1, :) ./ (tan(b0 + db)');
    txa = u_delta_xyz(2, :)'.^2 + u_delta_xyz(3, :)'.^2 - v_delta_x'.^2;
    txb = u_xyz(2, :)' .* u_delta_xyz(2, :)' + u_xyz(3, :)' .* u_delta_xyz(3, :)' - v_xyz_x' .* v_delta_x';
    txc = u_xyz(2, :)'.^2 + u_xyz(3, :)'.^2 - v_xyz_x'.^2;
    tx = (-txb - sqrt(txb.^2 - txa .* txc)) ./ txa;
    tx(imag(tx) ~= 0) = inf;

    ty = (-u_xyz(2, :)' - u_xyz(3, :)' .* tan(a0 + da)) ./ (u_delta_xyz(2, :)' + u_delta_xyz(3, :)' .* tan(a0 + da));

    kkb = dot(u_xyz, u_delta_xyz, 1)' ./ v_norm.^2;
    kkc = (tf0 .* tf0 - f_max^2) ./ v_norm.^2;
    tz = -kkb + sqrt(kkb .* kkb - kkc);

    tw = [tx ty tz]
    tw(tw < 0) = inf;
    ti = min(tw, [], 2);
    ti(N_esp <= 0) = inf;
    [c_star, i_star] = min(ti);
end


function [c_star, i_star] = calc_nearest_c2(N_esp, u0, ud, a0, b0, tf0, dan, dap, dbn, dbp, f_max)
    % a0, b0, tf0: previous state
    % dan, dbn: negtive bound of rate
    % dap, dbp: positive bound of rate

    % u0: allocation result in previous step
    % ud: temporary target

    % u is 3n x 1 column vector
    % u_xyz is 3 x n matrix
    % v is alias for u / tan
    n = length(N_esp);
    u_xyz = reshape(u0, [3 n]);
    u_delta = ud - u0;
    u_delta_xyz = reshape(u_delta, [3, n]);
    u_delta_xyz(:, ~N_esp) = 0; % disabled actuators

    % Determine direction
    [f1, a1, b1] = inverse_input(ud);
    da = dap;
    db = dbp;
    da(a1 - a0 < 0) = dan(a1 - a0 < 0);
    db(b1 - b0 < 0) = dbn(b1 - b0 < 0);

    % Finding nearest
    tw = solve_intersections(u_xyz, u_delta_xyz, a0, b0, tf0, da, db, f_max);

    % Checking direction of delta
    % is_at_origin = (ones([1 3]) * (u_xyz == zeros([3 n])) == 3);
    is_at_origin = (u_xyz(3, :) == zeros([1 n]));
    [df1, da1, db1] = inverse_input(u_delta);
    mask_a = (dan <= da1) & (da1 <= dap); % in range
    mask_b = (dbn <= db1) & (db1 <= dbp); 
    t_dir = is_at_origin' & mask_a & mask_b;
    % [is_at_origin; t_dir'; N_esp']

    tw(tw < 0) = inf;        % Not intersecting in forward direction
    ti = min(tw, [], 2);     % The nearest collision in the 3 DOFs
    ti(N_esp <= 0) = inf;    % 
    ti(t_dir > 0) = inf;

    [c_star, i_star] = min(ti);
end

function tw = solve_intersections(u_xyz, u_delta_xyz, a0, b0, tf0, da, db, f_max)
    v_delta_x = u_delta_xyz(1, :) ./ (tan(b0 + db)');
    v_xyz_x = u_xyz(1, :) ./ (tan(b0 + db)');
    v_norm = vecnorm(u_delta_xyz, 2, 1)';

    % x-axis
    txa = u_delta_xyz(2, :)'.^2 + u_delta_xyz(3, :)'.^2 - v_delta_x'.^2;
    txb = u_xyz(2, :)' .* u_delta_xyz(2, :)' + u_xyz(3, :)' .* u_delta_xyz(3, :)' - v_xyz_x' .* v_delta_x';
    txc = u_xyz(2, :)'.^2 + u_xyz(3, :)'.^2 - v_xyz_x'.^2;
    tx = (-txb - sqrt(txb.^2 - txa .* txc)) ./ txa;
    tx(imag(tx) ~= 0) = inf;

    % y-axis
    ty = (-u_xyz(2, :)' - u_xyz(3, :)' .* tan(a0 + da)) ./ (u_delta_xyz(2, :)' + u_delta_xyz(3, :)' .* tan(a0 + da));

    % z-axis
    kkb = dot(u_xyz, u_delta_xyz, 1)' ./ v_norm.^2;
    kkc = (tf0 .* tf0 - f_max^2) ./ v_norm.^2;
    tz = -kkb + sqrt(kkb .* kkb - kkc);

    tw = [tx, ty, tz];
end

function tf = get_tf_from_u(u, n)
    % u: column vector
    tf = vecnorm(reshape(u, [3 n]), 2, 1)';
end
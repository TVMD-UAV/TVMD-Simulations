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
   
    u0 = get_f(zeros([n 1]), zeros([n 1]), ones([n 1]));
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
            funcx = @(a, b, r) r .* cos(a) .* sin(b);
            funcy = @(a, b, r) - r .* sin(a);
            funcz = @(a, b, r) r .* cos(a) .* cos(b);

            funcx_r = @(a, b) funcx(a, b, f_max); 
            funcy_r = @(a, b) funcy(a, b, f_max); 
            funcz_r = @(a, b) funcz(a, b, f_max); 
            fsurf(funcx_r, funcy_r, funcz_r, [-sigma_a sigma_a -sigma_b sigma_b], 'FaceColor', '#77AC30', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 

            % positive y
            funcx_r = @(r, b) funcx(sigma_a, b, r); 
            funcy_r = @(r, b) funcy(sigma_a, b, r); 
            funcz_r = @(r, b) funcz(sigma_a, b, r); 
            fsurf(funcx_r, funcy_r, funcz_r, [0 f_max -sigma_b sigma_b], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 

            % negative y
            funcx_r = @(r, b) funcx(-sigma_a, b, r); 
            funcy_r = @(r, b) funcy(-sigma_a, b, r); 
            funcz_r = @(r, b) funcz(-sigma_a, b, r); 
            fsurf(funcx_r, funcy_r, funcz_r, [0 f_max -sigma_b sigma_b], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 

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

        [d_max, i_star, N_esp, ti, tw] = calc_nearest_c2(N_esp, u0, u0+u_delta, sigma_a, sigma_b, f_max);

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

% region [calc_nearest_c2]
function [c_star, i_star, N_esp, ti, tw] = calc_nearest_c2(N_esp, u0, ud, sigma_a, sigma_b, f_max)
    % u0: allocation result in previous step
    % ud: temporary target

    % u is 3n x 1 column vector
    % u_xyz is 3 x n matrix
    % v is alias for u / tan
    n = length(N_esp);
    u_xyz = reshape(u0, [3 n]);
    u_delta = ud - u0;
    u_delta_xyz = reshape(u_delta, [3, n]);
    % u_delta_xyz(:, ~N_esp) = 0; % disabled actuators

    % Determine direction
    [f0, a0, b0] = inverse_input(u0);
    [f1, a1, b1] = inverse_input(ud);
    % norm(get_f(a1, b1, f1) - ud)
    a_s = -sigma_a * ones([n 1]);
    b_s = -sigma_b * ones([n 1]);
    a_s(a1 - a0 > 0) = sigma_a;
    b_s(b1 - b0 > 0) = sigma_b;

    % Finding nearest
    tw = solve_intersections(u_xyz, u_delta_xyz, a_s, b_s, f_max);

    % Checking direction of delta
    is_at_origin = (u_xyz(3, :) == zeros([1 n]));
    [df1, da1, db1] = inverse_input(u_delta);
    mask_da = (-sigma_a <= da1) & (da1 <= sigma_a); % in range
    mask_db = (-sigma_b <= db1) & (db1 <= sigma_b); 
    t_dir = is_at_origin' & mask_da & mask_db;
    % [is_at_origin; t_dir'; N_esp']

    % Checking at boundary
    % mask_a0 = ((a0 == sigma_a) & (a1 <= a0)) | ((a0 == -sigma_a) & (a1 >= a0));
    % mask_b0 = ((b0 == sigma_b) & (a1 <= a0)) | ((b0 == -sigma_b) & (a1 >= a0));
    % mask_f0 = ((f0 == f_max) & (f1 < f0)) | ((f0 == 0) & (f1 > 0));

    % tw(tw < 0) = inf;        % Not intersecting in forward direction
    % ti = min(tw, [], 2);     % The nearest collision in the 3 DOFs
    % ti(N_esp <= 0) = inf;

    % Checking direction of delta
    tw(tw < 0) = inf;        % Not intersecting in forward direction
    ti = min(tw, [], 2);     % The nearest collision in the 3 DOFs
    ti(N_esp <= 0) = inf;    % 
    ti(t_dir > 0) = inf;

    [c_star, i_star] = min(ti);
end
% % end region [calc_nearest_c2]

% region [solve_intersection]
function tw = solve_intersections(u_xyz, u_delta_xyz, a_s, b_s, f_max)
    % a_s: bound of a (pos / neg sigmal_a, according to the direction of delta)
    % b_s: bound of a (pos / neg sigmal_b, according to the direction of delta)
    v_delta_y = u_delta_xyz(2, :) ./ (tan(a_s)');
    v_xyz_y = u_xyz(2, :) ./ (tan(a_s)');
    v_norm = vecnorm(u_delta_xyz, 2, 1)';
    tf0 = vecnorm(u_xyz, 2, 1)';

    % x-axis
    txa = u_delta_xyz(1, :)'.^2 + u_delta_xyz(3, :)'.^2 - v_delta_y'.^2;
    txb = u_xyz(1, :)' .* u_delta_xyz(1, :)' + u_xyz(3, :)' .* u_delta_xyz(3, :)' - v_xyz_y' .* v_delta_y';
    txc = u_xyz(1, :)'.^2 + u_xyz(3, :)'.^2 - v_xyz_y'.^2;
    tx = (-txb - sqrt(txb.^2 - txa .* txc)) ./ txa;
    
    txl = -txc ./ (2 * txb);
    mask_a_small = abs(txa) < 1e-10;
    mask_no_sol = txb.^2 - txa .* txc < 0;
    tx(mask_a_small) = txl(mask_a_small);
    tx(mask_no_sol) = inf;

    % y-axis
    ty = -(u_xyz(1, :)' - u_xyz(3, :)' .* tan(b_s)) ./ (u_delta_xyz(1, :)' - u_delta_xyz(3, :)' .* tan(b_s));

    % z-axis
    kkb = dot(u_xyz, u_delta_xyz, 1)' ./ v_norm.^2;
    kkc = (tf0 .* tf0 - f_max^2) ./ v_norm.^2;
    tz = -kkb + sqrt(kkb .* kkb - kkc);
    mask_no_sol = kkb .* kkb - kkc < 0;
    tz(mask_no_sol) = inf;

    tw = [tx ty tz];
end
% end region [solve_intersection]

function tf = get_tf_from_u(u, n)
    % u: column vector
    tf = vecnorm(reshape(u, [3 n]), 2, 1)';
end
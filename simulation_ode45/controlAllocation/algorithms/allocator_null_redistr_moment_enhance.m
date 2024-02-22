function [a, b, F, u] = allocator_null_redistr_moment_enhance(t_d, conf, a0, b0, f0, W, dt)
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

    [f0, N_esp] = sat(conf, pinv(M, pinv_tol) * t_d);

    % Initialization
    uf_d = t_d(1:3);
    ut_d = t_d(4:6);

    Mf = M(1:3, :);
    Mt = M(4:6, :);

    % region [boundary visualization]
    if draw

        % region [moore allocation]
        % moore allocation saturation
        M_esp_dag = pinv(M, pinv_tol);
        moore_f = M_esp_dag * t_d;
        [moore_f, N_moore] = sat(conf, moore_f);
        f_moore = reshape(moore_f, [3 n]);
        % end region [moore allocation]

        % region [moore allocation]
        [eta_int, xi_int, R_int, F_int] = allocator_interior_point(t_d, W, conf, a0, b0, tf0);
        f_int = get_f(eta_int, xi_int, F_int);
        f_int = reshape(f_int, [3 n]);
        % end region [moore allocation]

        figure('Position', [10 10 1600 800])
        aa = reshape(ones([1 n]) .* [sigma_a -sigma_a -sigma_a sigma_a]', [4 * n 1]);
        bb = reshape(ones([1 n]) .* [-sigma_b -sigma_b sigma_b sigma_b]', [4 * n 1]);
        fs = reshape(get_f(aa, bb, f_max), [3, 4, n]);

        f_xyz = reshape(f0, [3 n]);
        for i = 1:n
            subplot(2, n / 2, i);

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

            
            quiver3(0, 0, 0, f_xyz(1, i), f_xyz(2, i), f_xyz(3, i), 'Color', '#000000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
            quiver3(0, 0, 0, f_moore(1, i), f_moore(2, i), f_moore(3, i), 'Color', '#0000AA', 'LineWidth', 1, 'AutoScale', 'off'); hold on
            quiver3(0, 0, 0, f_int(1, i), f_int(2, i), f_int(3, i), 'Color', '#00AAAA', 'LineWidth', 1, 'AutoScale', 'off'); hold on

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
    cmap = jet(n);
    % Iteration
    c = 0; d = 1;
    % N_esp = ones([n 1]); % rechecking
    % TODO: N_esp should be rechecked after the control is known (f_delta)
    Mt_esp = Mt * kron(diag(N_esp), eye(3)); % masking
    ut0 = Mt * f0;
    ut_delta = ut_d - ut0;

    % disp("start")
    while (c < 1) && (rank(Mt_esp) >= 3)
        Mf_esp = Mf * kron(diag(N_esp), eye(3)); % masking
        Mt_esp = Mt * kron(diag(N_esp), eye(3)); % masking
        N_Mf_esp = null(Mf_esp);
        z_esp = pinv(Mt_esp * N_Mf_esp, pinv_tol) * ut_delta;
        f_delta = N_Mf_esp * z_esp;
        N_esp = solve_available(conf, N_esp, f0, f_delta);
        N_esp'
        % ut_delta - Mt * f_delta
        
        % N_esp'
        % f0'
        % f_delta'
        [d_max, i_star, N_esp, ti, tw] = calc_nearest_c2(N_esp, f0, f0 + f_delta, sigma_a, sigma_b, f_max);

        if d_max == inf
            break;
        end

        % Update marked set
        N_esp(i_star) = 0;
        d = d_max;
        d

        if d_max > 1 - c
            d = 1 - c;
        end

        if draw
            % draw boundaries
            f_xyz = reshape(f0, [3 n]);
            f_delta_xyz = reshape(f_delta, [3 n]);
            ub = f_xyz + f_delta_xyz .* ([1 1 1]' * ti');
            ub_x = f_xyz + f_delta_xyz .* ([1 1 1]' * tw(:, 1)');
            ub_y = f_xyz + f_delta_xyz .* ([1 1 1]' * tw(:, 2)');
            ub_z = f_xyz + f_delta_xyz .* ([1 1 1]' * tw(:, 3)');

            for i = 1:n
                subplot(2, n / 2, i);
                quiver3(f0(3 * i - 2), f0(3 * i - 1), f0(3 * i), ...
                    d * f_delta_xyz(1, i), d * f_delta_xyz(2, i), d * f_delta_xyz(3, i), 'Color', cmap(c_idx, :), 'LineWidth', 2, 'AutoScale', 'off'); hold on
                scatter3(ub(1, i), ub(2, i), ub(3, i), 10, 'black'); hold on
                scatter3(ub_x(1, i), ub_x(2, i), ub_x(3, i), 10, 'red'); hold on
                scatter3(ub_y(1, i), ub_y(2, i), ub_y(3, i), 10, 'green'); hold on
                scatter3(ub_z(1, i), ub_z(2, i), ub_z(3, i), 10, 'blue'); hold on
            end

        end

        f0 = f0 + d * f_delta;
        % ut0 = ut0 + d * ut_delta;
        % ut0 = Mt * f0;
        c = c + d;

        % Checking
        if draw
            uo = M * f0;
            fprintf("d=%.2f, c=%.2f, sat=%#d,\t", d, c, i_star);
            [ef, em, df, dm] = output_error(t_d, uo);
            [f1, a1, b1] = inverse_input(uo);
            tef = thrust_efficiency(a1, b1, f1);

            fprintf("[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f],", uo(1), uo(2), uo(3), uo(4), uo(5), uo(6));
            fprintf(" \ttef=%.4f,\tef=%.4f,\tem=%.4f,\tdf=%.4f,\tdm=%.4f \n", tef, ef, em, df, dm);
            pause
        end

        c_idx = c_idx + 1;
    end
    
    % [lllf0, N_esp] = sat(conf, f0);
    % N_esp'
    % fprintf("rank: %d\n", rank(Mt_esp))
    % uf_d - Mf * f0
    % (ut_d - Mt * f0) ./ ut_d

    [F, a, b] = inverse_input(f0);
    u = full_dof_mixing(P, psi, a, b, F);
end

function N_esp = solve_available(conf, N_esp, f0, f_delta)
    n = length(conf('psi'));
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    [tf0, a0, b0] = inverse_input(f0);
    [tf1, a1, b1] = inverse_input(f0 + f_delta);

    % Checking direction of delta
    % is_at_origin = (u_xyz(3, :) == zeros([1 n]));
    % [df1, da1, db1] = inverse_input(u_delta);
    % mask_da = (-sigma_a <= da1) & (da1 <= sigma_a); % in range
    % mask_db = (-sigma_b <= db1) & (db1 <= sigma_b); 
    % t_dir = is_at_origin' & mask_da & mask_db;
    % [is_at_origin; t_dir'; N_esp']

    % Checking at boundary
    % mask_a0 = ((a0 >= sigma_a) & (a1 <= a0)) | ((a0 <= -sigma_a) & (a1 >= a0));
    % mask_b0 = ((b0 >= sigma_b) & (a1 <= a0)) | ((b0 <= -sigma_b) & (a1 >= a0));
    % mask_f0 = ((f0 >= f_max) & (f1 < f0)) | ...
    %           ((f0 <= 0) & (f1 > 0) & (-sigma_a <= a1 & a1 <= sigma_a) & (-sigma_b <= b1 & b1 <= sigma_b));
    % N_esp = mask_f0 | mask_a0 | mask_b0;

    mask_a0 = ((a0 <= -sigma_a) & (a1 < a0)) | ((a0 >= sigma_a) & (a1 > a0));
    mask_b0 = ((b0 <= -sigma_b) & (b1 < b0)) | ((b0 >= sigma_b) & (b1 > b0));
    mask_tf0 = ((tf0 >= f_max) & (tf1 > tf0)) | ((tf0 <= 0) & (tf1 < 0));
    N_esp = N_esp & ~(mask_tf0 | mask_a0 | mask_b0);
end

% region [sat]
function [f0, N_esp] = sat(conf, f)
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    a_m_max = sigma_a;
    a_m_min = -sigma_a;
    b_m_max = sigma_b;
    b_m_min = -sigma_b;
    tf_m_max = f_max;
    tf_m_min = 0;

    [tfl0, al0, bl0] = inverse_input(f);

    mask_a = (a_m_min < al0) & (al0 < a_m_max); % in range
    mask_b = (b_m_min < bl0) & (bl0 < b_m_max); 
    mask_f = (tf_m_min < tfl0) & (tfl0 < tf_m_max); 
    N_esp = mask_a & mask_b & mask_f;

    al0 = min(a_m_max, max(a_m_min, al0));
    bl0 = min(b_m_max, max(b_m_min, bl0));
    tfl0 = min(tf_m_max, max(tf_m_min, tfl0));
    f0 = get_f(al0, bl0, tfl0);
end
% end region [sat]

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
    % is_at_origin = (u_xyz(3, :) == zeros([1 n]));
    % [df1, da1, db1] = inverse_input(u_delta);
    % mask_da = (-sigma_a <= da1) & (da1 <= sigma_a); % in range
    % mask_db = (-sigma_b <= db1) & (db1 <= sigma_b); 
    % t_dir = is_at_origin' & mask_da & mask_db;
    % [is_at_origin; t_dir'; N_esp']

    % Checking at boundary
    % mask_a0 = ((a0 == sigma_a) & (a1 <= a0)) | ((a0 == -sigma_a) & (a1 >= a0));
    % mask_b0 = ((b0 == sigma_b) & (a1 <= a0)) | ((b0 == -sigma_b) & (a1 >= a0));
    % mask_f0 = ((f0 == f_max) & (f1 < f0)) | ((f0 == 0) & (f1 > 0));

    tw(tw < 0) = inf;        % Not intersecting in forward direction
    ti = min(tw, [], 2);     % The nearest collision in the 3 DOFs
    % N_esp = N_esp | mask_a0 | mask_b0;
    ti(N_esp <= 0) = inf;    % 
    % ti(t_dir > 0) = tw(t_dir > 0, 3); 
    % ti(mask_a0) = tw(mask_a0 > 0, 1); 
    % ti(mask_b0) = tw(mask_b0 > 0, 2); 

    [c_star, i_star] = min(ti);
end
% end region [calc_nearest_c2]

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

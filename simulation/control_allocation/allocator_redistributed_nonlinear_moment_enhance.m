function [Tf_d, eta_xd, eta_yd, N_esp, incre, packed_meta, packed_detail, packed_tw, packed_intersections, validness, sat_order, remain_rank] = allocator_redistributed_nonlinear_moment_enhance(env_params, drone_params, ctrl_params, u_d, z, dt, t)
    rate_cons = true;
    % Configurations
    P = drone_params.pos;
    psi = drone_params.psi;
    f_max = drone_params.f_max;
    pinv_tol = 0.0001;
    W = diag([1 1 1 10 10 10]);

    n = length(psi);
    M = get_M(n, psi, P);

    % Initialization
    N_esp0 = ones([n 1]);
    if drone_params.agent_disable && ~isempty(t)
        if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
            for m=1:length(drone_params.agent_disable_id)
                N_esp0(drone_params.agent_disable_id(m)) = 0;
            end
        end
    end
    N_esp = N_esp0;
    % To avoid the contribution of disabled agents in wrench
    M_esp = M * kron(diag(N_esp), eye(3));
    c = 0;

    
    packed_detail = zeros([n+1, 3*n]);
    packed_tw = zeros([n, n, 3]);
    packed_intersections = zeros([n, n, 3, 3]);

    if rate_cons
        [Tf0, eta_x0, eta_y0] = z2raw(n, z, env_params);
        if ctrl_params.pseudo_boundary
            f_attemp = pinv(M_esp, pinv_tol) * u_d;
            [Tf0, eta_x0, eta_y0] = calc_shrink_control(drone_params, ctrl_params, dt, Tf0, eta_x0, eta_y0, f_attemp);
            for k = 1:n 
                if N_esp(k) == 0
                    Tf0(k) = 0; eta_x0(k) = 0; eta_y0(k) = 0; 
                end
            end
        end
        [lower_x, upper_x, lower_y, upper_y] = solve_upper_lower_bounds(drone_params, dt, Tf0, eta_x0, eta_y0);
        packed_meta = [lower_x upper_x lower_y upper_y];
        %Tf0(Tf0 > f_max) = f_max;  % scaling down a bit
        f0 = get_f(eta_x0, eta_y0, Tf0);
        %validness = (lower_x <= eta_x0) & (eta_x0 <= upper_x) & (lower_y <= eta_y0) & (eta_y0 <= upper_y);
    else
        sigma_x = drone_params.sigma_a;
        sigma_y = drone_params.sigma_b;
        lower_x = -sigma_x * ones([n 1]);
        upper_x = sigma_x * ones([n 1]); 
        lower_y = -sigma_y * ones([n 1]);
        upper_y = sigma_y * ones([n 1]);
        f0 = get_f(zeros([n 1]), zeros([n 1]), ones([n 1]));
    end

    % [f0, a0, b0] = inverse_input(n, f0);
    % mask_closed_boundary = (abs(a0 - lower_x) < 1e-5) | (abs(a0 - upper_x) < 1e-5) | (abs(b0 - lower_y) < 1e-5) | (abs(b0 - upper_y) < 1e-5);
    % N_esp(mask_closed_boundary) = 0;
    packed_detail(1, :) = f0';
    u_delta = u_d - M_esp * f0;
    f_TRPI = f0;
    idx = 1;

    sat_order = zeros([n, 1]);
    % sat_order = 0;

    % Iteration
    while (c < 1) && (rank(M_esp) >= 6)
        % Calculate
        M_esp = M * kron(diag(N_esp), eye(3)); % masking
        M_esp_dag = pinv(M_esp, pinv_tol);
        % M_esp_dag = weighted_pinv(M, W, pinv_tol);
        f_delta = M_esp_dag * u_delta;

        %[d_max, i_star, N_esp, ti, tw] = calc_nearest_c2(N_esp, f0, f0 + f_delta, sigma_x, sigma_y, f_max);
        [d_max, i_star, N_esp, ti, tw, txn, txp] = calc_nearest_c2(N_esp, f_TRPI, f_TRPI + f_delta, lower_x, upper_x, lower_y, upper_y, f_max);


        % Update marked set
        for l=1:length(i_star)
            N_esp(i_star(l)) = 0;
            sat_order(idx) = i_star(l);
        end

        if d_max == inf
            idx = idx + 1;
            break;
        end

        d = d_max;

        if d_max > 1 - c
            d = 1 - c;
        end


        packed_tw(idx, :, :) = tw;
        u0_xyz = reshape(f_TRPI, [3, n]);
        u_delta_xyz = reshape(f_delta, [3, n]);
        for k = 1:n
            % packed_intersections(idx, k, 1, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * tw(k, 2))';
            packed_intersections(idx, k, 1, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * ti(k))';
            packed_intersections(idx, k, 2, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * txn(k))';
            packed_intersections(idx, k, 3, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * txp(k))';
        end

        f_TRPI = f_TRPI + d * f_delta;
        c = c + d; 
        packed_detail(idx+1, :) = f_TRPI';
        
        idx = idx + 1;
    end
    remain_rank = rank(M_esp);
    incre = [c c];
    % u0 = M * f0;
    % fprintf("(u_d ) %.4f, %.4f, %.4f; %.4f, %.4f, %.4f\n", u_d(1), u_d(2), u_d(3), u_d(4), u_d(5), u_d(6));
    % fprintf("(u_fd) %.4f, %.4f, %.4f; %.4f, %.4f, %.4f\n", u0(1), u0(2), u0(3), u0(4), u0(5), u0(6));

    c_tau = c;
    f_PTE = f_TRPI;
    if ctrl_params.post_torque_enhance
        % fprintf("enhance\n")
        sat_order = zeros([n, 1]);
        packed_detail = zeros([n+1, 3*n]);
        packed_tw = zeros([n, n, 3]);
        packed_intersections = zeros([n, n, 3, 3]);
        wrench_c = c;
        idx = 1;

        Mf = M(1:3, :);
        Mt = M(4:6, :);
        if c_tau < 1
            % Moment enhancement in the nullspace of force part
            % f0 = f0 * 0.95;
            N_esp = N_esp0;
            
            % Mf_esp = Mf * kron(diag(N_esp), eye(3)); % masking
            Mt_esp = Mt * kron(diag(N_esp), eye(3)); % masking
            % u_tdelta = u_d(4:6) - Mt * f0;
            u_tdelta = u_delta(4:6);
            % N_Mf_esp = null(Mf_esp);
            % z_esp = pinv(Mt_esp * N_Mf_esp, pinv_tol) * u_tdelta;
            % f_delta = N_Mf_esp * z_esp;
            % N_esp = solve_available(drone_params, N_esp, f0, f_delta, lower_x, upper_x, lower_y, upper_y);


            while (c_tau < 1) && (rank(Mt_esp) >= 3)
                % Mf_esp = Mf * kron(diag(N_esp), eye(3)); % masking
                Mf_esp = Mf; % masking
                Mt_esp = Mt * kron(diag(N_esp), eye(3)); % masking
                N_Mf_esp = null(Mf_esp);
                z_esp = pinv(Mt_esp * N_Mf_esp, pinv_tol) * u_tdelta;
                f_delta = N_Mf_esp * z_esp;
                % N_esp = solve_available(drone_params, N_esp, f0, f_delta);
                [d_max, i_star, N_esp, ti, tw, txn, txp] = ...
                    calc_nearest_c2(N_esp, f_PTE, f_PTE + f_delta, lower_x, upper_x, lower_y, upper_y, f_max);

                if d_max == inf
                    break;
                end
        
                % Update marked set
                for l=1:length(i_star)
                    N_esp(i_star(l)) = 0;
                    sat_order(idx) = i_star(l);
                end
                d = d_max;

                if d_max > 1 - (c_tau)
                    d = 1 - (c_tau);
                end
                assert(d >= 0)

                packed_tw(idx, :, :) = tw;
                u0_xyz = reshape(f_PTE, [3, n]);
                u_delta_xyz = reshape(f_delta, [3, n]);
                for k = 1:n
                    % packed_intersections(idx, k, 1, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * tw(k, 2))';
                    packed_intersections(idx, k, 1, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * ti(k))';
                    packed_intersections(idx, k, 2, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * txn(k))';
                    packed_intersections(idx, k, 3, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * txp(k))';
                end

                f_PTE = f_PTE + d * f_delta;
                c_tau = c_tau + d;
                packed_detail(idx+1, :) = f_PTE';
        
                idx = idx + 1;
            end
        end
        remain_rank = rank(Mt * kron(diag(N_esp), eye(3)));
        incre(2) = c_tau;
    end
    % fprintf("%.4f, %.4f, %.4f\n", f_PTE(4), f_PTE(5), f_PTE(6));
    [Tf_d, eta_xd, eta_yd] = inverse_input(n, f_PTE);

    if drone_params.agent_disable && ~isempty(t)
        if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
            for m=1:length(drone_params.agent_disable_id)
                k = drone_params.agent_disable_id(m);
                Tf_d(k) = 0;
                eta_xd(k) = 0;
                eta_yd(k) = 0;
            end
        end
    end

    % Check for validness
    validness = sqrt((lower_x > eta_xd) .* (lower_x - eta_xd).^2 + ...  
                     (eta_xd > upper_x) .* (eta_xd - upper_x).^2 + ... 
                     (lower_y > eta_yd) .* (lower_y - eta_yd).^2 + ... 
                     (eta_yd > upper_y) .* (eta_yd - upper_y).^2);
    %alignment = (u_d(4:6)' * (M(4:6, :) * f0)) / (norm(u_d(4:6)) * norm(M(4:6, :) * f0));
    %validness = alignment;
end


function M_inv = weighted_pinv(M, W, pinv_tol)
    M_inv = W \ M' / (M * W \ M');
end


function N_esp = solve_available(drone_params, N_esp, f0, f_delta, lower_x, upper_x, lower_y, upper_y)
    n = length(drone_params.psi);
    sigma_a = drone_params.sigma_a;
    sigma_b = drone_params.sigma_b;
    f_max = drone_params.f_max;

    [tf0, a0, b0] = inverse_input(n, f0);
    [tf1, a1, b1] = inverse_input(n, f0 + f_delta);

    % Checking direction of delta
    % is_at_origin = (u_xyz(3, :) == zeros([1 n]));
    % [df1, da1, db1] = inverse_input(f_delta);
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

    % mask_a0 = ((a0 <= -sigma_a) & (a1 < a0)) | ((a0 >= sigma_a) & (a1 > a0));
    % mask_b0 = ((b0 <= -sigma_b) & (b1 < b0)) | ((b0 >= sigma_b) & (b1 > b0));
    % mask_tf0 = ((tf0 >= f_max) & (tf1 > tf0)) | ((tf0 <= 0) & (tf1 < 0));
    % N_esp = double(N_esp & ~(mask_tf0 | mask_a0 | mask_b0));
    mask_a0 = ((a0 <= lower_x) & (a1 <= a0)) | ((a0 >= upper_x) & (a1 >= a0));
    mask_b0 = ((b0 <= lower_y) & (b1 <= b0)) | ((b0 >= upper_y) & (b1 >= b0));
    mask_tf0 = ((tf0 >= f_max) & (tf1 >= tf0)) | ((tf0 <= 0) & (tf1 <= 0));
    N_esp = double(N_esp & ~(mask_tf0 | mask_a0 | mask_b0));
end

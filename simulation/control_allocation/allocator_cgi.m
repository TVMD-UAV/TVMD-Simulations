function [Tf_d, eta_xd, eta_yd, N_esp, incre, packed_meta, packed_detail] = allocator_cgi(env_params, drone_params, u_d, z, dt, t)
    % Configurations
    P = drone_params.pos;
    psi = drone_params.psi;

    n = length(psi);
    M = get_M(n, psi, P);

    % Fi = pseudo_inverse(M) * u;
    pinv_tol = 0.0001;

    % Initialization
    fc = zeros([3*n 1]);
    N_esp = ones([n 1]);
    if drone_params.agent_disable && ~isempty(t)
        if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
            for m=1:length(drone_params.agent_disable_id)
                N_esp(drone_params.agent_disable_id(m)) = 0;
            end
        end
    end

    M_esp = M * kron(diag(N_esp), eye(3)); % masking
    
    [Tf0, eta_x0, eta_y0] = z2raw(n, z, env_params);
    Tf_s = Tf0; eta_xs = eta_x0; eta_ys = eta_y0;
    [lower_x, upper_x, lower_y, upper_y] = solve_upper_lower_bounds(drone_params, dt, Tf0, eta_x0, eta_y0);
    packed_meta = [lower_x upper_x lower_y upper_y];
    packed_detail = zeros([n+1, 3*n]);
    count = 1;
    packed_detail(count, :) = fc';
    target_achieved = false;

    while ((sum(N_esp) > 0) && (rank(M_esp) >= 6)) 
        M_esp = M * kron(diag(N_esp), eye(3)); % masking
        M_esp_dag = pinv(M_esp, pinv_tol);
        f = fc + M_esp_dag * (u_d - M * fc);
        [Tf_d, eta_xd, eta_yd] = inverse_input(n, f);

        % Saturation
        [Tf_s, eta_xs, eta_ys] = saturation_with_internal_dynamic(drone_params, dt, Tf_d, eta_xd, eta_yd, Tf0, eta_x0, eta_y0);

        fs = reshape(get_f(eta_xs, eta_ys, Tf_s), [3 n]);
        fd = reshape(f, [3 n]);
        cost = vecnorm(fs - fd, 2, 1);
        assert((length(size(cost)) == 2) && all(size(cost) == [1 n]))

        % Not saturated but exceed bounds
        violations = solve_feasible_violation(drone_params, N_esp, eta_xd, eta_yd, Tf_d, lower_x, upper_x, lower_y, upper_y);
        cost(~violations) = -1;
        % Choose the one violates more
        [d_max, idx] = max(cost);

        if d_max <= 1e-7
            % end of iteration
            % fprintf("Earily end with d=%.4f\n", d_max);
            target_achieved = true;
            break;
        end
        N_esp(idx) = 0;
        fc(3*idx-2:3*idx, 1) = get_f(eta_xs(idx), eta_ys(idx), Tf_s(idx));
        count = count + 1;
        packed_detail(count, :) = fc';
    end 
    % fs = get_f(eta_xs, eta_ys, Tf_s);

    % if ~target_achieved
    %     N_esp = ones([n 1]);
    %     if drone_params.agent_disable && ~isempty(t)
    %         if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
    %             for m=1:length(drone_params.agent_disable_id)
    %                 N_esp(drone_params.agent_disable_id(m)) = 0;
    %             end
    %         end
    %     end
    %     Mt_esp = Mt * kron(diag(N_esp), eye(3)); % masking
    %     t_tdelta = u_d(4:6) - Mt * fs;
    %     while (sum(N_esp) > 0) && (rank(Mt_esp) >= 3)
    %         Mf_esp = Mf * kron(diag(N_esp), eye(3)); % masking
    %         Mt_esp = Mt * kron(diag(N_esp), eye(3)); % masking
    %         N_Mf_esp = null(Mf_esp);
    %         z_esp = pinv(Mt_esp * N_Mf_esp, pinv_tol) * t_tdelta;
    %         f_delta = N_Mf_esp * z_esp;
    %         f = fc + M_esp_dag * (u_d - M * fc);
    %     end
    % end
    incre = [sum(N_esp) rank(M_esp)];
    Tf_d = Tf_s; eta_xd = eta_xs; eta_yd = eta_ys;
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
end

function violations = solve_feasible_violation(drone_params, N_esp, eta_x, eta_y, Tf, lower_x, upper_x, lower_y, upper_y)
    n = length(drone_params.psi);
    f_max = drone_params.f_max;

    mask_x = (eta_x >= lower_x) & (eta_x <= upper_x);
    mask_y = (eta_y >= lower_y) & (eta_y <= upper_y);
    mask_tf = (Tf >= 0) & (Tf < f_max);
    violations = (N_esp & ~(mask_tf & mask_x & mask_y));
end
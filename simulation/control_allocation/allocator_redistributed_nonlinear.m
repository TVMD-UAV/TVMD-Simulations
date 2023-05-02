function [Tf_d, eta_xd, eta_yd, N_esp, c, packed_meta, packed_detail, packed_tw, packed_intersections, validness, sat_order] = allocator_redistributed_nonlinear(env_params, drone_params, u_d, z, dt)
    rate_cons = true;
    % Configurations
    P = drone_params.pos;
    psi = drone_params.psi;
    f_max = drone_params.f_max;
    pinv_tol = 0.0001;

    n = length(psi);
    M = get_M(n, psi, P);

    % Initialization
    N_esp = ones([n 1]);
    M_esp = M;
    c = 0;

    packed_detail = zeros([n, 3*n]);
    packed_tw = zeros([n, n, 3]);
    packed_intersections = zeros([n, n, 3, 3]);

    if rate_cons
        f0 = pinv(M, pinv_tol) * u_d;
        [Tf0, eta_x0, eta_y0] = z2raw(n, z, env_params);
        [Tf0, eta_x0, eta_y0] = calc_shrink_control(drone_params, dt, Tf0, eta_x0, eta_y0, f0);
        [lower_x, upper_x, lower_y, upper_y] = solve_upper_lower_bounds(drone_params, dt, Tf0, eta_x0, eta_y0);
        packed_meta = [lower_x upper_x lower_y upper_y];
        %Tf0(Tf0 > f_max) = f_max;  % scaling down a bit
        u0 = get_f(eta_x0, eta_y0, Tf0);
        %validness = (lower_x <= eta_x0) & (eta_x0 <= upper_x) & (lower_y <= eta_y0) & (eta_y0 <= upper_y);
    else
        sigma_x = drone_params.sigma_a;
        sigma_y = drone_params.sigma_b;
        lower_x = -sigma_x * ones([n 1]);
        upper_x = sigma_x * ones([n 1]); 
        lower_y = -sigma_y * ones([n 1]);
        upper_y = sigma_y * ones([n 1]);
        u0 = get_f(zeros([n 1]), zeros([n 1]), ones([n 1]));
    end

    t0 = M * u0;
    t_delta = u_d - t0;
    idx = 1;

    sat_order = zeros([n, 1]);

    % Iteration
    while (c < 1) && (rank(M_esp) >= 6)
        % Calculate
        M_esp = M * kron(diag(N_esp), eye(3)); % masking
        M_esp_dag = pinv(M_esp, pinv_tol);
        u_delta = M_esp_dag * t_delta;

        %[d_max, i_star, N_esp, ti, tw] = calc_nearest_c2(N_esp, u0, u0 + u_delta, sigma_x, sigma_y, f_max);
        [d_max, i_star, N_esp, ti, tw, txn, txp] = calc_nearest_c2(N_esp, u0, u0 + u_delta, lower_x, upper_x, lower_y, upper_y, f_max);

        if d_max == inf
            idx = idx + 1;
            break;
        end

        % Update marked set
        for l=1:length(i_star)
            N_esp(i_star(l)) = 0;
            sat_order(idx) = i_star(l);
        end

        d = d_max;

        if d_max > 1 - c
            d = 1 - c;
        end


        u0 = u0 + d * u_delta;
        c = c + d; 

        packed_detail(idx, :) = u0';
        packed_tw(idx, :, :) = tw;
        u0_xyz = reshape(u0, [3, n]);
        u_delta_xyz = reshape(u_delta, [3, n]);
        for k = 1:n
            packed_intersections(idx, k, 1, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * tw(k, 2))';
            packed_intersections(idx, k, 2, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * txn(k))';
            packed_intersections(idx, k, 3, :) = (u0_xyz(:, k) + u_delta_xyz(:, k) * txp(k))';
        end
        
        idx = idx + 1;
    end
    [Tf_d, eta_xd, eta_yd] = inverse_input(n, u0);

    % Check for validness
    validness = sqrt((lower_x > eta_xd) .* (lower_x - eta_xd).^2 + ...  
                     (eta_xd > upper_x) .* (eta_xd - upper_x).^2 + ... 
                     (lower_y > eta_yd) .* (lower_y - eta_yd).^2 + ... 
                     (eta_yd > upper_y) .* (eta_yd - upper_y).^2);
    %alignment = (u_d(4:6)' * (M(4:6, :) * u0)) / (norm(u_d(4:6)) * norm(M(4:6, :) * u0));
    %validness = alignment;
end


function M_inv = weighted_pinv(M, W, pinv_tol)
    M_inv = W \ M' * pinv(M / W * M', pinv_tol);
end

function [Tf, eta_x, eta_y] = calc_shrink_control(drone_params, dt, Tf, eta_x, eta_y, f0)
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    r_sigma_x = drone_params.r_sigma_a;
    r_sigma_y = drone_params.r_sigma_b;
    f_max = drone_params.f_max;
    r_f = drone_params.r_f;

    % todo: only shrink the agent when the direction is outward admissible set
    n = length(drone_params.psi);
    [f1, x1, y1] = inverse_input(n, f0);

    eta_x = sat(eta_x, -sigma_x + r_sigma_x*dt*(x1<=-sigma_x), sigma_x - r_sigma_x*dt*(x1>=sigma_x));
    eta_y = sat(eta_y, -sigma_y + r_sigma_y*dt*(y1<=-sigma_y), sigma_y - r_sigma_y*dt*(y1>=sigma_y));
    % Tf = sat(Tf, 0 + r_f*dt*(f1<=0), f_max - r_f*dt*(f1>=f_max));
    Tf = sat(Tf, 0 + r_f*dt, f_max - r_f*dt*(f1>=f_max));
    % eta_x = sat(eta_x, -sigma_x + r_sigma_x*dt, sigma_x - r_sigma_x*dt);
    % eta_y = sat(eta_y, -sigma_y + r_sigma_y*dt, sigma_y - r_sigma_y*dt);
    % Tf = sat(Tf, 0 + r_f*dt, f_max - r_f*dt);
end

function x = sat(x, l, h)
    x = min(max(x, l), h);
end

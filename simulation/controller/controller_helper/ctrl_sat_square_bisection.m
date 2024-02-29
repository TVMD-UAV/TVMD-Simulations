function [theta] = ctrl_sat_square_bisection(drone_params, ctrl_params, n, b1r, b3r, f_r, t)
    %f_r = f_r / norm(f_r);
    %sigma_a = conf('sigma_a');
    b2r = cross(b3r, b1r);
    theta_max = acos(b3r' * f_r / norm(f_r));
    % theta_max = asin(norm(cross(b3r, f_r)) / norm(f_r));
    theta = theta_max / 2;
    % abs(f_r' * b2r)
    % f_r' * b3r * tan(sigma_a)
    % if abs(f_r' * b2r) > f_r' * b3r * tan(sigma_a)
    if ~is_attainable(drone_params, ctrl_params, b1r, b2r, b3r, f_r, t)
        k = cross(b3r, f_r) / norm(cross(b3r, f_r));
        for i=1:n
            b3 = rot_vec_by_theta(b3r, k, theta);
            b2 = cross(b3, b1r) / norm(cross(b3, b1r));
            b1 = cross(b2, b3);
            % if abs(f_r' * b2) < f_r' * b3 * tan(sigma_a)
            if is_attainable(drone_params, ctrl_params, b1, b2, b3, f_r, t)
                theta = theta - 0.5 * theta_max / (2^i);
            else
                theta = theta + 0.5 * theta_max / (2^i);
            end
            % fprintf("%d: %.4f\n", i, theta)
        end
    else
        theta = 0;
    end
end

function feasible = is_attainable(drone_params, ctrl_params, b1, b2, b3, f_r, t)
    feasible = elliptic_cone(drone_params, ctrl_params, b1, b2, b3, f_r, t);
    % feasible = pie(conf, b1, b2, b3, f_r);
end

function feasible = elliptic_cone(drone_params, ctrl_params, b1, b2, b3, f_r, t)
    n = length(drone_params.psi);
    psi = drone_params.psi;
    f_max = drone_params.f_max;
    % Tunable range of relaxation
    sigma_a = drone_params.sigma_a * ctrl_params.attitude_planner_relax_ratio;
    sigma_b = drone_params.sigma_b * ctrl_params.attitude_planner_relax_ratio;

    n_x = sum(psi == 0);
    n_y = n - n_x;

    % % Motor Failure
    if drone_params.agent_disable
        if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
            for m=1:length(drone_params.agent_disable_id)
                if (psi(drone_params.agent_disable_id(m)) == 0)
                    n_x = n_x - 1;
                else
                    n_y = n_y - 1;
                end
            end
        end
    end

    z = f_r' * b3;
    c_x = fun_g2(n_x, sigma_b, z, f_max) + fun_g2(n_y, sigma_a, z, f_max);
    c_y = fun_g2(n_x, sigma_a, z, f_max) + fun_g2(n_y, sigma_b, z, f_max);

    feasible = (z > 0) & (((f_r' * b1 / c_x)^2 + (f_r' * b2 / c_y)^2) <= 1);
end

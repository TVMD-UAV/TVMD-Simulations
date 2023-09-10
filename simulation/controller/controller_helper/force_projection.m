function u_sat = force_projection(drone_params, t, u)
    psi = drone_params.psi;
    sigma_a = drone_params.sigma_a;
    sigma_b = drone_params.sigma_b;
    f_max = drone_params.f_max;
    n = length(psi);

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

    %a = n_x * f_max + (n_y ~= 0) * u(3) * tan(sigma_a);
    %b = n_y * f_max + (n_x ~= 0) * u(3) * tan(sigma_a);
    %if u(3) < 0.1; u(3) = 0.1; end
    if u(3) < 1; u(3) = 1; end
    z = u(3);
    a = fun_g2(n_x, sigma_b, z, f_max) + fun_g2(n_y, sigma_a, z, f_max);
    b = fun_g2(n_x, sigma_a, z, f_max) + fun_g2(n_y, sigma_b, z, f_max);
    t = (a * b) / sqrt(u(1)^2 * b^2 + u(2)^2 * a^2);

    % Directional saturation
    u_sat = u;
    if t < 1
        u_sat(1:2) = t * u(1:2);
    end

    % Magnitude saturation
    u_sat(1:3) = force_magnitude_sat(drone_params, t, u_sat(1:3));
    % roof_lim = n * drone_params.f_max;
    % scale = norm(u_sat(1:3)) / roof_lim;
    % if scale > 1
    %     u_sat(1:3) = u_sat(1:3) / scale;
    % end
end

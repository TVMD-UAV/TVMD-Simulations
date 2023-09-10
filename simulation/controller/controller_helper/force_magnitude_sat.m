function u_sat = force_magnitude_sat(drone_params, t, u)
    n = length(drone_params.psi);

    % % Motor Failure
    if drone_params.agent_disable
        if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
            n = n - length(drone_params.agent_disable_id);
        end
    end

    roof_lim = n * drone_params.f_max;
    scale = norm(u(1:3)) / roof_lim;
    u_sat = u;
    if scale > 1
        u_sat(1:3) = u(1:3) / scale;
    end
end

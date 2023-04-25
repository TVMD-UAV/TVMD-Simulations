function [Tf_d, eta_xd, eta_yd] = allocator_moore_penrose(env_params, drone_params, u_d, z, dt)
    % Configurations
    P = drone_params.pos;
    psi = drone_params.psi;

    n = length(psi);
    M = get_M(n, psi, P);

    % Fi = pseudo_inverse(M) * u;
    pinv_tol = 0.0001;
    Fi = pinv(M, pinv_tol) * u_d;
    [Tf_d, eta_xd, eta_yd] = inverse_input(n, Fi);

    % Saturation
    [Tf0, eta_x0, eta_y0] = z2raw(n, z, env_params);
    [Tf_d, eta_xd, eta_yd] = saturation_with_internal_dynamic(drone_params, dt, Tf_d, eta_xd, eta_yd, Tf0, eta_x0, eta_y0);
end
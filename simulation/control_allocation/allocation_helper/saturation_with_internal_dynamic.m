function [Tf_d, eta_xd, eta_yd] = saturation_with_internal_dynamic(drone_params, dt, Tf, eta_x, eta_y, Tf0, eta_x0, eta_y0)
    [lower_x, upper_x, lower_y, upper_y] = solve_upper_lower_bounds(drone_params, dt, Tf0, eta_x0, eta_y0);
    eta_xd = sat(eta_x, lower_x, upper_x);
    eta_yd = sat(eta_y, lower_y, upper_y);
    Tf_d = sat(Tf, 0, drone_params.f_max);
    % Tf_d = Tf;
end

function [lower_x, upper_x, lower_y, upper_y] = solve_upper_lower_bounds(drone_params, dt, Tf0, eta_x0, eta_y0)
    %% Parameters
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    r_sigma_x = drone_params.r_sigma_a;
    r_sigma_y = drone_params.r_sigma_b;

    lower_x = max(-sigma_x, eta_x0 - r_sigma_x * dt);
    upper_x = min( sigma_x, eta_x0 + r_sigma_x * dt);

    lower_y = max(-sigma_y, eta_y0 - r_sigma_y * dt);
    upper_y = min( sigma_y, eta_y0 + r_sigma_y * dt);
end
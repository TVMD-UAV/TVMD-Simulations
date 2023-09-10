function plot_est_boundary_elliptic_cone(drone_params, R, p, scaling)
    psi = drone_params.psi;
    sigma_a = drone_params.sigma_a;
    sigma_b = drone_params.sigma_b;
    f_max = drone_params.f_max;
    n = length(psi);
    n_x = sum(psi == 0);
    n_y = n - n_x;

    c_x = @(z) fun_g2(n_x, sigma_b, z, f_max) + fun_g2(n_y, sigma_a, z, f_max);
    c_y = @(z) fun_g2(n_x, sigma_a, z, f_max) + fun_g2(n_y, sigma_b, z, f_max);
    
    funcx = @(t, z) c_x(z) .* cos(t) * scaling;
    funcy = @(t, z) c_y(z) .* sin(t) * scaling;
    funcz = @(t, z) z * scaling;

    funcx_r = @(t, z) R(1, 1) * funcx(t, z) + R(1, 2) * funcy(t, z) + R(1, 3) * funcz(t, z) + p(1);
    funcy_r = @(t, z) R(2, 1) * funcx(t, z) + R(2, 2) * funcy(t, z) + R(2, 3) * funcz(t, z) + p(2);
    funcz_r = @(t, z) R(3, 1) * funcx(t, z) + R(3, 2) * funcy(t, z) + R(3, 3) * funcz(t, z) + p(3);
    fsurf(funcx_r, funcy_r, funcz_r, [0 2*pi 0.1 n*f_max], 'FaceColor', '#77AC30', 'FaceAlpha', 0.2, 'EdgeColor', 'none','HandleVisibility','off'); hold on 
end

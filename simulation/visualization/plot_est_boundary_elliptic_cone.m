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
    
    % Estimation of the boundary
    funcx = @(t, z) c_x(z) .* cos(t) * scaling;
    funcy = @(t, z) c_y(z) .* sin(t) * scaling;
    % funcz = @(t, z) z * scaling;
    funcz = @(t, z) func(t, z) * scaling;

    funcx_r = @(t, z) R(1, 1) * funcx(t, z) + R(1, 2) * funcy(t, z) + R(1, 3) * funcz(t, z) + p(1);
    funcy_r = @(t, z) R(2, 1) * funcx(t, z) + R(2, 2) * funcy(t, z) + R(2, 3) * funcz(t, z) + p(2);
    funcz_r = @(t, z) R(3, 1) * funcx(t, z) + R(3, 2) * funcy(t, z) + R(3, 3) * funcz(t, z) + p(3);
    
    z0 = n*f_max;
    fsurf(funcx_r, funcy_r, funcz_r, [0 2*pi 0.1 z0], 'FaceColor', '#00AA00', 'FaceAlpha', 0.2, 'EdgeColor', '#008800','HandleVisibility','off'); hold on 

    % Estimate the roof
    c_x0 = c_x(z0) * scaling;
    c_y0 = c_y(z0) * scaling;
    funcx_r2 = @(t, m) c_x0 * cos(t) * m;
    funcy_r2 = @(t, m) c_y0 * sin(t) * m;
    % funcz_r2 = @(t, m) sqrt((n*f_max)^2 - funcx_r2(t, m)^2 - funcy_r2(t, m)^2);
    funcz_r2 = @(t, m) func2(t, m);
    fsurf(funcx_r2, funcy_r2, funcz_r2, [0 2*pi 0.01 1], 'FaceColor', '#00AA00', 'FaceAlpha', 0.1, 'EdgeColor', '#008800','HandleVisibility','off'); hold on 

    function z = func(t, z)
        x_s = funcx(t, z);
        y_s = funcy(t, z);
        z_s = z;
        if x_s^2 + y_s^2 + z_s^2 <= (n*f_max)^2 
            z = z_s;
        else
            z = NaN;
        end
    end

    function z = func2(t, z)
        x_s = funcx_r2(t, z);
        y_s = funcy_r2(t, z);
        z_s = sqrt((n*f_max)^2 - x_s^2 - y_s^2);
        c_x0 = c_x(z_s);
        c_y0 = c_y(z_s);
        if (x_s / c_x0)^2 + (y_s / c_y0)^2 <= 1
            z = z_s;
        else
            z = NaN;
        end
    end
end

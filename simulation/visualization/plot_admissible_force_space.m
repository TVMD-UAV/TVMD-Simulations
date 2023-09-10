function plot_admissible_force_space(drone_params, bound, R, p, scale, alpha)
    % lower_x = -drone_params.sigma_a;
    % upper_x = drone_params.sigma_a; 
    % lower_y = -drone_params.sigma_b; 
    % upper_y = drone_params.sigma_b;
    lower_x = bound(1);
    upper_x = bound(2); 
    lower_y = bound(3); 
    upper_y = bound(4);
    f_max = drone_params.f_max * scale;

    % roof
    funcx = @(a, b, r) r .* cos(a) .* sin(b);
    funcy = @(a, b, r) - r .* sin(a);
    funcz = @(a, b, r) r .* cos(a) .* cos(b);

    funcx_r = @(a, b) R(1, 1) * funcx(a, b, f_max) + R(1, 2) * funcy(a, b, f_max) + R(1, 3) * funcz(a, b, f_max) + p(1);
    funcy_r = @(a, b) R(2, 1) * funcx(a, b, f_max) + R(2, 2) * funcy(a, b, f_max) + R(2, 3) * funcz(a, b, f_max) + p(2);
    funcz_r = @(a, b) R(3, 1) * funcx(a, b, f_max) + R(3, 2) * funcy(a, b, f_max) + R(3, 3) * funcz(a, b, f_max) + p(3);

    fsurf(funcx_r, funcy_r, funcz_r, [lower_x upper_x lower_y upper_y], 'FaceColor', '#77AC30', 'FaceAlpha', alpha, 'EdgeColor', 'none'); hold on 

    % positive y
    funcx_r = @(r, b) R(1, 1) * funcx(upper_x, b, r) + R(1, 2) * funcy(upper_x, b, r) + R(1, 3) * funcz(upper_x, b, r) + p(1);
    funcy_r = @(r, b) R(2, 1) * funcx(upper_x, b, r) + R(2, 2) * funcy(upper_x, b, r) + R(2, 3) * funcz(upper_x, b, r) + p(2);
    funcz_r = @(r, b) R(3, 1) * funcx(upper_x, b, r) + R(3, 2) * funcy(upper_x, b, r) + R(3, 3) * funcz(upper_x, b, r) + p(3);
    fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', alpha, 'EdgeColor', 'none'); hold on 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % funcx_r = @(r, b) funcx(-upper_x, b, r); 
    % funcy_r = @(r, b) funcy(-upper_x, b, r); 
    % funcz_r = @(r, b) funcz(-upper_x, b, r); 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % negative y
    funcx_r = @(r, b) R(1, 1) * funcx(lower_x, b, r) + R(1, 2) * funcy(lower_x, b, r) + R(1, 3) * funcz(lower_x, b, r) + p(1);
    funcy_r = @(r, b) R(2, 1) * funcx(lower_x, b, r) + R(2, 2) * funcy(lower_x, b, r) + R(2, 3) * funcz(lower_x, b, r) + p(2);
    funcz_r = @(r, b) R(3, 1) * funcx(lower_x, b, r) + R(3, 2) * funcy(lower_x, b, r) + R(3, 3) * funcz(lower_x, b, r) + p(3);
    fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', alpha, 'EdgeColor', 'none'); hold on 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % funcx_r = @(r, b) funcx(-lower_x, b, r); 
    % funcy_r = @(r, b) funcy(-lower_x, b, r); 
    % funcz_r = @(r, b) funcz(-lower_x, b, r); 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 
end
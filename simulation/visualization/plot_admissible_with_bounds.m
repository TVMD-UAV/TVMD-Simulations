function plot_admissible_with_bounds(bound, f_max)
    lower_x = bound(1);
    upper_x = bound(2); 
    lower_y = bound(3); 
    upper_y = bound(4);

    % roof
    funcx = @(a, b, r) r .* cos(a) .* sin(b);
    funcy = @(a, b, r) - r .* sin(a);
    funcz = @(a, b, r) r .* cos(a) .* cos(b);

    funcx_r = @(a, b) funcx(a, b, f_max); 
    funcy_r = @(a, b) funcy(a, b, f_max); 
    funcz_r = @(a, b) funcz(a, b, f_max); 
    fsurf(funcx_r, funcy_r, funcz_r, [lower_x upper_x lower_y upper_y], 'FaceColor', '#77AC30', 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % positive y
    funcx_r = @(r, b) funcx(upper_x, b, r); 
    funcy_r = @(r, b) funcy(upper_x, b, r); 
    funcz_r = @(r, b) funcz(upper_x, b, r); 
    fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    funcx_r = @(r, b) funcx(-upper_x, b, r); 
    funcy_r = @(r, b) funcy(-upper_x, b, r); 
    funcz_r = @(r, b) funcz(-upper_x, b, r); 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % negative y
    funcx_r = @(r, b) funcx(lower_x, b, r); 
    funcy_r = @(r, b) funcy(lower_x, b, r); 
    funcz_r = @(r, b) funcz(lower_x, b, r); 
    fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    funcx_r = @(r, b) funcx(-lower_x, b, r); 
    funcy_r = @(r, b) funcy(-lower_x, b, r); 
    funcz_r = @(r, b) funcz(-lower_x, b, r); 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 
end
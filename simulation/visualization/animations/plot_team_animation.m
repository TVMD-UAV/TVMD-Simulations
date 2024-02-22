function plot_team_animation(env_params, drone_params, fps, t, P, R, traj, zo, bounds, options, cam_pos, fname_posfix)
    n = length(drone_params.psi);

    r = zeros([floor(fps * t(end)) 1]);
    idx = 1;
    r(idx) = 1;
    for i=1:length(t)
        if t(i) > idx * (1/fps)
            idx = idx + 1;
            r(idx) = i;
        end
    end

    scaling = 0.01;

    % Data processing
    beta_allo = drone_params.beta_allo;
    P_prop = env_params.rho * env_params.prop_d^4 * beta_allo;
    eta_x = zo(:, 1:4:size(zo, 2));
    eta_y = zo(:, 2:4:size(zo, 2));
    Tf = P_prop(1,1) * (zo(:, 3:4:size(zo, 2)).^2 + zo(:, 4:4:size(zo, 2)).^2);

    % figure('Position', [10 10 1200 1200])
    figure('Position', [10 10 1200 800])
    warning('off', 'MATLAB:hg:ColorSpec_None')
    set(gca, 'DataAspectRatio', [1 1 1])
    animation_name = strcat(options('foldername'), options('filename'), '_3d', fname_posfix, '.gif');
    set(groot,{'DefaultAxesXColor','DefaultAxesYColor','DefaultAxesZColor', 'defaultfigurecolor'},{'none','none','none','none'})

    % scatter3(P(r, 1), P(r, 2), P(r, 3), 'Color', 'none'); hold on
    scatter3([0 max(P(:, 1)) + 2], [0 max(P(:, 2)) + 2], [0 max(P(:, 3)) + 2], 'Color', 'none'); hold on
    % ss_traj = scatter3(traj(1, 1, 1), traj(1, 2, 1), traj(1, 3, 1), "red", 'Marker', '.'); hold on
    % ss_state = scatter3(P(1, 1), P(1, 2), P(1, 3), "blue"); hold on
    ss_traj = plot3(traj(1, 1, 1), traj(1, 2, 1), traj(1, 3, 1), "Color", "#FF0000", "LineStyle", "--", "LineWidth", 3); hold on
    ss_state = plot3(P(1, 1), P(1, 2), P(1, 3), "Color", "#00FFFF", "LineWidth", 3); hold on

    f0 = get_f(eta_x(r(1), :)', eta_y(r(1), :)', Tf(r(1), :)');
    f0 = reshape(f0, [3 n]);
    [patch_obj, quiver_obj, beam_obj] = plot_3kg_swarm(drone_params, P(r(1), :), R(r(1), :, :), 1 + 256 * t(r(1)) / t(end), f0); hold on
    fs_obj = plot_animation_est_boundary_elliptic_cone("", 1, drone_params, squeeze(R(r(1), :, :)), P(1, 1:3), scaling);

    [vec, eta_xq, eta_yq, Tfq] = actuator_mixing(squeeze(zo(r(1), :))', drone_params, env_params);
    f_rd = squeeze(R(r(1), :, :)) * vec(1:3) * scaling;
    fquiver_obj = quiver3(P(r(1), 1), P(r(1), 2), P(r(1), 3), f_rd(1), f_rd(2), f_rd(3), 'LineStyle', '-', 'Color', '#FF0000', 'LineWidth', 3, 'AutoScale', 'off'); hold on 

    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    hlight = camlight('headlight'); 
    p.AmbientStrength = 0.1;
    p.SpecularStrength = 1;
    p.DiffuseStrength = 1;
    set(gcf,'Color',[0, 0, 0])
    set(gca,'Color',[0,0,0])

    camzoom(1.002)
    for i = 1:length(r)
        % campos(squeeze(traj(1:3, 1, r(i))) + [-3 3 3]')
        % camtarget(squeeze(traj(1:3, 1, r(i))))
        % campos(squeeze(P(r(i), 1:3))' + [-20 20 20]')
        % camtarget(squeeze(P(r(i), 1:3))')
        
        % campos(squeeze(P(r(i), 1:3))' + [20 20 20]')
        campos(squeeze(P(r(i), 1:3))' + cam_pos)
        camtarget(squeeze(P(r(i), 1:3))')
        camzoom(1)
        camlight(hlight,'headlight')

        ss_traj.XData = traj(1, 1, 1:r(i));
        ss_traj.YData = traj(2, 1, 1:r(i));
        ss_traj.ZData = traj(3, 1, 1:r(i));

        ss_state.XData = P(1:r(i), 1);
        ss_state.YData = P(1:r(i), 2);
        ss_state.ZData = P(1:r(i), 3);

        set(gca, 'DataAspectRatio', [1 1 1])
        f0 = get_f(eta_x(r(i), :)', eta_y(r(i), :)', Tf(r(i), :)');
        f0 = reshape(f0, [3 n]) * scaling;
        draw_3kg_swarm_animation(drone_params, patch_obj, quiver_obj, beam_obj, P(r(i), :), R(r(i), :, :), 1 + 256 * t(r(i)) / t(end), f0);

        % Admissible Force Space
        % fsurf_obj = gobjects([n, 3]);
        % for k=1:n
        %     fsurf_obj(k, :) = plot_animation_admissible_with_bounds(k, drone_params, squeeze(bounds(k, :, r(i))), squeeze(R(r(i), :, :)), P(r(i), :), scaling, 0.2);
        % end

        % Attainable Force Space
        delete(fs_obj)
        fs_obj = plot_animation_est_boundary_elliptic_cone(fs_obj, 1, drone_params, squeeze(R(r(i), :, :)), P(r(i), 1:3), scaling);
        
        update_force_arrow(i, fquiver_obj, r, zo, R, P, drone_params, env_params, scaling)

        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if i == 1
            imwrite(imind, cm, animation_name, 'gif', 'Loopcount', inf, 'DelayTime', 1/fps);
        else
            imwrite(imind, cm, animation_name, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
        end

        % delete(fsurf_obj)
        
    end

end

function update_force_arrow(i, fquiver_obj, r, zo, R, P, drone_params, env_params, scaling)
    [vec, eta_xq, eta_yq, Tfq] = actuator_mixing(squeeze(zo(r(i), :))', drone_params, env_params);
    f_rd = squeeze(R(r(i), :, :)) * vec(1:3) * scaling;
    fquiver_obj.XData = P(r(i), 1);
    fquiver_obj.YData = P(r(i), 2);
    fquiver_obj.ZData = P(r(i), 3);
    fquiver_obj.UData = f_rd(1);
    fquiver_obj.VData = f_rd(2);
    fquiver_obj.WData = f_rd(3);
end


function fs_obj = plot_animation_est_boundary_elliptic_cone(fs_obj, i, drone_params, R, p, scaling)
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

    if i == 1
        fs_obj = fsurf(funcx_r, funcy_r, funcz_r, [0 2*pi 0.1 n*f_max], 'FaceColor', '#77AC30', 'FaceAlpha', 0.4, 'EdgeColor', 'none','HandleVisibility','off'); hold on 
    else 
        fs_obj.XData = funcx_r;
        fs_obj.YData = funcy_r;
        fs_obj.DData = funcz_r;
    end
end

function fsurf_obj = plot_animation_admissible_with_bounds(i, drone_params, bound, Rt, p, scale, alpha)
    fsurf_obj = gobjects([3 1]);
    lower_x = bound(1);
    upper_x = bound(2); 
    lower_y = bound(3); 
    upper_y = bound(4);
    f_max = drone_params.f_max * scale;

    pt = drone_params.pos(:, i);
    I_pt = Rt * pt + p;
    R = Rz(drone_params.psi(i));

    % roof
    funcx = @(a, b, r) r .* cos(a) .* sin(b);
    funcy = @(a, b, r) - r .* sin(a);
    funcz = @(a, b, r) r .* cos(a) .* cos(b);

    funcx_r = @(a, b) R(1, 1) * funcx(a, b, f_max) + R(1, 2) * funcy(a, b, f_max) + R(1, 3) * funcz(a, b, f_max) + I_pt(1);
    funcy_r = @(a, b) R(2, 1) * funcx(a, b, f_max) + R(2, 2) * funcy(a, b, f_max) + R(2, 3) * funcz(a, b, f_max) + I_pt(2);
    funcz_r = @(a, b) R(3, 1) * funcx(a, b, f_max) + R(3, 2) * funcy(a, b, f_max) + R(3, 3) * funcz(a, b, f_max) + I_pt(3);

    fsurf_obj(1) = fsurf(funcx_r, funcy_r, funcz_r, [lower_x upper_x lower_y upper_y], 'FaceColor', '#77AC30', 'FaceAlpha', alpha, 'EdgeColor', 'none'); hold on 

    % positive y
    funcx_r = @(r, b) R(1, 1) * funcx(upper_x, b, r) + R(1, 2) * funcy(upper_x, b, r) + R(1, 3) * funcz(upper_x, b, r) + I_pt(1);
    funcy_r = @(r, b) R(2, 1) * funcx(upper_x, b, r) + R(2, 2) * funcy(upper_x, b, r) + R(2, 3) * funcz(upper_x, b, r) + I_pt(2);
    funcz_r = @(r, b) R(3, 1) * funcx(upper_x, b, r) + R(3, 2) * funcy(upper_x, b, r) + R(3, 3) * funcz(upper_x, b, r) + I_pt(3);
    fsurf_obj(2) = fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', alpha, 'EdgeColor', 'none'); hold on 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % funcx_r = @(r, b) funcx(-upper_x, b, r); 
    % funcy_r = @(r, b) funcy(-upper_x, b, r); 
    % funcz_r = @(r, b) funcz(-upper_x, b, r); 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % negative y
    funcx_r = @(r, b) R(1, 1) * funcx(lower_x, b, r) + R(1, 2) * funcy(lower_x, b, r) + R(1, 3) * funcz(lower_x, b, r) + I_pt(1);
    funcy_r = @(r, b) R(2, 1) * funcx(lower_x, b, r) + R(2, 2) * funcy(lower_x, b, r) + R(2, 3) * funcz(lower_x, b, r) + I_pt(2);
    funcz_r = @(r, b) R(3, 1) * funcx(lower_x, b, r) + R(3, 2) * funcy(lower_x, b, r) + R(3, 3) * funcz(lower_x, b, r) + I_pt(3);
    fsurf_obj(3) = fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', alpha, 'EdgeColor', 'none'); hold on 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

    % funcx_r = @(r, b) funcx(-lower_x, b, r); 
    % funcy_r = @(r, b) funcy(-lower_x, b, r); 
    % funcz_r = @(r, b) funcz(-lower_x, b, r); 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 
end
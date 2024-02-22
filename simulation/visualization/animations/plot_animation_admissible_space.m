function plot_animation_admissible_space(env_params, drone_params, fps, t, P, zo, options)
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

    scaling = 0.02;
    
    % Data processing
    beta_allo = drone_params.beta_allo;
    P_prop = env_params.rho * env_params.prop_d^4 * beta_allo;
    eta_x = zo(:, 1:4:size(zo, 2));
    eta_y = zo(:, 2:4:size(zo, 2));
    Tf = P_prop(1,1) * (zo(:, 3:4:size(zo, 2)).^2 + zo(:, 4:4:size(zo, 2)).^2);

    % Figure settings
    warning('off', 'MATLAB:hg:ColorSpec_None')
    animation_name = strcat(options('foldername'), options('filename'), '_3d_admissible.gif');
    set(groot,{'DefaultAxesXColor','DefaultAxesYColor','DefaultAxesZColor', 'defaultfigurecolor'},{'none','none','none','none'})
    

    figure('Position', [10 10 800 800])
    camproj perspective
    set(gca, 'DataAspectRatio', [1 1 1])    
    % set(gcf, 'Renderer', 'painters')
    set(gcf,'Color',[0, 0, 0])
    set(gca,'Color',[0, 0, 0])

    campos([20 20 20]')
    camtarget([0 0 0]')
    camzoom(1)

    f0 = get_f(eta_x(r(1), :)', eta_y(r(1), :)', Tf(r(1), :)');
    f0 = reshape(f0, [3 n]) * scaling;
    [patch_obj, quiver_obj, beam_obj] = plot_3kg_swarm(drone_params, [0 0 0], eye(3), 1, f0 * 0); 
    
    lower_x = -drone_params.sigma_a;
    upper_x = drone_params.sigma_a; 
    lower_y = -drone_params.sigma_b; 
    upper_y = drone_params.sigma_b;
    for k=1:n 
        p = drone_params.pos(:, k);
        R = Rz(drone_params.psi(k));
        plot_admissible_force_space(drone_params, [lower_x upper_x lower_y upper_y], R, p, scaling, 0.2);
    end
    
    for i = 1:length(r)

        f0 = get_f(eta_x(r(i), :)', eta_y(r(i), :)', Tf(r(i), :)');
        f0 = reshape(f0, [3 n]) * scaling;
        draw_3kg_swarm_animation(drone_params, patch_obj, quiver_obj, beam_obj, [0 0 0], eye(3), 1 + 256 * t(r(i)) / t(end), f0);

        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if i == 1
            imwrite(imind, cm, animation_name, 'gif', 'Loopcount', inf, 'DelayTime', 1/fps);
        else
            imwrite(imind, cm, animation_name, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
        end
    end
end
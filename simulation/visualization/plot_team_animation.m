function plot_team_animation(env_params, drone_params, fps, t, P, R, traj, zo, options)
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

    % Data processing
    beta_allo = drone_params.beta_allo;
    P_prop = env_params.rho * env_params.prop_d^4 * beta_allo;
    eta_x = zo(:, 1:4:size(zo, 2));
    eta_y = zo(:, 2:4:size(zo, 2));
    Tf = P_prop(1,1) * (zo(:, 3:4:size(zo, 2)).^2 + zo(:, 4:4:size(zo, 2)).^2);

    figure('Position', [10 10 1200 1200])
    warning('off', 'MATLAB:hg:ColorSpec_None')
    set(gca, 'DataAspectRatio', [1 1 1])
    animation_name = strcat(options('foldername'), options('filename'), '_3d.gif');

    scatter3(P(r, 1), P(r, 2), P(r, 3), 'Color', 'none'); hold on
    scatter3([0 max(P(:, 1)) + 2], [0 max(P(:, 2)) + 2], [0 max(P(:, 3)) + 2], 'Color', 'none'); hold on
    ss_traj = scatter3(traj(1, 1, 1), traj(1, 2, 1), traj(1, 3, 1), "red", 'Marker', '.'); hold on
    ss_state = scatter3(P(1, 1), P(1, 2), P(1, 3), "blue"); hold on
    f0 = get_f(eta_x(r(1), :)', eta_y(r(1), :)', Tf(r(1), :)');
    f0 = reshape(f0, [3 n]);
    [patch_obj, quiver_obj, beam_obj] = plot_3kg_swarm(drone_params, P(r(1), :), R(r(1), :, :), 1 + 256 * t(r(1)) / t(end), f0); hold on
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

    for i = 1:length(r)
        campos(squeeze(traj(1:3, 1, r(i))) + [-3 3 3]')
        camtarget(squeeze(traj(1:3, 1, r(i))))
        camlight(hlight,'headlight')

        ss_traj.XData = traj(1, 1, 1:r(i));
        ss_traj.YData = traj(2, 1, 1:r(i));
        ss_traj.ZData = traj(3, 1, 1:r(i));

        ss_state.XData = P(1:r(i), 1);
        ss_state.YData = P(1:r(i), 2);
        ss_state.ZData = P(1:r(i), 3);

        set(gca, 'DataAspectRatio', [1 1 1])
        f0 = get_f(eta_x(r(i), :)', eta_y(r(i), :)', Tf(r(i), :)');
        f0 = reshape(f0, [3 n]);
        draw_3kg_swarm_animation(drone_params, patch_obj, quiver_obj, beam_obj, P(r(i), :), R(r(i), :, :), 1 + 256 * t(r(i)) / t(end), f0);

        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if i == 1
            imwrite(imind, cm, animation_name, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(imind, cm, animation_name, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end

    end

end
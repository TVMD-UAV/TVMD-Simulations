function plot_team_3d(env_params, drone_params, num_slot, t, P, R, traj, zo, options, adjust_campos)
    camproj perspective
    n = length(drone_params.psi);
    figure('Position', [10 10 1200 1200])

    if t(end) * 1 < num_slot; num_slot = t(end) * 0.2; end
    interval = floor(length(t) / num_slot);
    fprintf("Ploting interval: %d\n", interval);
    r = 1:interval:length(t);

    % Data processing
    beta_allo = drone_params.beta_allo;
    P_prop = env_params.rho * env_params.prop_d^4 * beta_allo;
    eta_x = zo(:, 1:4:size(zo, 2));
    eta_y = zo(:, 2:4:size(zo, 2));
    Tf = P_prop(1,1) * (zo(:, 3:4:size(zo, 2)).^2 + zo(:, 4:4:size(zo, 2)).^2);
    % Draw desired trajectory
    % scatter3(traj(:, 1, 1), traj(:, 2, 1), traj(:, 3, 1), 4, "red"); hold on
    scatter3(traj(1, 1, :), traj(2, 1, :), traj(3, 1, :), 10, "red", 'Marker', '.'); hold on

    % Draw positions
    scatter3(P(:, 1), P(:, 2), P(:, 3), 2, t(:)); hold on
    %quiver3(CoP(r, 1), CoP(r, 2), CoP(r, 3), thrust(r, 1), thrust(r, 2), thrust(r, 3), 'magenta')

    % Draw coordinates of the agent
    q1 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(r, 1, 1), R(r, 2, 1), R(r, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(r, 1, 2), R(r, 2, 2), R(r, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(r, 1, 3), R(r, 2, 3), R(r, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';

    % Draw agent body
    if n > 1
        for i = 1:length(r)
            f0 = get_f(eta_x(r(i), :)', eta_y(r(i), :)', Tf(r(i), :)');
            f0 = reshape(f0, [3 n]) * 0.1;
            plot_3kg_swarm(drone_params, P(r(i), :), R(i, :, :), 1 + 256 * t(r(i)) / t(end), f0); hold on
        end
    else
        for i = 1:length(r)
            draw_agent_quad(P(r(i), :), R(i, :, :), 1 + 256 * t(r(i)) / t(end)); hold on
        end
    end 

    xlabel('x', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('y', 'FontName', 'Times New Roman', 'FontSize', 12)
    zlabel('z', 'FontName', 'Times New Roman', 'FontSize', 12)
    if adjust_campos; campos([-3 25 3]); end
    % title('3D view')
    
    tt = colorbar;
    ylabel(tt, 'Time (sec)', 'FontName', 'Times New Roman');
    
    pp = tt.Position;
    pp(4) = pp(4) * 0.8;
    pp(1) = pp(1) + 0.05;

    tt.Label.Position(1) = 0.5;
    tt.Label.Position(2) = -2;
    tt.Label.Rotation = 0;
    set(tt, 'Position', pp);

    set(gca, 'DataAspectRatio', [1 1 1])
    set(gcf, 'Renderer', 'painters')
    print(gcf, '-depsc', 'test.eps')
    savefig_helper(options, '_3d');
end
function plot_3d(t, r, P, CoP, traj, thrust, R, options)
    figure
    scatter3(P(:, 1), P(:, 2), P(:, 3), 10, t(:), 'Marker', '.'); hold on
    % quiver3(CoP(r, 1), CoP(r, 2), CoP(r, 3), thrust(1,r), thrust(2,r), thrust(3,r), 'magenta')

    % Draw coordinates of the agent
    q1 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(r, 1, 1), R(r, 2, 1), R(r, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(r, 1, 2), R(r, 2, 2), R(r, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(r, 1, 3), R(r, 2, 3), R(r, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';

    % Draw desired trajectory
    scatter3(traj(1, 1, :), traj(2, 1, :), traj(3, 1, :), 10, "red", 'Marker', '.'); hold on

    % Draw agent body
    for i = 1:length(r)
        draw_agent_quad(P(r(i), :), R(r(i), :, :), 1 + 256 * t(r(i)) / t(end)); hold on
    end

    xlabel('$x$', 'interpreter', 'latex', 'FontSize', 12)
    ylabel('$y$', 'interpreter', 'latex', 'FontSize', 12)
    zlabel('$z$', 'interpreter', 'latex', 'FontSize', 12)

    tt = colorbar;
    ylabel(tt, 'Time (sec)', 'FontName', 'Times New Roman')
    tt.Label.Position(1) = 0.5;
    tt.Label.Position(2) = -2;
    tt.Label.Rotation = 0;
    
    set(gcf, 'Renderer', 'painters')
    set(gca, 'DataAspectRatio', [1 1 1])
    print(gcf, '-depsc', 'test.eps')
    savefig_helper(options, '_3d');
end

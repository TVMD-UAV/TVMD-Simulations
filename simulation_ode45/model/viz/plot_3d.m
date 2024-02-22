function plot_3d(t, r, P, CoP, traj, thrust, R, options)
    figure
    scatter3(P(r, 1), P(r, 2), P(r, 3), 40, t(r)); hold on
    quiver3(CoP(r, 1), CoP(r, 2), CoP(r, 3), thrust(r, 1), thrust(r, 2), thrust(r, 3), 'magenta')

    % Draw coordinates of the agent
    q1 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 1), R(:, 2, 1), R(:, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 2), R(:, 2, 2), R(:, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 3), R(:, 2, 3), R(:, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';

    % Draw desired trajectory
    scatter3(traj(:, 1, 1), traj(:, 2, 1), traj(:, 3, 1), "red", 'Marker', '.'); hold on

    % Draw agent body
    for i = 1:length(r)
        draw_agent_quad(P(r(i), :), R(i, :, :), 1 + 256 * t(r(i)) / t(end)); hold on
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
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.png'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.eps'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.fig'));
end

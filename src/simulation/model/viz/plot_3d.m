function plot_3d(t, r, P, CoP, traj, thrust, R, options)
    figure
    scatter3(P(:, 1), P(:, 2), P(:, 3), 40, t); hold on
    quiver3(CoP(:, 1), CoP(:, 2), CoP(:, 3), thrust(:, 1), thrust(:, 2), thrust(:, 3), 'magenta')
    
    % Draw coordinates of the agent
    q1 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 1), R(:, 2, 1), R(:, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 2), R(:, 2, 2), R(:, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 3), R(:, 2, 3), R(:, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';

    % Draw desired trajectory
    scatter3(traj(:, 1, 1), traj(:, 2, 1), traj(:, 3, 1), "red", 'Marker','.'); hold on
    
    % Draw agent body
    for i=1:length(r)
        draw_agent_quad(P(r(i), :), R(i, :, :), 1+256*t(r(i))/t(end)); hold on
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    colorbar
    set(gca,'DataAspectRatio',[1 1 1])
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.png'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.fig'));
end
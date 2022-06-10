function plot_animation(t, r, P, traj, options, R)
    figure('Position', [10 10 1200 1200])
    set(gca,'DataAspectRatio',[1 1 1])
    animation_name = strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.gif');

    scatter3(P(:, 1), P(:, 2), P(:, 3), 'Color', 'none'); hold on
    scatter3([0 max(P(:, 1))+2], [0 max(P(:, 2))+2], [0 max(P(:, 3))+2], 'Color', 'none'); hold on
    ss_traj = scatter3(traj(1, 1, 1), traj(1, 2, 1), traj(1, 3, 1), "red", 'Marker','.'); hold on
    ss_state = scatter3(P(1, 1), P(1, 2), P(1, 3), "blue"); hold on
    patch_obj = draw_agent_quad(P(r(1), :), R(r(1), :, :), 1+256*t(r(1))/t(end)); hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    for i=1:length(r)
        ss_traj.XData = traj(1:r(i), 1, 1);
        ss_traj.YData = traj(1:r(i), 2, 1);
        ss_traj.ZData = traj(1:r(i), 3, 1);

        ss_state.XData = P(1:r(i), 1);
        ss_state.YData = P(1:r(i), 2);
        ss_state.ZData = P(1:r(i), 3);

        draw_agent_quad_animation(patch_obj, P(r(i), :), R(r(i), :, :), 1+256*t(r(i))/t(end));
        
        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i == 1
            imwrite(imind,cm,animation_name,'gif', 'Loopcount',inf, 'DelayTime',0.1);
        else
            imwrite(imind,cm,animation_name,'gif', 'WriteMode', 'append', 'DelayTime',0.1);
        end
    end
end
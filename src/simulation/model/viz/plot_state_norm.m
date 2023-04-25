function plot_state_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    % Draw orientation
    figure('Position', [10 10 800 1000])
    subplot(4, 1, 1);
    plot(t, vecnorm(attitude_d - eulZXY, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('rad')
    xlabel('time')
    %ylim([-0.1 0.1])
    title('Norm of orientation error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    % Draw angular velocity
    subplot(4, 1, 2);
    plot(t, vecnorm(W - beta, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\Omega\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('rad/s')
    xlabel('time')
    %ylim([-0.2 0.2])
    title('Norm of angular velocity error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    % Draw position
    subplot(4, 1, 3);
    plot(t, vecnorm(traj(:, 1:3, 1) - P, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{p}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('m')
    xlabel('time')
    title('Norm of position error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    % Draw velocity
    subplot(4, 1, 4);
    plot(t, vecnorm(traj(:, 1:3, 2) - dP, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{v}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('m/s')
    xlabel('time')
    %ylim([-10 10])
    title('Norm of velocity error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    % Draw acceleration

    % bnorm_att = vecnorm(attitude_d - eulZXY, 2, 2);
    % bnorm_ang = vecnorm(W - beta, 2, 2);
    % bnorm_pos = vecnorm(traj(:, 1:3, 1) - P, 2, 2);
    % bnorm_vel = vecnorm(traj(:, 1:3, 2) - dP, 2, 2);
    %save('mine_norm_profile.mat', 'bnorm_att', 'bnorm_ang', 'bnorm_pos', 'bnorm_vel', 't')

    sgtitle('Error profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_norm.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_norm.fig'));
end

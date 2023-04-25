function plot_norm(t, dP, P, traj, eR, eOmega, theta1, theta_a, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');

    if (isempty(theta1)) n_subf = 5; else n_subf = 4; end

    % Draw orientation
    figure('Position', [10 10 800 1000])
    subplot(n_subf, 1, 1);
    plot(t, vecnorm(eR, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('rad')
    xlabel('time')
    %ylim([-0.1 0.1])
    title('Norm of orientation error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    % Draw angular velocity
    subplot(n_subf, 1, 2);
    plot(t, vecnorm(eOmega, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\Omega\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('rad/s')
    xlabel('time')
    %ylim([-0.2 0.2])
    title('Norm of angular velocity error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    % Draw position
    subplot(n_subf, 1, 3);
    plot(t, vecnorm(traj(:, 1:3, 1) - P, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{p}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('m')
    xlabel('time')
    title('Norm of position error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    % Draw velocity
    subplot(n_subf, 1, 4);
    plot(t, vecnorm(traj(:, 1:3, 2) - dP, 2, 2), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{v}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylabel('m/s')
    xlabel('time')
    %ylim([-10 10])
    title('Norm of velocity error')
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')

    % Draw acceleration
    if length(theta1) > 1
        subplot(n_subf, 1, 5);
        plot(t, vecnorm(theta1 - theta_a, 2, 2), 'DisplayName', '$$\Vert\tilde{\theta}_1\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
        ylabel('m/s^2')
        xlabel('time')
        %ylim([-2 2])
        title('Norm of estimation error')
        hl = legend('show');
        set(hl, 'Interpreter', 'latex')
    end

    sgtitle('Error profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_norm.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_norm.fig'));
end

function plot_error(t, P, dP, traj, eR, eOmega, tilde_mu, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    %% Draw orientation
    figure('Position', [10 10 800 1000])
    subplot(5, 1, 1);
    plot(t, eR(:, 1),'DisplayName','$$\mathbf{R}_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, eR(:, 2),'DisplayName','$$\mathbf{R}_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, eR(:, 3),'DisplayName','$$\mathbf{R}_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('rad')
    xlabel('time')
    ylim([-0.1 0.1])
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    %% Draw angular velocity
    subplot(5, 1, 2);
    plot(t, eOmega(:, 1),'DisplayName','$$\Omega_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, eOmega(:, 2),'DisplayName','$$\Omega_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, eOmega(:, 3),'DisplayName','$$\Omega_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('rad/s')
    xlabel('time')
    ylim([-0.2 0.2])
    title('Angular velocity')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    %% Draw position
    subplot(5, 1, 3);
    plot(t, traj(:, 1, 1) - P(:, 1),'DisplayName','$$P_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 1) - P(:, 2),'DisplayName','$$P_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 1) - P(:, 3),'DisplayName','$$P_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m')
    xlabel('time')
    title('Position')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    %% Draw velocity
    subplot(5, 1, 4);
    plot(t, traj(:, 1, 2) - dP(:, 1),'DisplayName','$$V_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 2) - dP(:, 2),'DisplayName','$$V_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 2) - dP(:, 3),'DisplayName','$$V_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s')
    xlabel('time')
    ylim([-10 10])
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    sgtitle('Error profile')

    %% Draw acceleration
    subplot(5, 1, 5);
    plot(t, tilde_mu(:, 1),'DisplayName','$$\mu_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, tilde_mu(:, 2),'DisplayName','$$\mu_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, tilde_mu(:, 3),'DisplayName','$$\mu_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s^2')
    xlabel('time')
    ylim([-2 2])
    title('Acceleration w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    sgtitle('Error profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_error.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_error.fig'));
end
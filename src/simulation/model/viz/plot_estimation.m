function plot_estimation(t, theta1, theta2, theta3, theta_a, theta_b, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    figure(7)
    subplot(3, 1, 1);
    plot(t, theta1(:, 1), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{1,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta1(:, 2), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{1,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta1(:, 3), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{1,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    
    plot(t, theta_a(:, 1), 'LineStyle','--', 'DisplayName','$$\theta_{a,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta_a(:, 2), 'LineStyle','--', 'DisplayName','$$\theta_{a,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta_a(:, 3), 'LineStyle','--', 'DisplayName','$$\theta_{a,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    title('\theta_1 estimation');
    
    subplot(3, 1, 2);
    plot(t, theta2(:, 1), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{2,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta2(:, 2), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{2,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta2(:, 3), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{2,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    
    plot(t, theta_a(:, 1), 'LineStyle','--', 'DisplayName','$$\theta_{a,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta_a(:, 2), 'LineStyle','--', 'DisplayName','$$\theta_{a,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta_a(:, 3), 'LineStyle','--', 'DisplayName','$$\theta_{a,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    title('\theta_2 estimation');
    
    subplot(3, 1, 3);
    plot(t, theta3(:, 1), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{3,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta3(:, 2), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{3,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta3(:, 3), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{3,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    
    plot(t, theta_b(:, 1), 'LineStyle','--', 'DisplayName','$$\theta_{b,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta_b(:, 2), 'LineStyle','--', 'DisplayName','$$\theta_{b,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta_b(:, 3), 'LineStyle','--', 'DisplayName','$$\theta_{b,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    title('\theta_3 estimation');
    sgtitle('Estimation profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_estimation.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_estimation.fig'));
end
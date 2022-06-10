function plot_command(t, Tf, u, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    %% Draw torque
    figure(6)
    yyaxis left
    plot(t, u(:, 1), 'LineStyle','-', 'DisplayName','u_x', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, u(:, 2), 'LineStyle','-', 'DisplayName','u_y', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, u(:, 3), 'LineStyle','-', 'DisplayName','u_z', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('Nm')
    ylim([-10 4])
    
    yyaxis right
    plot(t, Tf, 'LineStyle','-', 'DisplayName','u_t', 'Color', '#000000','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    ylim([8 16])
    title('Command profile')
    legend()
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_command.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_command.fig'));
end
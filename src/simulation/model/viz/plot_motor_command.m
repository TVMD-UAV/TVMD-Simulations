function plot_motor_command(t, w_m1, w_m2, xi, eta, xi_d, eta_d, options)
    figure
    subplot(2, 1, 1);
    plot(t, w_m1,'DisplayName','$$w_{m1}$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, w_m2,'DisplayName','$$w_{m2}$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    ylabel('rot/s')
    xlabel('time')
    title('Propeller speed')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    legend()

    subplot(2, 1, 2);
    plot(t, xi,'DisplayName','$$\xi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, eta,'DisplayName','$$\eta$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, xi_d,'DisplayName','$$\xi_d$$','LineWidth',2, 'LineStyle','--', 'Color', '#0072BD'); hold on 
    plot(t, eta_d,'DisplayName','$$\eta_d$$','LineWidth',2, 'LineStyle','--', 'Color', '#D95319'); hold on 
    ylabel('rad')
    xlabel('time')
    title('Servo angle')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    legend()

    sgtitle('Command profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_command.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_command.fig'));
end
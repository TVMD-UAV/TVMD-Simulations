function plot_torque(t, B_M_f, B_M_d, B_M_g, B_M_a, options)
    figure
    plot(t, B_M_f(:, 1),  'r^','DisplayName','M_f: x'); hold on 
    plot(t, B_M_f(:, 2),  'r-','DisplayName','M_f: y'); hold on 
    plot(t, B_M_f(:, 3),  'r+','DisplayName','M_f: z'); hold on  
    plot(t, B_M_d(:, 1), 'g^','DisplayName','M_d: x'); hold on 
    plot(t, B_M_d(:, 2), 'g-','DisplayName','M_d: y'); hold on 
    plot(t, B_M_d(:, 3), 'g+','DisplayName','M_d: z'); hold on
    plot(t, B_M_g(:, 1), 'b^','DisplayName','M_g: x'); hold on 
    plot(t, B_M_g(:, 2), 'b-','DisplayName','M_g: y'); hold on 
    plot(t, B_M_g(:, 3), 'b+','DisplayName','M_g: z'); hold on
    plot(t, B_M_a(:, 1), 'c^','DisplayName','M_a: x'); hold on 
    plot(t, B_M_a(:, 2), 'c-','DisplayName','M_a: y'); hold on 
    plot(t, B_M_a(:, 3), 'c+','DisplayName','M_a: z'); hold on
    ylabel('Nm')
    xlabel('time')
    title('Torque profile')
    legend()
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.fig'));
end
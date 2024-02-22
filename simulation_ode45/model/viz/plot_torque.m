function plot_torque(t, B_M_f, B_M_d, B_M_g, B_M_a, options)
    figure
    plot(t, B_M_f(:, 1), 'r-'); hold on 
    plot(t, B_M_f(:, 2), 'r-'); hold on 
    plot(t, B_M_f(:, 3), 'r-'); hold on  
    plot(t, B_M_d(:, 1), 'g-'); hold on 
    plot(t, B_M_d(:, 2), 'g-'); hold on 
    plot(t, B_M_d(:, 3), 'g-'); hold on
    plot(t, B_M_g(:, 1), 'b-'); hold on 
    plot(t, B_M_g(:, 2), 'b-'); hold on 
    plot(t, B_M_g(:, 3), 'b-'); hold on
    plot(t, B_M_a(:, 1), 'c-'); hold on 
    plot(t, B_M_a(:, 2), 'c-'); hold on 
    plot(t, B_M_a(:, 3), 'c-'); hold on

    interval = ceil(length(t) / 40); 
    delta = ceil(interval / 4);
    rr = 1:interval:length(t)-interval;
    s1 =  scatter(t(rr)        , B_M_f(rr        , 1), 'r^','DisplayName','M_f: x', 'LineWidth',1.5); hold on 
    s2 =  scatter(t(rr)        , B_M_f(rr        , 2), 'r' ,'DisplayName','M_f: y', 'LineWidth',1.5); hold on 
    s3 =  scatter(t(rr)        , B_M_f(rr        , 3), 'r+','DisplayName','M_f: z', 'LineWidth',1.5); hold on  
    s4 =  scatter(t(rr+delta)  , B_M_d(rr+delta  , 1), 'g^','DisplayName','M_d: x', 'LineWidth',1.5); hold on 
    s5 =  scatter(t(rr+delta)  , B_M_d(rr+delta  , 2), 'g' ,'DisplayName','M_d: y', 'LineWidth',1.5); hold on 
    s6 =  scatter(t(rr+delta)  , B_M_d(rr+delta  , 3), 'g+','DisplayName','M_d: z', 'LineWidth',1.5); hold on
    s7 =  scatter(t(rr+delta*2), B_M_g(rr+delta*2, 1), 'b^','DisplayName','M_g: x', 'LineWidth',1.5); hold on 
    s8 =  scatter(t(rr+delta*2), B_M_g(rr+delta*2, 2), 'b' ,'DisplayName','M_g: y', 'LineWidth',1.5); hold on 
    s9 =  scatter(t(rr+delta*2), B_M_g(rr+delta*2, 3), 'b+','DisplayName','M_g: z', 'LineWidth',1.5); hold on
    s10 = scatter(t(rr+delta*3), B_M_a(rr+delta*3, 1), 'c^','DisplayName','M_a: x', 'LineWidth',1.5); hold on 
    s11 = scatter(t(rr+delta*3), B_M_a(rr+delta*3, 2), 'c' ,'DisplayName','M_a: y', 'LineWidth',1.5); hold on 
    s12 = scatter(t(rr+delta*3), B_M_a(rr+delta*3, 3), 'c+','DisplayName','M_a: z', 'LineWidth',1.5); hold on
    
    ylabel('Nm')
    xlabel('time')
    title('Torque profile')
    legend([s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12])
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.fig'));
end
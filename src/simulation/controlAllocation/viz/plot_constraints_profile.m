function plot_constraints_profile(conf, a, b, F, t, dt, options)
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    r_sigma_a = conf('r_sigma_a');
    r_sigma_b = conf('r_sigma_b');
    f_max = conf('f_max');
    r_f = conf('r_f');

    n = length(psi);
    n_sample = length(F(:, 1));
    xx = [t(1); t(end)];
    yy_f_lb = [0; 0];
    yy_f_ub = [f_max; f_max];
    yy_a_lb =- [sigma_a; sigma_a];
    yy_a_ub = [sigma_a; sigma_a];
    yy_b_lb =- [sigma_b; sigma_b];
    yy_b_ub = [sigma_b; sigma_b];

    figure
    subplot(3, 1, 1);

    for i = 1:n
        scatter(t, F(:, i), [], ones([n_sample 1]) * i, 'Marker', '.'); hold on
    end

    plot(xx, yy_f_ub, 'Color', '#BD0000', 'LineStyle', '--');
    plot(xx, yy_f_lb, 'Color', '#0000BD', 'LineStyle', '--');
    ylabel('$T_f$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 2);

    for i = 1:n
        scatter(t, a(:, i), [], ones([n_sample 1]) * i, 'Marker', '.'); hold on
    end

    plot(xx, yy_a_ub, 'Color', '#BD0000', 'LineStyle', '--');
    plot(xx, yy_a_lb, 'Color', '#0000BD', 'LineStyle', '--');
    ylabel('$\alpha$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 3);

    for i = 1:n
        scatter(t, b(:, i), [], ones([n_sample 1]) * i, 'Marker', '.'); hold on
    end

    plot(xx, yy_b_ub, 'Color', '#BD0000', 'LineStyle', '--');
    plot(xx, yy_b_lb, 'Color', '#0000BD', 'LineStyle', '--');
    ylabel('$\beta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time (sec)', 'FontName', 'Times New Roman', 'FontSize', 12);
    caxis([1, n])
    sgtitle('$X$', 'interpreter', 'latex')

    colorbar('Position', [0.93 0.168 0.022 0.7]);
    colormap jet
    caxis([1, n])

    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_cons.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_cons.fig'));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xx = [t(1); t(end)];
    yy_f_lb =- [r_f; r_f];
    yy_f_ub = [r_f; r_f];
    yy_a_lb =- [r_sigma_a; r_sigma_a];
    yy_a_ub = [r_sigma_a; r_sigma_a];
    yy_b_lb =- [r_sigma_b; r_sigma_b];
    yy_b_ub = [r_sigma_b; r_sigma_b];

    figure
    subplot(3, 1, 1);

    for i = 1:n
        scatter(t(2:end), (F(2:end, i) - F(1:end - 1, i)) ./ (t(2:end) - t(1:end - 1)), [], ones([n_sample - 1 1]) * i, 'Marker', '.'); hold on
    end

    plot(xx, yy_f_ub, 'Color', '#BD0000', 'LineStyle', '--');
    plot(xx, yy_f_lb, 'Color', '#0000BD', 'LineStyle', '--');
    ylabel('$\Delta T_f$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 2);

    for i = 1:n
        scatter(t(2:end), (a(2:end, i) - a(1:end - 1, i)) ./ (t(2:end) - t(1:end - 1)), [], ones([n_sample - 1 1]) * i, 'Marker', '.'); hold on
    end

    plot(xx, yy_a_ub, 'Color', '#BD0000', 'LineStyle', '--');
    plot(xx, yy_a_lb, 'Color', '#0000BD', 'LineStyle', '--');
    ylabel('$\Delta \alpha$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 3);

    for i = 1:n
        scatter(t(2:end), (b(2:end, i) - b(1:end - 1, i)) ./ (t(2:end) - t(1:end - 1)), [], ones([n_sample - 1 1]) * i, 'Marker', '.'); hold on
    end

    plot(xx, yy_b_ub, 'Color', '#BD0000', 'LineStyle', '--');
    plot(xx, yy_b_lb, 'Color', '#0000BD', 'LineStyle', '--');
    ylabel('$\Delta \beta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time (sec)', 'FontName', 'Times New Roman', 'FontSize', 12);
    caxis([1, n])
    sgtitle('$\Delta X$', 'interpreter', 'latex')

    colorbar('Position', [0.93 0.168 0.022 0.7]);
    colormap jet
    caxis([1, n])

    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_rcons.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_rcons.fig'));
end

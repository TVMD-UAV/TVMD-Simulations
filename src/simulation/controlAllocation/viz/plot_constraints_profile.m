function plot_constraints_profile(conf, a, b, F, t, dt)
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    r_sigma_a = conf('r_sigma_a');
    r_sigma_b = conf('r_sigma_b');
    f_max = conf('f_max');
    r_f = conf('r_f');

    n = length(psi);
    n_sample = length(F(1, :));
    xx = reshape([1 1]' * t + [-0.5 0.5]', [2 * n_sample 1]);
    yy_f_lb = -f_max * ones([2 * n_sample 1]);
    yy_f_ub = f_max * ones([2 * n_sample 1]);
    yy_a_lb = -sigma_a * ones([2 * n_sample 1]);
    yy_a_ub = sigma_a * ones([2 * n_sample 1]);
    yy_b_lb = -sigma_b * ones([2 * n_sample 1]);
    yy_b_ub = sigma_b * ones([2 * n_sample 1]);

    figure
    subplot(3, 1, 1);

    for i = 1:n
        scatter(t, F(i, :), [], ones([n_sample 1]) * i); hold on
    end

    plot(xx, yy_f_ub, 'Color', '#BD0000');
    plot(xx, yy_f_lb, 'Color', '#0000BD');
    ylabel('$T_f$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 2);

    for i = 1:n
        scatter(t, a(i, :), [], ones([n_sample 1]) * i); hold on
    end

    plot(xx, yy_a_ub, 'Color', '#BD0000');
    plot(xx, yy_a_lb, 'Color', '#0000BD');
    ylabel('$\alpha$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 3);

    for i = 1:n
        scatter(t, b(i, :), [], ones([n_sample 1]) * i); hold on
    end

    plot(xx, yy_b_ub, 'Color', '#BD0000');
    plot(xx, yy_b_lb, 'Color', '#0000BD');
    ylabel('$\beta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time (sec)', 'FontName', 'Times New Roman', 'FontSize', 12);
    caxis([1, n])
    sgtitle('$X$', 'interpreter', 'latex')

    colorbar('Position', [0.93 0.168 0.022 0.7]);
    colormap jet
    caxis([1, n])

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dx_lb = -dt * (ones([1 n_sample]) .* [r_f; r_sigma_a; r_sigma_b])';
    dx_ub = dt * (ones([1 n_sample]) .* [r_f; r_sigma_a; r_sigma_b])';
    yy_f_lb = reshape([1 1]' * dx_lb(:, 1)', [2 * n_sample 1]);
    yy_f_ub = reshape([1 1]' * dx_ub(:, 1)', [2 * n_sample 1]);
    yy_a_lb = reshape([1 1]' * dx_lb(:, 2)', [2 * n_sample 1]);
    yy_a_ub = reshape([1 1]' * dx_ub(:, 2)', [2 * n_sample 1]);
    yy_b_lb = reshape([1 1]' * dx_lb(:, 3)', [2 * n_sample 1]);
    yy_b_ub = reshape([1 1]' * dx_ub(:, 3)', [2 * n_sample 1]);

    figure
    subplot(3, 1, 1);

    for i = 1:n
        scatter(t(2:end), F(i, 2:end) - F(i, 1:end - 1), [], ones([n_sample - 1 1]) * i); hold on
    end

    plot(xx, yy_f_ub, 'Color', '#BD0000');
    plot(xx, yy_f_lb, 'Color', '#0000BD');
    ylabel('$\Delta T_f$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 2);

    for i = 1:n
        scatter(t(2:end), a(i, 2:end) - a(i, 1:end - 1), [], ones([n_sample - 1 1]) * i); hold on
    end

    plot(xx, yy_a_ub, 'Color', '#BD0000');
    plot(xx, yy_a_lb, 'Color', '#0000BD');
    ylabel('$\Delta \alpha$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    caxis([1, n])

    subplot(3, 1, 3);

    for i = 1:n
        scatter(t(2:end), b(i, 2:end) - b(i, 1:end - 1), [], ones([n_sample - 1 1]) * i); hold on
    end

    plot(xx, yy_b_ub, 'Color', '#BD0000');
    plot(xx, yy_b_lb, 'Color', '#0000BD');
    ylabel('$\Delta \beta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time (sec)', 'FontName', 'Times New Roman', 'FontSize', 12);
    caxis([1, n])
    sgtitle('$\Delta X$', 'interpreter', 'latex')

    colorbar('Position', [0.93 0.168 0.022 0.7]);
    colormap jet
    caxis([1, n])
end

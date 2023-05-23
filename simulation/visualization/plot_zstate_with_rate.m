function plot_zstate_with_rate(n, lower, upper, rlower, rupper, t, z_d, ts, z, dz, name, label_state, label_rate, options, show_bound, pos_offset, savename)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    % Bounds
    xx = [t(1); t(end)];

    figure('Position', [1410+pos_offset(1) 10+pos_offset(2) 400 300])

    % State
    subplot(2, 1, 1);
    if show_bound
        plot(xx, [lower; lower], 'Color', '#333333', 'LineStyle', '--', 'HandleVisibility','off'); hold on
        plot(xx, [upper; upper], 'Color', '#333333', 'LineStyle', '--', 'DisplayName', 'Bounds'); hold on
    end
    for i = 1:n
        zs = interp1(ts, z(:, i), t);
        plot(t, zs, 'LineStyle', '-', 'LineWidth', 2, 'DisplayName', 'State', 'Color', '#0072BD'); hold on
        plot(t, z_d(i, :), 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Desired', 'Color', '#D95319'); hold on
    end

    ylabel(label_state, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    legend('show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'Orientation','horizontal');
    if n > 1; caxis([1, n]); end

    % Desired
    subplot(2, 1, 2);
    if show_bound
        plot(xx, [rlower; rlower], 'Color', '#333333', 'LineStyle', '--', 'HandleVisibility','off'); hold on
        plot(xx, [rupper; rupper], 'Color', '#333333', 'LineStyle', '--', 'DisplayName', 'Bounds'); hold on
    end
    for i = 1:n
        zds = interp1(ts, dz(:, i), t);
        plot(t, zds, 'LineStyle', '-', 'LineWidth', 2, 'Color', '#0072BD'); hold on
    end

    ylabel(label_rate, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    if n > 1; caxis([1, n]); end

    sgtitle(name, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    
    if n > 1
        colorbar('Position', [0.93 0.168 0.022 0.7]);
        colormap jet
        caxis([1, n]); 
    end

    savefig_helper(options, savename);
end

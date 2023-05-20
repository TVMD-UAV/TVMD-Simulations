function plot_constraints_profile_with_rates(n, lower, upper, t, z_d, ts, z, name, options, show_bound, pos_offset, savename)
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
        plot(xx, [lower; lower], 'Color', '#333333', 'LineStyle', '--'); hold on
        plot(xx, [upper; upper], 'Color', '#333333', 'LineStyle', '--'); hold on
    end
    for i = 1:n
        scatter(ts, z(:, i), [], ones([length(ts) 1]) * i, 'Marker', '.'); hold on
    end

    ylabel('State', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    if n > 1; caxis([1, n]); end

    % Desired
    subplot(2, 1, 2);
    if show_bound
        plot(xx, [lower; lower], 'Color', '#333333', 'LineStyle', '--'); hold on
        plot(xx, [upper; upper], 'Color', '#333333', 'LineStyle', '--'); hold on
    end
    for i = 1:n
        scatter(t, z_d(i, :), [], ones([length(t) 1]) * i, 'Marker', '.'); hold on
    end

    ylabel('Desired', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    if n > 1; caxis([1, n]); end

    sgtitle(name, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    
    if n > 1
        colorbar('Position', [0.93 0.168 0.022 0.7]);
        colormap jet
        caxis([1, n]); 
    end

    savefig_helper(options, savename);
end

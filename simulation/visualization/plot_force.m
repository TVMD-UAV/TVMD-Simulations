function plot_force(t, B_M, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    % B_M_f, B_M_d, B_M_g, B_M_a, B_M_delta
    color = ["r", "g", "b", "c", "m"];
    marker = ["^", "o", "x"];
    labels = ["$$\mathbf{f}_x$$ (Nm)", "$$\mathbf{f}_y$$ (Nm)", "$$\mathbf{f}_z$$ (Nm)"];
    torque_name = ["$$\mathbf{f}_T$$", "$$\mathbf{f}_G$$", "$$\mathbf{f}_I$$", "$$\mathbf{f}_E$$", "$$M_\Delta$$"];

    interval = ceil(length(t) / 10); 
    delta = ceil(interval / 5);

    figure('Position', [1210 10 400 500])
    tiledlayout(3,1, 'TileSpacing', 'compact');
    for k=1:3
        % subplot(3, 1, k)
        nexttile
        for i=1:size(B_M, 2)
            M = squeeze(B_M(:, i, :));
            plot(t, M(k+3, :), "color", color(i), "Marker", marker(k), 'MarkerIndices', delta*i:interval:length(t), ...
                'DisplayName', torque_name(i)); hold on 
        end
        labely = ylabel(labels(k), 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
        labely.Position(1) = labely_pos;
    end
    legend(nexttile(3), 'show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, ...
        'NumColumns', 5, 'Orientation','horizontal', 'Location', 'southoutside');

    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    sgtitle('Force profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    options('savepdf') = false;
    savefig_helper(options, '_force');
end
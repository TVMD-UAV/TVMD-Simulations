function plot_torque(t, B_M, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    % B_M_f, B_M_d, B_M_g, B_M_a, B_M_delta
    color = ["r", "g", "b", "c", "m"];
    marker = ["^", "o", "x"];
    labels = ["$$M_x$$ (Nm)", "$$M_y$$ (Nm)", "$$M_z$$ (Nm)"];
    torque_name = ["$$M_F$$", "$$M_D$$", "$$M_G$$", "$$M_A$$", "$$M_\Delta$$"];
    torque_name = ["$$M_F$$", "$$M_G$$", "$$M_I$$", "$$M_E$$", "$$M_\Delta$$"];

    interval = ceil(length(t) / 10); 
    delta = ceil(interval / 5);

    figure('Position', [1210 10 400 500])
    tiledlayout(3,1, 'TileSpacing', 'compact');
    for k=1:3
        % subplot(3, 1, k)
        nexttile
        for i=1:size(B_M, 2)
            M = squeeze(B_M(:, i, :));
            plot(t, M(k, :), "color", color(i), "Marker", marker(k), 'MarkerIndices', delta*i:interval:length(t), ...
                'DisplayName', torque_name(i)); hold on 
        end
        labely = ylabel(labels(k), 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
        labely.Position(1) = labely_pos;
    end
    legend(nexttile(3), 'show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, ...
        'NumColumns', 5, 'Orientation','horizontal', 'Location', 'southoutside');

    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    sgtitle('Moment profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    options('savepdf') = false;
    savefig_helper(options, '_torque');
end
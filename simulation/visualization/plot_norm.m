function plot_norm(t, eX, eV, eR, eOmega, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    labely_pos = options('labely_pos');

    % Draw orientation
    figure('Position', [10 10 400 400])
    subplot(4, 1, 1);
    % plot(t, vecnorm(eR, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, eR, 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Psi(\mathbf{R})$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw angular velocity
    subplot(4, 1, 2);
    plot(t, vecnorm(eOmega, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\Omega\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\Omega\Vert_2$$ (rad/s)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw position
    subplot(4, 1, 3);
    plot(t, vecnorm(eX, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{p}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{X}\Vert_2$$ (m)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw velocity
    subplot(4, 1, 4);
    plot(t, vecnorm(eV, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{v}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{v}\Vert_2$$ (m/s)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    
    sgtitle('Error Profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    savefig_helper(options, '_norm');
end

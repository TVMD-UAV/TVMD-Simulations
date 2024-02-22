function plot_lyapunov_candidates(t, eX, eV, eR, eOmega, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    labely_pos = options('labely_pos');

    % Draw orientation
    figure('Position', [10 10 400 400])
    subplot(3, 1, 1);
    % plot(t, vecnorm(eR, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, eR, 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Psi(\mathbf{R}, \mathbf{R}_d)$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw angular velocity
    subplot(3, 1, 2);
    plot(t, vecnorm(eOmega, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\Omega\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\Omega\Vert_{\mathbf{K}_\Omega}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    subplot(3, 1, 3);
    plot(t, eR + vecnorm(eOmega, 2, 1), 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Psi + \Vert\mathbf{e}_\Omega\Vert_{\mathbf{K}_\Omega}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    savefig_helper(options, '_lyapunov_candidates_attitude');
    
    % Draw position
    figure('Position', [10 410 400 400])
    subplot(3, 1, 1);
    plot(t, vecnorm(eX, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{x}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{x}\Vert_{\mathbf{K}_\mathbf{x}}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw velocity
    subplot(3, 1, 2);
    plot(t, vecnorm(eV, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{v}\Vert_{\mathbf{K}_\mathbf{v}}$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{v}\Vert_{\mathbf{K}_\mathbf{v}}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    subplot(3, 1, 3);
    plot(t, vecnorm(eX+eV, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{v}\Vert_{\mathbf{K}_\mathbf{v}}$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{x}\Vert_{\mathbf{K}_\mathbf{x}} + \Vert\mathbf{e}_\mathbf{v}\Vert_{\mathbf{K}_\mathbf{v}}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    
    sgtitle('Lyapunov Candidates', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    savefig_helper(options, '_lyapunov_candidates_position');
end

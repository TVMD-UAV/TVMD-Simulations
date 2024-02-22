%% ===================================
%
% Plot forces profile
% t: T x 1 vector
% u: request desired vector, 6 x T matrix
% vecs: output vector, 6 x T matrix
%
%% ===================================
function plot_wrench(t, u, u_d, u_f_r, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    figure('Position', [810 10 400 500])
    % subplot(2, 1, 1);
    tile = tiledlayout(6,1);
    ax1 = nexttile;
    % subplot(6, 1, 1);
    plot(ax1, t, u_d(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
    plot(ax1, t, u(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    if ~isempty(u_f_r); plot(ax1, t, u_f_r(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', ':', 'Color', '#0072BD');    end
    % ylim([-40, -20])
    labely = ylabel(ax1, "${\mathbf{u}_f}_x$", 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    % title('Forces', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    ax2 = nexttile;
    % subplot(6, 1, 2);
    plot(ax2, t, u_d(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
    plot(ax2, t, u(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    if ~isempty(u_f_r); plot(ax2, t, u_f_r(2, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', ':', 'Color', '#D95319');    end
    % ylim([-12, 17])
    labely = ylabel(ax2, '${\mathbf{u}_f}_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    ax3 = nexttile;
    % subplot(6, 1, 3);
    plot(ax3, t, u_d(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120'); hold on
    plot(ax3, t, u(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    if ~isempty(u_f_r); plot(ax3, t, u_f_r(3, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', ':', 'Color', '#EDB120');    end
    % ylim([40, 70])
    labely = ylabel(ax3, '${\mathbf{u}_f}_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % subplot(2, 1, 2);
    % t = tiledlayout(3,1);
    ax4 = nexttile;
    % subplot(6, 1, 4);
    plot(ax4, t, u_d(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
    plot(ax4, t, u(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    % if ~isempty(u_f_r); plot(ax4, t, u_f_r(4, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', ':', 'Color', '#0072BD');    end
    % ylim([-2, 2]);
    labely = ylabel(ax4, '${\mathbf{u}_\tau}_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    % title('Torques', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    ax5 = nexttile;
    % subplot(6, 1, 5);
    plot(ax5, t, u_d(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
    plot(ax5, t, u(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    % if ~isempty(u_f_r); plot(ax5, t, u_f_r(5, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', ':', 'Color', '#D95319');    end
    % ylim([-2, 2]);
    labely = ylabel(ax5, '${\mathbf{u}_\tau}_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    ax6 = nexttile;
    % subplot(6, 1, 6);
    plot(ax6, t, u_d(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120'); hold on
    plot(ax6, t, u(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    % if ~isempty(u_f_r); plot(ax6, t, u_f_r(6, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', ':', 'Color', '#EDB120');     end
    % ylim([-2, 2]);
    labely = ylabel(ax6, '${\mathbf{u}_\tau}_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % xticklabels(ax1,{})
    % xticklabels(ax2,{})
    % xticklabels(ax3,{})
    % xticklabels(ax4,{})
    % xticklabels(ax5,{})
    % linkaxes([ax1, ax1, ax3, ax4, ax5, ax6], 'x');
    pp(1) = plot(ax6, nan, nan, 'k:', 'DisplayName', '$$\mathbf{u}_r$$', 'LineWidth', 2); hold on
    pp(2) = plot(ax6, nan, nan, 'k--', 'DisplayName', '$$\mathbf{u}_{d}$$', 'LineWidth', 2);
    pp(3) = plot(ax6, nan, nan, 'k-', 'DisplayName', '$$\mathbf{u}_{zd}$$', 'LineWidth', 2);

    tile.TileSpacing = 'tight';
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    
    legend(nexttile(6), pp, 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, ...
        'NumColumns', 5, 'Orientation','horizontal', 'Location', 'southoutside');
    sgtitle('Wrench Allocation', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size, 'FontWeight', 'bold');

    savefig_helper(options, '_wrench');

    plot_wrench_error(t, u_d, u, options)
end

function plot_wrench_error(t, u_d, u, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    figure('Position', [810 10 400 500])
    % subplot(2, 1, 1);
    tile = tiledlayout(6,1);
    ax1 = nexttile;
    u_e = u_d - u;
    plot(ax1, t, u_e(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel(ax1, "${\mathbf{u}_f}_x$", 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    % title('Forces Error', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    ax2 = nexttile;
    plot(ax2, t, u_e(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319'); hold on
    labely = ylabel(ax2, '${\mathbf{u}_f}_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    ax3 = nexttile;
    plot(ax3, t, u_e(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120'); hold on
    labely = ylabel(ax3, '${\mathbf{u}_f}_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % subplot(2, 1, 2);
    % t = tiledlayout(3,1);
    ax4 = nexttile;
    plot(ax4, t, u_e(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    ylim([-2, 2]);
    labely = ylabel(ax4, '${\mathbf{u}_\tau}_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    % title('Torques Error', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    ax5 = nexttile;
    plot(ax5, t, u_e(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319'); hold on
    ylim([-2, 2]);
    labely = ylabel(ax5, '${\mathbf{u}_\tau}_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    ax6 = nexttile;
    plot(ax6, t, u_e(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120'); hold on
    ylim([-2, 2]);
    labely = ylabel(ax6, '${\mathbf{u}_\tau}_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % xticklabels(ax1,{})
    % xticklabels(ax2,{})
    % xticklabels(ax3,{})
    % xticklabels(ax4,{})
    % xticklabels(ax5,{})
    tile.TileSpacing = 'tight';
    linkaxes([ax1, ax1, ax3, ax4, ax5, ax6], 'x');
    
    xlabel(tile, '$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    sgtitle('Wrench Allocation Error', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size, 'FontWeight', 'bold');

    savefig_helper(options, '_wrench_error');
end

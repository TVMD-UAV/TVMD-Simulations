%% ===================================
%
% Plot forces profile
% t: T x 1 vector
% u: request desired vector, 6 x T matrix
% vecs: output vector, 6 x T matrix
%
%% ===================================
function plot_wrench(t, u_d, u, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    figure('Position', [910 10 500 600])
    subplot(6, 1, 1);
    plot(t, u_d(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
    plot(t, u(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    labely = ylabel("${\mathbf{u}_t}_x$ (N)", 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    title('Forces', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    subplot(6, 1, 2);
    plot(t, u_d(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
    plot(t, u(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    labely = ylabel('${\mathbf{u}_t}_y (N)$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(6, 1, 3);
    plot(t, u_d(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120'); hold on
    plot(t, u(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    labely = ylabel('${\mathbf{u}_t}_z$ (N)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(6, 1, 4);
    plot(t, u_d(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
    plot(t, u(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    labely = ylabel('${\mathbf{u}_\tau}_x$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    title('Moments', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    subplot(6, 1, 5);
    plot(t, u_d(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
    plot(t, u(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    labely = ylabel('${\mathbf{u}_\tau}_y$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(6, 1, 6);
    plot(t, u_d(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120'); hold on
    plot(t, u(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    labely = ylabel('${\mathbf{u}_\tau}_z$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    savefig_helper(options, '_wrench');
end

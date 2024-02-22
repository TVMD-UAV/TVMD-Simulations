%% ===================================
%
% Plot forces profile
% t: T x 1 vector
% u: request desired vector, 6 x T matrix
% vecs: output vector, 6 x T matrix
%
%% ===================================
function plot_output_profile(t, u, vecs, options)
    figure('Position', [10 10 800 800])
    subplot(6, 1, 1);
    plot(t, u(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', 'o'); hold on
    plot(t, vecs(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    ylabel('$F_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    title('Forces')

    subplot(6, 1, 2);
    plot(t, u(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319', 'marker', 'o'); hold on
    plot(t, vecs(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    ylabel('$F_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 3);
    plot(t, u(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120', 'marker', 'o'); hold on
    plot(t, vecs(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    ylabel('$F_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 4);
    plot(t, u(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', 'o'); hold on
    plot(t, vecs(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    ylabel('$M_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    title('Moments')

    subplot(6, 1, 5);
    plot(t, u(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319', 'marker', 'o'); hold on
    plot(t, vecs(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    ylabel('$M_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 6);
    plot(t, u(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120', 'marker', 'o'); hold on
    plot(t, vecs(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    ylabel('$M_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_output.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_output.fig'));
end

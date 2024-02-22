%% ===================================
%
% Plot moments profile
% t: T x 1 vector
% u: request desired vector, 6 x T matrix
% vecs: output vector, 6 x T matrix
%
%% ===================================
function plot_error_profile(t, u, vecs, options)
    figure('Position', [10 10 800 800])
    subplot(6, 1, 1);
    plot(t, (vecs(1, :) - u(1, :)), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', 'o');
    ylabel('$F_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    title('Forces Error')

    subplot(6, 1, 2);
    plot(t, (vecs(2, :) - u(2, :)), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319', 'marker', 'o');
    ylabel('$F_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 3);
    plot(t, (vecs(3, :) - u(3, :)), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120', 'marker', 'o');
    ylabel('$F_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 4);
    plot(t, (vecs(4, :) - u(4, :)), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', 'o');
    ylabel('$M_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    title('Moments Error')

    subplot(6, 1, 5);
    plot(t, (vecs(5, :) - u(5, :)), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319', 'marker', 'o');
    ylabel('$M_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 6);
    plot(t, (vecs(6, :) - u(6, :)), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120', 'marker', 'o');
    ylabel('$M_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_error.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_error.fig'));
end

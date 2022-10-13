%% ===================================
%
% Plot metrics
% t: T x 1 vector
% te: thrust efficiency
% ef: force error
% em: moment error
%
%% ===================================
function plot_metrics(t, te, ef, em, df, dm, options)
    figure
    subplot(5, 1, 1);
    plot(t, te, 'DisplayName', 'Thrust efficiency', 'LineWidth', 2, 'LineStyle', '-');
    title('Thrust efficiency', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(5, 1, 2);
    plot(t, ef, 'DisplayName', 'Forces error', 'LineWidth', 2, 'LineStyle', '-');
    title('Force error', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(5, 1, 3);
    plot(t, em, 'DisplayName', 'Moments error', 'LineWidth', 2, 'LineStyle', '-');
    title('Moment error', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(5, 1, 4);
    plot(t, df, 'DisplayName', 'Forces alignment', 'LineWidth', 2, 'LineStyle', '-');
    title('Forces alignment', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(5, 1, 5);
    plot(t, dm, 'DisplayName', 'Moments alignment', 'LineWidth', 2, 'LineStyle', '-');
    title('Moments alignment', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('', 'FontName', 'Times New Roman', 'FontSize', 12)

    sgtitle('Metrics profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_metrics.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_metrics.fig'));
end

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
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    figure('Position', [410 10 400 600])
    subplot(5, 1, 1);
    plot(t, te, 'DisplayName', 'Thrust efficiency', 'LineWidth', 2, 'LineStyle', '-');
    title('Thrust efficiency', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    subplot(5, 1, 2);
    plot(t, ef, 'DisplayName', 'Forces error', 'LineWidth', 2, 'LineStyle', '-');
    title('Force error', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    subplot(5, 1, 3);
    plot(t, em, 'DisplayName', 'Moments error', 'LineWidth', 2, 'LineStyle', '-');
    title('Moment error', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    subplot(5, 1, 4);
    plot(t, df, 'DisplayName', 'Forces alignment', 'LineWidth', 2, 'LineStyle', '-');
    title('Forces alignment', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    ylabel('', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    subplot(5, 1, 5);
    plot(t, dm, 'DisplayName', 'Moments alignment', 'LineWidth', 2, 'LineStyle', '-');
    title('Moments alignment', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    ylabel('', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    sgtitle('Metrics Profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)
    savefig_helper(options, '_metrics');
end

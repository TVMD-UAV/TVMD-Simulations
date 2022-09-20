%% ===================================
%
% Plot metrics
% t: T x 1 vector
% te: thrust efficiency
% ef: force error
% em: moment error
%
%% ===================================
function plot_metrics(t, te, ef, em)
    figure
    subplot(3, 1, 1);
    plot(t, te, 'DisplayName', 'Thrust efficiency', 'LineWidth', 2, 'LineStyle', '--', 'marker', 'o');
    title('Thrust efficiency', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(3, 1, 2);
    plot(t, ef, 'DisplayName', 'Thrust efficiency', 'LineWidth', 2, 'LineStyle', '--', 'marker', 'o');
    title('Force error', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(3, 1, 3);
    plot(t, em, 'DisplayName', 'Thrust efficiency', 'LineWidth', 2, 'LineStyle', '--', 'marker', 'o');
    title('Moment error', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('%', 'FontName', 'Times New Roman', 'FontSize', 12)
end

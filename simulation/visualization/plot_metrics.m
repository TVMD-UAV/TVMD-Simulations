%% ===================================
%
% Plot metrics
% t: T x 1 vector
% te: thrust efficiency
% ef: force error
% em: moment error
%
%% ===================================
function plot_metrics(t, metrics, incre, options)
    % [te, ef, em, df, dm];
    % metrices(5, :), metrices(1, :), metrices(2, :), metrices(3, :), metrices(4, :)
    te = metrics(5, :);
    ef = metrics(1, :);
    em = metrics(2, :);
    df = metrics(3, :);
    dm = metrics(4, :);
    incre_f = incre(1, :);
    incre_t = incre(2, :);

    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    figure('Position', [410 10 400 600])
    % subplot(4, 1, 1);
    tile = tiledlayout(4,1);
    nexttile;
    plot(t, te, 'DisplayName', 'Thrust efficiency', 'LineWidth', 2, 'LineStyle', '-');
    title('Thrust efficiency', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    labely = ylabel('%', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    % subplot(5, 1, 2);
    % plot(t, ef, 'DisplayName', 'Forces error', 'LineWidth', 2, 'LineStyle', '-');
    % title('Force error', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    % ylabel('%', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    % subplot(5, 1, 3);
    % plot(t, em, 'DisplayName', 'Moments error', 'LineWidth', 2, 'LineStyle', '-');
    % title('Moment error', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    % ylabel('%', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    % subplot(4, 1, 2);
    nexttile;
    plot(t, ef, 'DisplayName', 'Force', 'LineWidth', 2, 'LineStyle', '-'); hold on
    plot(t, em, 'DisplayName', 'Torque', 'LineWidth', 2, 'LineStyle', '-');
    labely = ylabel('%', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    title('Controls Error', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    % legend('show', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, ...
    %        'NumColumns', 2, 'Orientation','horizontal', 'Location', 'northeast')
    

    % subplot(5, 1, 4);
    % plot(t, df, 'DisplayName', 'Forces alignment', 'LineWidth', 2, 'LineStyle', '-');
    % title('Forces alignment', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    % ylabel('', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    % subplot(5, 1, 5);
    % plot(t, dm, 'DisplayName', 'Moments alignment', 'LineWidth', 2, 'LineStyle', '-');
    % title('Moments alignment', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    % ylabel('', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    % subplot(4, 1, 3);
    nexttile;
    plot(t, df, 'DisplayName', 'Force', 'LineWidth', 2, 'LineStyle', '-'); hold on
    plot(t, dm, 'DisplayName', 'Torque', 'LineWidth', 2, 'LineStyle', '-');
    labely = ylabel('$$\cos\theta$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    title('Controls Alignment', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    % subplot(4, 1, 4);
    nexttile;
    plot(t, incre_f, 'DisplayName', 'Force', 'LineWidth', 2, 'LineStyle', '-'); hold on
    plot(t, incre_t, 'DisplayName', 'Torque', 'LineWidth', 2, 'LineStyle', '-');
    % labely = ylabel('$$\cos\theta$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    % labely.Position(1) = labely_pos;
    title('Controls Increment', 'FontName', 'Times New Roman', 'FontSize', title_font_size)

    tile.TileSpacing = 'tight';
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    legend(nexttile(4),'show', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, ...
           'NumColumns', 2, 'Orientation','horizontal', 'Location', 'southoutside')
    

    sgtitle('Control Allocation Metrics Profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size, 'FontWeight', 'bold')
    savefig_helper(options, '_metrics');
end

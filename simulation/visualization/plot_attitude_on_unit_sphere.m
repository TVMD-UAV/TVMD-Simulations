function plot_attitude_on_unit_sphere(t, R_d, R, num_idx, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    figure('Position', [410 10 800 400])
    % r = 1:floor(size(R_d, 3) / 1000):size(R_d, 3);
    r = 1:size(R_d, 3);
    idx_len = length(r);
    idx = floor(idx_len / num_idx);
    [X,Y,Z] = sphere();
    subplot(1, 2, 1);
    surf(X,Y,Z, 'FaceColor', '#555555', 'EdgeColor', 'none', 'FaceAlpha',0.1, 'HandleVisibility','off'); hold on;

    plot3(squeeze(R_d(1, 1, r)), squeeze(R_d(2, 1, r)), squeeze(R_d(3, 1, r)),'LineWidth',2, 'LineStyle','--', 'Color', '#0072BD', 'DisplayName','$${\mathbf{r}_d}_x$$', 'Marker', 'o', 'MarkerIndices', 1:idx:idx_len); hold on 
    plot3(squeeze(R_d(1, 2, r)), squeeze(R_d(2, 2, r)), squeeze(R_d(3, 2, r)),'LineWidth',2, 'LineStyle','--', 'Color', '#D95319', 'DisplayName','$${\mathbf{r}_d}_y$$', 'Marker', 'o', 'MarkerIndices', ceil(1*idx/3):idx:idx_len); hold on 
    plot3(squeeze(R_d(1, 3, r)), squeeze(R_d(2, 3, r)), squeeze(R_d(3, 3, r)),'LineWidth',2, 'LineStyle','--', 'Color', '#EDB120', 'DisplayName','$${\mathbf{r}_d}_z$$', 'Marker', 'o', 'MarkerIndices', ceil(2*idx/3):idx:idx_len); hold on 

    scatter3(squeeze(R_d(1, 1, 1)), squeeze(R_d(2, 1, 1)), squeeze(R_d(3, 1, 1)), 80, 'black', 'Marker', 'x', 'HandleVisibility','off'); hold on 
    scatter3(squeeze(R_d(1, 2, 1)), squeeze(R_d(2, 2, 1)), squeeze(R_d(3, 2, 1)), 80, 'black', 'Marker', 'x', 'HandleVisibility','off'); hold on 
    scatter3(squeeze(R_d(1, 3, 1)), squeeze(R_d(2, 3, 1)), squeeze(R_d(3, 3, 1)), 80, 'black', 'Marker', 'x', 'HandleVisibility','off'); hold on 

    xlabel('$$x$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    ylabel('$$y$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    zlabel('$$z$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    legend('show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 3, 'Orientation','horizontal', 'Location', 'southoutside');
    grid on;
    title('$$\mathbf{R}_d$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    set(gca, 'DataAspectRatio', [1 1 1])

    r = 1:floor(size(R, 1) / 1000):size(R, 1);
    idx_len = length(r);
    idx = floor(idx_len / num_idx);
    subplot(1, 2, 2);
    surf(X,Y,Z, 'FaceColor', '#555555', 'EdgeColor', 'none', 'FaceAlpha',0.1, 'HandleVisibility','off'); hold on;

    plot3(R(r, 1, 1), R(r, 2, 1), R(r, 3, 1),'LineWidth',1.5, 'LineStyle','-', 'Color', '#0072BD', 'DisplayName','$${\mathbf{r}}_x$$', 'Marker', 'o', 'MarkerIndices', 1:idx:idx_len); hold on 
    plot3(R(r, 1, 2), R(r, 2, 2), R(r, 3, 2),'LineWidth',1.5, 'LineStyle','-', 'Color', '#D95319', 'DisplayName','$${\mathbf{r}}_y$$', 'Marker', 'o', 'MarkerIndices', ceil(1*idx/3):idx:idx_len); hold on 
    plot3(R(r, 1, 3), R(r, 2, 3), R(r, 3, 3),'LineWidth',1.5, 'LineStyle','-', 'Color', '#EDB120', 'DisplayName','$${\mathbf{r}}_z$$', 'Marker', 'o', 'MarkerIndices', ceil(2*idx/3):idx:idx_len); hold on 

    scatter3(squeeze(R(1, 1, 1)), squeeze(R(1, 2, 1)), squeeze(R(1, 3, 1)), 80, 'black', 'Marker', 'x', 'HandleVisibility','off'); hold on 
    scatter3(squeeze(R(1, 1, 2)), squeeze(R(1, 2, 2)), squeeze(R(1, 3, 2)), 80, 'black', 'Marker', 'x', 'HandleVisibility','off'); hold on 
    scatter3(squeeze(R(1, 1, 3)), squeeze(R(1, 2, 3)), squeeze(R(1, 3, 3)), 80, 'black', 'Marker', 'x', 'HandleVisibility','off'); hold on 

    xlabel('$$x$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    ylabel('$$y$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    zlabel('$$z$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    legend('show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 3, 'Orientation','horizontal', 'Location', 'southoutside');
    grid on;
    title('$$\mathbf{R}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
    
    sgtitle('Attitude on Unit Sphere', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    set(gca, 'DataAspectRatio', [1 1 1])
    set(gcf, 'Renderer', 'painters')
    print(gcf, '-depsc', 'test.eps')
    savefig_helper(options, '_attitude_unit_sphere');
end

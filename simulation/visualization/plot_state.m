function plot_state(t, ts, P, dP, x_r, W, W_d, eulZXY, attitude_d, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    p_r = squeeze(x_r(1:3, 1, :));
    v_r = squeeze(x_r(1:3, 2, :));

    figure('Position', [410 10 400 600])

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw orientation
    subplot(4, 1, 1);
    plot(ts, eulZXY(:, 2),'DisplayName','Roll $$\phi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(ts, eulZXY(:, 3),'DisplayName','Pitch $$\theta$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(ts, eulZXY(:, 1),'DisplayName','Yaw $$\psi$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    
    % desired
    plot(t, attitude_d(2, :),'DisplayName','Roll $$\phi_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, attitude_d(3, :),'DisplayName','Pitch $$\theta_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, attitude_d(1, :),'DisplayName','Yaw $$\psi_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    
    labely = ylabel(["Euler Angles", "(rad)"], 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    legend('show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 3, 'Orientation','horizontal');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw angular velocity
    subplot(4, 1, 2);
    plot(ts, W(:, 1),'DisplayName','$$\Omega_x$$','LineWidth',2); hold on 
    plot(ts, W(:, 2),'DisplayName','$$\Omega_y$$','LineWidth',2); hold on 
    plot(ts, W(:, 3),'DisplayName','$$\Omega_z$$','LineWidth',2); hold on 
    
    plot(t, W_d(1, :),'DisplayName','$${\Omega_x}_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, W_d(2, :),'DisplayName','$${\Omega_y}_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, W_d(3, :),'DisplayName','$${\Omega_z}_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 

    labely = ylabel(["Angular velocity", "(rad/s)"], 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    legend('show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 3, 'Orientation','horizontal');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw positions
    subplot(4, 1, 3);
    plot(ts, P(:, 1),'DisplayName','$$\mathbf{x}_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(ts, P(:, 2),'DisplayName','$$\mathbf{x}_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(ts, P(:, 3),'DisplayName','$$\mathbf{x}_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 

    % desired
    plot(t, p_r(1, :),'DisplayName','$$\mathbf{x}_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, p_r(2, :),'DisplayName','$$\mathbf{x}_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, p_r(3, :),'DisplayName','$$\mathbf{x}_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    
    labely = ylabel(["Position", "(m)"], 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    legend('show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 3, 'Orientation','horizontal');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw velocity
    subplot(4, 1, 4);
    plot(ts, dP(:, 1),'DisplayName','$$\mathbf{v}_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(ts, dP(:, 2),'DisplayName','$$\mathbf{v}_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(ts, dP(:, 3),'DisplayName','$$\mathbf{v}_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 

    % desired
    
    plot(t, v_r(1, :),'DisplayName','$$\mathbf{v}_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, v_r(2, :),'DisplayName','$$\mathbf{v}_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    plot(t, v_r(3, :),'DisplayName','$$\mathbf{v}_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    
    labely = ylabel(["Velocity", "(m/s)"], 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    legend('show', 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 3, 'Orientation','horizontal');
    
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    sgtitle('State profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    savefig_helper(options, '_state');
end

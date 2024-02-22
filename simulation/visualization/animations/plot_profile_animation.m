function plot_profile_animation(env_params, drone_params, fps, t, P, R, x_r, eX, eR, incre, metrics, options)
    n = length(drone_params.psi);
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    labely_pos = options('labely_pos');

    r = zeros([floor(fps * t(end)) 1]);
    idx = 1;
    r(idx) = 1;
    for i=1:length(t)
        if t(i) > idx * (1/fps)
            idx = idx + 1;
            r(idx) = i;
        end
    end

    
    % Data processing
    norm_pos = vecnorm(eX, 2, 1);
    norm_att = eR;
    p_r = squeeze(x_r(1:3, 1, :));
    incre_f = incre(1, :);
    incre_m = incre(2, :);
    ef = metrics(1, :);
    em = metrics(2, :);

    
    figure('Position', [10 10 600 600])
    warning('off', 'MATLAB:hg:ColorSpec_None')
    set(gcf,'Color',[0, 0, 0])
    set(groot,{'DefaultAxesXColor','DefaultAxesYColor','DefaultAxesZColor', 'defaultfigurecolor'},{'w','w','w','w'})

    animation_name = strcat(options('foldername'), options('filename'), '_data_profile.gif');
    subplot(5, 1, 1)
    pp_norm_pos = plot(t(1), norm_pos(1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{x}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{x}\Vert_2$$ (m)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    % labely.Position(1) = labely_pos;
    xlim([0 t(end)]);
    % ylim([0 15]);
    ylim([0 40]);
    % legend('show', 'FontName', 'Times New Roman', 'interpreter', 'latex', 'FontSize', legend_font_size, ...
    %        'Location', 'eastoutside');
    set(gca,'Color',[0, 0, 0])

    subplot(5, 1, 2)
    pp_norm_att = plot(0, 0, 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Psi$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    xlim([0 t(end)]);
    ylim([0 2]);
    % legend('show', 'FontName', 'Times New Roman', 'interpreter', 'latex', 'FontSize', legend_font_size, ...
    %        'Location', 'eastoutside');
    set(gca,'Color',[0, 0, 0])

    subplot(5, 1, 3)
    pp_state_x = plot(t(1), P(1, 1),'DisplayName','$$\mathbf{x}_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    pp_state_y = plot(t(1), P(1, 2),'DisplayName','$$\mathbf{x}_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    pp_state_z = plot(t(1), P(1, 3),'DisplayName','$$\mathbf{x}_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    % desired
    pp_state_xd = plot(t(1), p_r(1, 1),'DisplayName','$$\mathbf{x}_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle', '--'); hold on 
    pp_state_yd = plot(t(1), p_r(2, 1),'DisplayName','$$\mathbf{x}_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle', '--'); hold on 
    pp_state_zd = plot(t(1), p_r(3, 1),'DisplayName','$$\mathbf{x}_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle', '--'); hold on 
    
    labely = ylabel(["Position", "(m)"], 'FontName', 'Times New Roman', 'interpreter', 'latex', 'FontSize', label_font_size);
    % labely.Position(1) = labely_pos;
    xlim([0 t(end)]);
    ylim([-30 30]);
    le = legend('show', 'FontName', 'Times New Roman', 'interpreter', 'latex', 'FontSize', legend_font_size, ...
           'NumColumns', 3, 'Orientation','horizontal', 'Location', 'northeast','color','k', 'TextColor', 'w');
    set(gca,'Color',[0, 0, 0])
    
    subplot(5, 1, 4)
    pp_control_f_inc = plot(t(1), incre_f(1), 'DisplayName', 'Force', 'LineWidth', 2, 'LineStyle', '-'); hold on
    pp_control_m_inc = plot(t(1), incre_m(1), 'DisplayName', 'Torque', 'LineWidth', 2, 'LineStyle', '-'); hold on
    xlim([0 t(end)]);
    ylim([0 1]);
    ylabel(["Wrench", "Allocation", "Increment"], 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    legend('show', 'FontName', 'Times New Roman', 'interpreter', 'latex', 'FontSize', legend_font_size, ...
           'Location', 'southeast','color','k', 'TextColor', 'w');
    set(gca,'Color',[0, 0, 0])

    sbp5 = subplot(5, 1, 5);
    pp_control_f_err = plot(t(1), ef(1), 'DisplayName', 'Force', 'LineWidth', 2, 'LineStyle', '-'); hold on
    pp_control_m_err = plot(t(1), em(1), 'DisplayName', 'Torque', 'LineWidth', 2, 'LineStyle', '-'); hold on
    xlim([0 t(end)]);
    ylim([0 35]); 
    ylabel(["Wrench", "Allocation", "Error"], 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    legend('show', 'FontName', 'Times New Roman', 'interpreter', 'latex', 'FontSize', legend_font_size, ...
           'Location', 'northeast','color','k', 'TextColor', 'w');

    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    set(gca,'Color',[0, 0, 0])
    for i = 1:length(r)
        if t(r(i)) > 15 && t(r(i)) < 15.1
            le.Location = 'northwest';
        end

        % if t(r(i)) > 2 && 60-30*(t(r(i))-2)>5
        %     sbp5.YLim = [0 60-30*(t(r(i))-2)];
        % end

        % suplot 1
        pp_norm_pos.XData = t(1:r(i));
        pp_norm_pos.YData = norm_pos(1:r(i));

        % suplot 2
        pp_norm_att.XData = t(1:r(i));
        pp_norm_att.YData = norm_att(1:r(i));

        % suplot 3
        pp_state_x.XData = t(1:r(i));
        pp_state_x.YData = P(1:r(i), 1);
        pp_state_y.XData = t(1:r(i)); 
        pp_state_y.YData = P(1:r(i), 2);
        pp_state_z.XData = t(1:r(i)); 
        pp_state_z.YData = P(1:r(i), 3);
        % desired
        pp_state_xd.XData = t(1:r(i));
        pp_state_xd.YData = p_r(1, 1:r(i));
        pp_state_yd.XData = t(1:r(i));
        pp_state_yd.YData = p_r(2, 1:r(i));
        pp_state_zd.XData = t(1:r(i));
        pp_state_zd.YData = p_r(3, 1:r(i));

        % suplot 4
        pp_control_f_inc.XData = t(1:r(i));
        pp_control_f_inc.YData = incre_f(1:r(i));
        pp_control_m_inc.XData = t(1:r(i));
        pp_control_m_inc.YData = incre_m(1:r(i));

        % suplot 5
        pp_control_f_err.XData = t(1:r(i));
        pp_control_f_err.YData = ef(1:r(i));
        pp_control_m_err.XData = t(1:r(i));
        pp_control_m_err.YData = em(1:r(i));
        
        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if i == 1
            imwrite(imind, cm, animation_name, 'gif', 'Loopcount', inf, 'DelayTime', 1/fps);
        else
            imwrite(imind, cm, animation_name, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
        end
    end

    % subplot(5, 1, 2)
    % pp_state_x = plot3(0, 0); hold on
    % xlim([0 t(end)]);
    % ylim([-40 40]);

    % subplot(5, 1, 3)
    % pp_state_y = plot3(0, 0); hold on
    % xlim([0 t(end)]);
    % ylim([-40 40]);

    % subplot(5, 1, 4)
    % pp_state_z = plot3(0, 0); hold on
    % xlim([0 t(end)]);
    % ylim([0 80]);

end
function plot_internal_animation(env_params, drone_params, fps, t, z_os, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    labely_pos = options('labely_pos');

    n = length(drone_params.psi);

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
    % Bounds
    xx = [t(1); t(end)];
    cmap = jet(n);

    sigma_w = drone_params.prop_max;
    C_p1 = kron(eye(n), [0 0 1 0]);
    C_p2 = kron(eye(n), [0 0 0 1]);
    z1 = C_p1 * z_os;
    z2 = C_p2 * z_os;

    % figure('Position', [10 10 1200 1200])
    fig = figure('Position', [10 210 600 200]);
    animation_name = strcat(options('foldername'), options('filename'), '_animation_internal.gif');
    
    warning('off', 'MATLAB:hg:ColorSpec_None')
    set(gcf,'Color',[0, 0, 0])
    set(groot,{'DefaultAxesXColor','DefaultAxesYColor','DefaultAxesZColor', 'defaultfigurecolor'},{'w','w','w','w'})

    pp_prop1 = gobjects([n, 1]);
    pp_prop2 = gobjects([n, 1]);

    sp1 = subplot(1, 2, 1,'Parent',fig);
    plot(sp1, xx, [0; 0], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
    plot(sp1, xx, [sigma_w(1); sigma_w(1)], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
    for i = 1:n
        pp_prop1(i) = plot(sp1, t(1), z1(i, 1), 'Color', cmap(i, :), 'LineStyle', '-', 'LineWidth', 2); hold on
    end
    set(gca,'Color',[0, 0, 0])
    xlim([0 t(end)]);
    ylim([0 sigma_w(1)]);
    ylabel('$$\omega_{P_1}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)


    sp2 = subplot(1, 2, 2,'Parent',fig);
    plot(sp2, xx, [0; 0], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
    plot(sp2, xx, [sigma_w(2); sigma_w(2)], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
    for i = 1:n
        pp_prop2(i) = plot(sp2, t(1), z2(i, 1), 'Color', cmap(i, :), 'LineStyle', '-', 'LineWidth', 2); hold on
    end
    set(gca,'Color',[0, 0, 0])
    xlim([0 t(end)]);
    ylim([0 sigma_w(2)]);
    ylabel('$$\omega_{P_2}$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    h = axes(fig,'visible','off'); 
    ticklabel = string(0:n);
    ticklabel(1) = "";
    cb = colorbar(h, 'Ticks', ((0:n)-0.5)/n, 'TickLabels', ticklabel, 'Location', 'southoutside');
    cb.Label.String = 'Agent';
    cb.Label.Position = cb.Label.Position + [-0.55 3 0];
    
    colormap(cb, cmap);

    sp1.Position = sp1.Position + [ 0 0.30 0 -0.40];
    sp2.Position = sp2.Position + [ 0 0.30 0 -0.40];
    
    h.Position = h.Position + [0 -0.15 0 0];
    % return
    for i = 1:length(r)
        
        for k = 1:n
            pp_prop1(k).XData = t(1:r(i));
            pp_prop1(k).YData = z1(k, 1:r(i));
        end

        for k = 1:n
            pp_prop2(k).XData = t(1:r(i));
            pp_prop2(k).YData = z2(k, 1:r(i));
        end

        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if i == 1
            imwrite(imind, cm, animation_name, 'gif', 'Loopcount', inf, 'DelayTime', 1/fps);
        else
            imwrite(imind, cm, animation_name, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
        end

        % delete(fsurf_obj)
        
    end
end
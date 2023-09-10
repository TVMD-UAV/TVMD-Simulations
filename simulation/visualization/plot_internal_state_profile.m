function plot_internal_state_profile(n, lower, upper, t, z_d, z, name, options, show_bound, pos_offset, limits, savename, id)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    % Bounds
    xx = [t(1); t(end)];
    cmap = jet(n);
    % cmap(end+1,:)=1;
    % uint8(ci);
    % c = cmap(uint8(ci), :);

    % figure('Position', [1410+pos_offset(1) 10+pos_offset(2) 400 300])
    % subplot(2, 2, id)
    if id == 1
        tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");
    end 
    nexttile
    % State
    % subplot(2, 1, 1);
    if show_bound
        plot(xx, [lower; lower], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
        plot(xx, [upper; upper], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
    end
    for i = 1:n
        plot(t, z(i, :), 'Color', cmap(i, :), 'LineStyle', '-', 'LineWidth', 2); hold on
        if i == 2
            plot(t, z(i, :), 'Color', cmap(i, :), 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerIndices', 1:floor(length(t)/5):length(t)); hold on 
        end
        % scatter(t, z(i, :), [], ones([length(t) 1]) * i, 'Marker', '.'); hold on
    end
    if ~isempty(limits); ylim(limits); end
    ylabel(name, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    % title(name, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    xlabel("$t$", 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    % if n > 1; caxis([1, n]); end

    % Desired
    % subplot(2, 1, 2);
    % nexttile
    % if show_bound
    %     plot(xx, [lower; lower], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
    %     plot(xx, [upper; upper], 'Color', '#333333', 'LineStyle', '--', 'LineWidth', 2); hold on
    % end
    % for i = 1:n
    %     plot(t, z_d(i, :), 'Color', cmap(i, :), 'LineStyle', '-', 'LineWidth', 2); hold on
    %     % scatter(t, z_d(i, :), [], ones([length(t) 1]) * i, 'Marker', '.'); hold on
    % end
    % if ~isempty(limits); ylim(limits); end
    % ylabel('Desired', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    % % if n > 1; caxis([1, n]); end

    % sgtitle(name, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    
    if n > 1 && id == 4
        % h = axes(gcf, 'visible','off'); 
        
        ticklabel = string(0:n);
        ticklabel(1) = "";
        cb = colorbar('Ticks', ((0:n)-0.5)/n, 'TickLabels', ticklabel);
        
        colormap(cb, cmap);

        % cb.Location = 'southoutside';
        % pos = cb.Position;
        % pos(2) = 0;
        % cb.Position = pos;
        cb.Layout.Tile = 'east';
        % cb.Layout.Tile = 'south';
        cb.Label.String = "Agent";
        cb.Label.FontName = "Times New Roman";
        cb.Label.FontSize = legend_font_size;
        cb.Label.Position = [1 -0.01 0];
        cb.Label.Rotation = 0;
        % cb.Label.Position = [-0.05 0 0];
        set(gcf, 'Renderer', 'painters')

        savefig_helper(options, savename);
        % set(gca,'position', [1410 10 400 350]) % sets figure size

        % 
        % xlabel(cb, "Agent")
        % colorbar('Position', [0.93 0.168 0.022 0.7]);
        % colorbar('Ticks', 0:n, 'TickLabels', ticklabel);
        % colormap jet
        % caxis([1, n]); 
    end

end

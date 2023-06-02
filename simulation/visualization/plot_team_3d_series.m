function plot_team_3d(env_params, drone_params, time_selected, t, P, Rr, zo, tt, zd, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    labely_pos = options('labely_pos');

    camproj perspective
    n = length(drone_params.psi);
    figure('Position', [10 10 300*n 300])

    scaling = 0.01;
    for i=1:length(time_selected)
        subplot(1, length(time_selected), i)
        [m, idx] = min(abs(t - time_selected(i)));
        R = squeeze(Rr(idx, :, :));

        [vec, eta_x, eta_y, Tf] = actuator_mixing(squeeze(zo(idx, :))', drone_params, env_params);
        f0 = get_f(eta_x, eta_y, Tf);
        f0 = reshape(f0, [3 n]) * scaling;
        plot_3kg_swarm(drone_params, P(idx, :), R(:, :), 1 + 256 * t(idx) / t(end), f0); hold on
        plot3(P(idx,1)+[-1 -1 1 1], P(idx,2)+[1 -1 1 -1], P(idx,3)+[0 0 0 0], 'Color','none'); hold on

        % Actual
        % f_r = R(:, :) * vec(1:3) * scaling;
        % t_r = R(:, :) * vec(4:6) * 0.1;
        % quiver3(P(idx, 1), P(idx, 2), P(idx, 3), f_r(1), f_r(2), f_r(3), 'Color', '#FF0000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
        % quiver3(P(idx, 1), P(idx, 2), P(idx, 3), t_r(1), t_r(2), t_r(3), 'Color', '#00FFFF', 'LineWidth', 2, 'AutoScale', 'off'); hold on

        % Desired
        [m, idx2] = min(abs(tt - time_selected(i)));
        [vecd, eta_xd, eta_yd, Tfd] = actuator_mixing(squeeze(zd(:, idx2)), drone_params, env_params);
        f_rd = R(:, :) * vecd(1:3) * scaling;
        t_rd = R(:, :) * vecd(4:6) * 0.1;
        quiver3(P(idx, 1), P(idx, 2), P(idx, 3), f_rd(1), f_rd(2), f_rd(3), 'LineStyle', '-', 'Color', '#FF0000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
        quiver3(P(idx, 1), P(idx, 2), P(idx, 3), t_rd(1), t_rd(2), t_rd(3), 'LineStyle', '-', 'Color', '#00FFFF', 'LineWidth', 2, 'AutoScale', 'off'); hold on

        plot_est_boundary_elliptic_cone(drone_params, R(:, :), squeeze(P(idx, :)), scaling);

        % Post Processing
        xlabel('x', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
        ylabel('y', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
        zlabel('z', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
        title("$t=" + string(t(idx)) + "$", 'Interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', title_font_size)
        set(gca, 'DataAspectRatio', [1 1 1])
        grid on

        campos([3 5 3] + P(idx, :))
        camtarget(P(idx, :))
    end

    set(gcf, 'Renderer', 'painters')
    print(gcf, '-depsc', 'test.eps')
    savefig_helper(options, '_3d_series');
end

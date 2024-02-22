function plot_attainable(vec)
    force_minmax = [min(vec(1:3, :), [], 'all') max(vec(1:3, :), [], 'all')];
    moment_minmax = [min(vec(4:6, :), [], 'all') max(vec(4:6, :), [], 'all')];

    figure(1)
    scatter3(vec(1, :), vec(2, :), vec(3, :));
    set(gca, 'DataAspectRatio', [1 1 1])

    figure(2)
    scatter3(vec(4, :), vec(5, :), vec(6, :));
    set(gca, 'DataAspectRatio', [1 1 1])

    figure('Position', [10 10 1400 800])
    %colormap(prism)
    ms = 80;
    ssp(1) = subplot(2, 3, 1);
    length(vec(1, :))
    length(1:n_max)
    scatter(vec(1, :), vec(2, :), ms, vec(1, :), '.')
    xlabel('x')
    ylabel('y')
    grid on

    ssp(2) = subplot(2, 3, 2);
    scatter(vec(2, :), vec(3, :), ms, vec(1, :), '.')
    xlabel('y')
    ylabel('z')
    grid on
    title ('Forces', 'FontSize', 12)

    ssp(3) = subplot(2, 3, 3);
    scatter(vec(1, :), vec(3, :), ms, vec(1, :), '.')
    xlabel('x')
    ylabel('z')
    grid on

    sspm(1) = subplot(2, 3, 4);
    scatter(vec(4, :), vec(5, :), ms, vec(1, :), '.')
    xlabel('x')
    ylabel('y')
    grid on

    sspm(2) = subplot(2, 3, 5);
    scatter(vec(5, :), vec(6, :), ms, vec(1, :), '.')
    xlabel('y')
    ylabel('z')
    grid on
    title ('Moments', 'FontSize', 12)

    sspm(3) = subplot(2, 3, 6);
    scatter(vec(4, :), vec(6, :), ms, vec(1, :), '.')
    xlabel('x')
    ylabel('z')
    grid on

    linkprop([ssp sspm], {'GridColor', 'GridLineStyle', 'GridAlpha', 'FontSize'})
    ax = gca;
    ax.GridColor = [0 .5 .5];
    ax.GridLineStyle = '-';
    ax.GridAlpha = 0.1;
    ax.FontSize = 12;

    linkaxes(ssp, 'xy')
    axis([force_minmax(1), force_minmax(2), force_minmax(1), force_minmax(2)])

    linkaxes(sspm, 'xy')
    axis([moment_minmax(1), moment_minmax(2), moment_minmax(1), moment_minmax(2)])
end

close all;
clear all;
addpath('../helper_functions')

projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\1012_redistributed_alg\\chirp_';
foldername = ["moore\\", "nullspace\\", "redistributed\\", "interior\\"];
filename = 'allocation_test.mat';

markers = ["o", "x", "^", "."];
color = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30"];

for i = 1:length(foldername)
    full_path = strcat(projectpath, foldername(i), filename);
    load(full_path)

    key = {'projectpath', 'foldername', 'filename', 'disp_name', 'markerStyle', 'color', 'marker_indice'};
    value = {projectpath, foldername, filename, foldername(i), "none", color(i), 1};
    options = containers.Map(key, value);

    plot_outputs(t, vecs, options);
    plot_metrics(t, te, ef, em, df, dm, options);
end

key = {'projectpath', 'foldername', 'filename', 'disp_name', 'markerStyle', 'color', 'marker_indice'};
value = {projectpath, foldername, filename, 'Desired', "none", color(5), 1};
options = containers.Map(key, value);

plot_outputs_setup(t, u, options);
plot_metrics_setup(options);

% region [plot_metrics_setup]
function plot_metrics_setup(options)
    projectpath = options('projectpath');
    filename = options('filename');

    labely_pos = -0.6;
    f = figure(2);
    f.Position = [700 100 600 800];
    subplot(5, 1, 1);
    labely = ylabel('Thrust efficiency (%)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(5, 1, 2);
    labely = ylabel('Force error (%)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(5, 1, 3);
    labely = ylabel('Moment error (%)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(5, 1, 4);
    labely = ylabel('Forces alignment', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(5, 1, 5);
    labely = ylabel('Moments alignment', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    sgtitle('Metrics profile', 'FontName', 'Times New Roman', 'FontSize', 16)
    hl = legend('show');
    hl.Position = [0.22, 0.02, 0.54, 0.045];
    set(hl, 'Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10, 'NumColumns', 3)

    saveas(gcf, strcat(projectpath, filename, '_metrics.svg'));
    saveas(gcf, strcat(projectpath, filename, '_metrics.fig'));
end

% end region [plot_metrics_setup]

% region [plot_metrics]
function plot_metrics(t, te, ef, em, df, dm, options)
    disp_name = options('disp_name');
    color = options('color');
    markerStyle = options('markerStyle');
    marker_indices = options('marker_indice');

    figure(2)
    subplot(5, 1, 1);
    plot(t, te, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    subplot(5, 1, 2);
    plot(t, ef, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    subplot(5, 1, 3);
    plot(t, em, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    subplot(5, 1, 4);
    plot(t, df, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    subplot(5, 1, 5);
    plot(t, dm, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
end

% end region [plot_metrics]

% region [plot_outputs_setup]
function plot_outputs_setup(t, u, options)
    projectpath = options('projectpath');
    filename = options('filename');
    color = options('color');
    markerStyle = options('markerStyle');
    marker_indices = options('marker_indice');
    n_subf = 6;

    % Draw forces
    labely_pos = -0.3;
    f = figure(1);
    f.Position = [100 100 600 800];
    subplot(n_subf, 1, 1);
    plot(t, u(1, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$F_x$ (N)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 2);
    plot(t, u(2, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$F_y$ (N)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 3);
    plot(t, u(3, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$F_z$ (N)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % Draw moments
    subplot(n_subf, 1, 4);
    plot(t, u(4, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$M_x$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 5);
    plot(t, u(5, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$M_y$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 6);
    plot(t, u(6, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$M_z$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    sgtitle('Output profile', 'FontName', 'Times New Roman', 'FontSize', 16)
    hl = legend('show');
    hl.Position = [0.22, 0.02, 0.54, 0.045];
    set(hl, 'Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10, 'NumColumns', 3)

    saveas(gcf, strcat(projectpath, filename, '_norm.svg'));
    saveas(gcf, strcat(projectpath, filename, '_norm.fig'));
end

% end region [plot_outputs]

% region [plot_outputs]
function plot_outputs(t, vecs, options)
    % lineStyle = options('lineStyle');
    disp_name = options('disp_name');
    color = options('color');
    markerStyle = options('markerStyle');
    marker_indices = options('marker_indice');

    n_subf = 6;

    % Draw orientation
    figure(1)
    subplot(n_subf, 1, 1);
    plot(t, vecs(1, :), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    % Draw angular velocity
    subplot(n_subf, 1, 2);
    plot(t, vecs(2, :), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    % Draw position
    subplot(n_subf, 1, 3);
    plot(t, vecs(3, :), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    % Draw velocity
    subplot(n_subf, 1, 4);
    plot(t, vecs(4, :), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    subplot(n_subf, 1, 5);
    plot(t, vecs(5, :), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on

    subplot(n_subf, 1, 6);
    plot(t, vecs(6, :), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
end

% end region [plot_outputs]

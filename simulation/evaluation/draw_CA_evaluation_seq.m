conf_name = "model_A8_inc";

run('initialization/init_params_team.m')   
close all;

projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
projectname = "CA_seq_cmd_8A";

% filenames = ["without_pseudo_boundary", "with_pseudo_boundary", "with_post_enhance"];
% legends = ["$\times$ PBP $\times$ PTE", "$\vee$ PBP $\times$ PTE", "$\vee$ PBP $\vee$ PTE"];
% comp_name = "diff_enables";

filenames = ["ebra_test", "cgi_test", "sqp_test"];
legends = ["EBRA", "CGI", "SQP"];
comp_name = "diff_alg";

markers = ["o", "x", "^", "square", "."];
color = ["#0072BD", "#D95319", "#77AC30", "#EDB120", "#7E2F8E", "#77AC30"];

err_norm = zeros([6 length(filenames)]);

marker_num = 10;
for i = 1:length(filenames)
    full_path = strcat(projectpath, "\\", projectname, "\\", filenames(i), ".mat");
    load(full_path)
    options = gen_project_options_subtask(projectpath, projectname, filenames(i), t(end), true);

    options('disp_name') = legends(i);
    options('color') = color(i);
    options('markerStyle') = markers(i);
    marker_len = floor(length(t)/marker_num);
    options('marker_indice') = floor(marker_len * i / length(filenames)) : marker_len : length(t);
    options('labely_pos') = -t(end)/15;

    te = metrics(5, :);
    ef = metrics(1, :);
    em = metrics(2, :);
    df = metrics(3, :);
    dm = metrics(4, :);
    plot_metrics(t, te, ef, em, df, dm, options)
    plot_outputs(t, vecs, options)

    err_norm(:, i) = vecnorm(u_d - vecs, 2, 2);

    figure(3)
    subplot(3, 1, 1)
    plot(t, violation(1, :), 'DisplayName', options('disp_name'), 'Color', options('color')); hold on
    subplot(3, 1, 2)
    plot(t, violation(2, :), 'DisplayName', options('disp_name'), 'Color', options('color')); hold on
    subplot(3, 1, 3)
    plot(t, violation(3, :), 'DisplayName', options('disp_name'), 'Color', options('color')); hold on

    fprintf("%25s \t", filenames(i));
    fprintf(" & %.2f", err_norm(1:3, i)');
    [avr, vmin, vmax, integ] = cal_statistic(t, ef);
    fprintf("\t & %.2f & %.2f", vmax, integ);
    [avr, vmin, vmax, integ] = cal_statistic(t, df);
    fprintf("\t & %.2f & %.2f", vmin, integ);
    fprintf(" \\\\\n");

    fprintf(" & %.2f", err_norm(4:6, i)');
    [avr, vmin, vmax, integ] = cal_statistic(t, em);
    fprintf("\t & %.2f & %.2f", vmax, integ);
    [avr, vmin, vmax, integ] = cal_statistic(t, dm);
    fprintf("\t & %.2f & %.2f", vmin, integ);
    fprintf(" \\\\\n");
end

% for i = 1:length(filenames)
%     fprintf("%s ", filenames(i));
%     fprintf(" & %.2f", err_norm(:, i)');
%     fprintf(" \\\\\n");
% end

options('color') = color(length(filenames)+1);
% options('filename') = "comp_" + options('filename');
options('filename') = "comp_" + comp_name;
plot_metrics_setup(options)
plot_outputs_setup(t, u_d, options)

figure(3)
legend('show', 'Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', options('legend_font_size'), 'NumColumns', 2);

% region [plot_metrics_setup]
function plot_metrics_setup(options)
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    labely_pos = options('labely_pos');

    f = figure(2);
    % f.OuterPosition = [100 500 600 600];
    % f.InnerPosition = [100 100 600 600];
    f.Position = [100 100 400 600];
    subplot(5, 1, 1);
    labely = ylabel('TE (%)', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    subplot(5, 1, 2);
    labely = ylabel('FE', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    % ylim([0 20])
    labely.Position(1) = labely_pos;

    subplot(5, 1, 3);
    labely = ylabel('ME', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    % ylim([0 10])
    labely.Position(1) = labely_pos;

    subplot(5, 1, 4);
    labely = ylabel('FA', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    % ylim([0.99 1])
    labely.Position(1) = labely_pos;

    subplot(5, 1, 5);
    labely = ylabel('MA', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    % ylim([0.8 1])
    labely.Position(1) = labely_pos;

    xlabel('$t$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    
    offset = 0.01; % vertical offset per row
    scale = 1.0; % amount to scale each axes vertically
    gh = gcf;
    h = gh.Children;
    h = flipud(h(isgraphics(h,'axes')));
    for ax = 1:numel(h)
        h(ax).Position(2) = h(ax).Position(2) + offset; % assumes 2 columns
        h(ax).Position(4) = h(ax).Position(4) * scale;
    end

    hl = legend('show', 'Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 2);
    hl.Position = [0.22, 0.02, 0.54, 0.03];

    sgtitle('Metrics Profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    savefig_helper(options, '_metrics');
end
% endregion [plot_metrics_setup]

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
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    labely_pos = options('labely_pos');

    
    color = options('color');
    markerStyle = options('markerStyle');
    marker_indices = options('marker_indice');
    n_subf = 6;

    % Draw forces
    f = figure(1);
    f.Position = [500 100 400 600];
    subplot(n_subf, 1, 1);
    plot(t, u(1, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$\mathbf{u}_{fx}$ (N)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 2);
    plot(t, u(2, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$\mathbf{u}_{fy}$ (N)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 3);
    plot(t, u(3, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$\mathbf{u}_{fz}$ (N)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    % Draw moments
    subplot(n_subf, 1, 4);
    plot(t, u(4, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$\mathbf{u}_{\tau x}$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 5);
    plot(t, u(5, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$\mathbf{u}_{\tau y}$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;

    subplot(n_subf, 1, 6);
    plot(t, u(6, :), 'DisplayName', 'Desired', 'LineWidth', 2, 'LineStyle', '--', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indices); hold on
    labely = ylabel('$\mathbf{u}_{\tau z}$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)

    offset = 0.01; % vertical offset per row
    scale = 1.0; % amount to scale each axes vertically
    gh = gcf;
    h = gh.Children;
    h = flipud(h(isgraphics(h,'axes')));
    for ax = 1:numel(h)
        h(ax).Position(2) = h(ax).Position(2) + offset; % assumes 2 columns
        h(ax).Position(4) = h(ax).Position(4) * scale;
    end

    hl = legend('show', 'Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', legend_font_size, 'NumColumns', 2);
    hl.Position = [0.22, 0.02, 0.54, 0.03];
    
    sgtitle('Output profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    savefig_helper(options, '_wrench');
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



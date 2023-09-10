projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\icra2024";
projectname = "motor_failure";
filename = "mot_fail";

options = gen_project_options_subtask(projectpath, projectname, filename, 20, true);
fnames = ["cgi_init1_sine_forward_gain1_mot_fail.mat", "erpi_init1_sine_forward_gain1_mot_fail.mat", "interior_init1_sine_forward_gain1_mot_fail.mat"];
marker_style = ["o", "x", "^"];
line_color = ["#0072BD", "#D95319", "#EDB120"];

close all
conf_name = "model_A6_inc";
run('initialization/init_params_team_icra.m')    
model = 'SwarmSystem_2021b';
figure('Position', [10 10 400 500])

for i = 1:length(fnames)
    options('marker_style')  = marker_style(i);
    options('line_color')  = line_color(i);
    options('idx') = i; 
    matfilename = strcat(options('foldername'), fnames(i));
    load(matfilename)
    fprintf("%d, %s\n", i, matfilename)
    plotter(env_params, drone_params, out, options);
end
legend(["CGI", "EBRCA", "INT"], 'FontName', 'Times New Roman', 'FontSize', options('legend_font_size'), 'NumColumns', 3, 'Orientation','horizontal', 'Location', 'southoutside');
% legend()

savefig_helper(options, '_norm_batch');

function plotter(env_params, drone_params, out, options)
    % region [Data Decoding]
    % States
    n = length(drone_params.psi);
    ts = out.tout;
    t = get(out.logsout, 'time').Values.Data;
    x = get(out.xout, 'x').Values.Data;
    z = get(out.xout, 'z').Values.Data;
    num_sample = length(ts);

    eTrans = get(out.logsout, 'eX').Values.Data;
    eX = squeeze(eTrans(:, 1, :));
    eV = squeeze(eTrans(:, 2, :));
    eR = squeeze(get(out.logsout, 'eR').Values.Data);
    eOmega = squeeze(get(out.logsout, 'eOmega').Values.Data);

    plot_norm(t, eX, eV, eR, eOmega, options);
end

function plot_norm_init(t, eX, eV, eR, eOmega, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    labely_pos = options('labely_pos');

    % Draw orientation
    figure('Position', [10 10 400 500])
    subplot(4, 1, 1);
    % plot(t, vecnorm(eR, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, eR, 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Psi$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw angular velocity
    subplot(4, 1, 2);
    plot(t, vecnorm(eOmega, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\Omega\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\Omega\Vert_2$$ (rad/s)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw position
    subplot(4, 1, 3);
    plot(t, vecnorm(eX, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{p}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{X}\Vert_2$$ (m)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw velocity
    subplot(4, 1, 4);
    plot(t, vecnorm(eV, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{v}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{v}\Vert_2$$ (m/s)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    
    sgtitle('Error profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

    savefig_helper(options, '_norm');
end

function plot_norm(t, eX, eV, eR, eOmega, options)
    title_font_size = options('title_font_size');
    sgtitle_font_size = options('sgtitle_font_size');
    label_font_size = options('label_font_size');
    legend_font_size = options('legend_font_size');

    labely_pos = options('labely_pos');
    ml = floor(length(t) / 5);
    idx = options('idx');
    slice = floor(idx*ml/3):ml:length(t);

    % Draw orientation
    subplot(5, 1, 1);
    % plot(t, vecnorm(eR, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, eR, 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{R}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', options('line_color'), 'Marker', options('marker_style'), 'MarkerIndices', slice); hold on
    labely = ylabel('$$\Psi$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw angular velocity
    subplot(5, 1, 2);
    plot(t, vecnorm(eOmega, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\Omega\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', options('line_color'), 'Marker', options('marker_style'), 'MarkerIndices', slice); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\Omega\Vert_2$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw position
    subplot(5, 1, 3);
    plot(t, vecnorm(eX, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{p}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', options('line_color'), 'Marker', options('marker_style'), 'MarkerIndices', slice); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{X}\Vert_2$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    
    % Draw velocity
    subplot(5, 1, 4);
    plot(t, vecnorm(eV, 2, 1), 'DisplayName', '$$\Vert\mathbf{e}_\mathbf{v}\Vert_2$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', options('line_color'), 'Marker', options('marker_style'), 'MarkerIndices', slice); hold on
    labely = ylabel('$$\Vert\mathbf{e}_\mathbf{v}\Vert_2$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size);
    labely.Position(1) = labely_pos;
    xlabel('$$t$$ (sec)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', label_font_size)
    
    sgtitle('State Error Profile', 'FontName', 'Times New Roman', 'FontSize', sgtitle_font_size)

end

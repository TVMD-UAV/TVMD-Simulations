close all;
addpath('../helper_functions')

projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\1015_redistributed_full_dynamics\\r_50_rf_50\\';
foldername = ["moore\\", "nullspace\\", "redistributed\\"];
filename = 'swarm_allocation.mat';

markers = ["o", "x", "^"];
color = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E"];

[key, params] = get_swarm_params();
n = length(params('psi'));

for i = 1:length(foldername)
    full_path = strcat(projectpath, foldername(i), filename);
    load(full_path)

    % Parameters
    g = params('g'); % gravity
    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient

    % Marker style
    makerstyle = false;

    if makerstyle == true
        lineStyle = ':';
        markerStyle = 'o';
    else
        lineStyle = '--';
        markerStyle = 'none';
    end

    % Extract parameters
    u_d = inputs(:, 1:6);
    F_d = inputs(:, 7:16);
    u = inputs(:, 17:22);

    % States
    dW = dydt(:, 1:3);
    W = y(:, 1:3); % Angular velocity
    % Translational
    ddP = dydt(:, 13:15);
    dP = y(:, 13:15);
    P = y(:, 16:18);

    thrust = outputs(:, 1:3);
    B_M_f = outputs(:, 4:6);
    B_M_d = outputs(:, 7:9);
    B_M_a = outputs(:, 10:12);
    B_M_g = outputs(:, 13:15);
    traj = reshape(refs(:, 1:12), [length(y), 3, 4]);
    traj = permute(traj, [1, 3, 2]);
    Q_d = refs(:, 13:15);
    beta = refs(:, 16:18);
    dist_s = refs(:, 19:21);

    % details
    P_prop = rho * prop_d^4 * [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    vtheta = reshape(y(:, 19:18 + 6 * n), [length(y) 6 n]);
    wm = vtheta(:, 1:2, :);
    a = squeeze(vtheta(:, 3, :));
    b = squeeze(vtheta(:, 4, :));

    TfTd = pagemtimes(P_prop, permute(wm .* wm, [2 1 3]));
    Tf = reshape(TfTd(1, :, :), [length(y) n]);

    % Rotation matrix
    R = reshape(y(:, 4:12), [length(y) 3 3]); % 3x3
    eulZXY = rot2zxy_crossover(R);
    attitude_d = Q_d(:, 1:3);
    R = R(r, :, :);

    CoP = P(:, 1:3);

    key = {'projectpath', 'foldername', 'filename', 'disp_name', 'markerStyle', 'color', 'marker_indice'};
    % value = {foldername(i), markers(i), color(i), "none"};
    value = {projectpath, foldername, filename, foldername(i), "none", color(i), 1};
    options = containers.Map(key, value);

    plot_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, options);
    plot_metrics(t, metrics(:, 1), metrics(:, 2), metrics(:, 3), metrics(:, 4), metrics(:, 5), options);
end

plot_norm_setup(options);
plot_metrics_setup(options);

% region [plot_metrics_setup]
function plot_metrics_setup(options)
    projectpath = options('projectpath');
    filename = options('filename');

    labely_pos = -0.3;
    f = figure(2);
    f.Position = [700 100 600 800];
    subplot(5, 1, 1);
    labely = ylabel('Thrust efficiency (%)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(5, 1, 2);
    labely = ylabel('Force error (%)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    subplot(5, 1, 3);
    ylim([0 400])
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
    markerStyle = options('markerStyle');
    color = options('color');
    marker_indice = options('marker_indice');

    figure(2)
    subplot(5, 1, 1);
    plot(t, te, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    subplot(5, 1, 2);
    plot(t, ef, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    subplot(5, 1, 3);
    plot(t, em, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    subplot(5, 1, 4);
    plot(t, df, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    subplot(5, 1, 5);
    plot(t, dm, 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    % saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_metrics.svg'));
    % saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_metrics.fig'));
end

% end region [plot_metrics]

% region [plot_norm_setup]
function plot_norm_setup(options)
    projectpath = options('projectpath');
    filename = options('filename');
    n_subf = 4;
    labely_pos = -0.3;

    % Draw orientation
    f = figure(1);
    f.Position = [100 100 600 800];
    subplot(n_subf, 1, 1);
    labely = ylabel({'Orientation'; '$\Vert\tilde{q}\Vert$ (rad)'}, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % Draw angular velocity
    subplot(n_subf, 1, 2);
    labely = ylabel({'Angular velocity'; '$\Vert\tilde{\Omega}\Vert$ (rad/s)'}, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % Draw position
    subplot(n_subf, 1, 3);
    labely = ylabel({'Position'; '$\Vert\tilde{p}\Vert$ (m)'}, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    % Draw velocity
    subplot(n_subf, 1, 4);
    labely = ylabel({'Velocity'; '$\Vert\tilde{v}\Vert$ (m/s)'}, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;

    sgtitle('Error Norm profile', 'FontName', 'Times New Roman', 'FontSize', 16)
    hl = legend('show');
    hl.Position = [0.22, 0.02, 0.54, 0.045];
    set(hl, 'Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10, 'NumColumns', 3)

    saveas(gcf, strcat(projectpath, filename, '_norm.svg'));
    saveas(gcf, strcat(projectpath, filename, '_norm.fig'));
end

% end region [plot_norm]

% region [plot_norm]
function plot_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, options)
    % lineStyle = options('lineStyle');
    disp_name = options('disp_name');
    markerStyle = options('markerStyle');
    color = options('color');
    marker_indice = options('marker_indice');

    n_subf = 4;

    % Draw orientation
    figure(1)
    subplot(n_subf, 1, 1);
    plot(t, vecnorm(attitude_d - eulZXY, 2, 2), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    % Draw angular velocity
    subplot(n_subf, 1, 2);
    plot(t, vecnorm(W - beta, 2, 2), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    % Draw position
    subplot(n_subf, 1, 3);
    plot(t, vecnorm(traj(:, 1:3, 1) - P, 2, 2), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on

    % Draw velocity
    subplot(n_subf, 1, 4);
    plot(t, vecnorm(traj(:, 1:3, 2) - dP, 2, 2), 'DisplayName', disp_name, 'LineWidth', 2, 'LineStyle', '-', 'Color', color, 'Marker', markerStyle, 'MarkerIndices', marker_indice); hold on
end

% end region [plot_norm]

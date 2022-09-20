close all;
rng('default')

addpath('../model/model')
addpath('../model/viz')
addpath('../helper_functions')

%% Simulation parameters
global T progress;
T = 10;
progress = 0;

% Desired trajectory
syms ts
global ts;
global desire

%% Circular trajectory
w0 = 2 * pi / 10;
%                x,          y, z, psi
%zeta = [10*ts; 30*sin(0.1*ts+3.48); 20*sin(0.1*ts+4.71); 0*ts];
zeta = [3 * sin(0.5 * ts); 2 * ts; 0 * ts; 0 * ts];
%zeta = [cos(w0*ts); sin(w0*ts); 0.1*ts; 0.1*ts];
%zeta = [cos(w0*ts); sin(w0*ts); 1; 0];
d_zeta = diff(zeta);
dd_zeta = diff(d_zeta);
desire = [zeta d_zeta dd_zeta];

% Initial conditions
%                   x y z
%initial_position = [0 -10 -10];
initial_position = [0 0 0];
%                              psi, phi, theta
initial_orientation = getI_R_B(0, 0.5, 0.5);

y0 = zeros([18 + 4 1]);
y0(4:12) = reshape(eye(3) * initial_orientation, [9 1]);
y0(16:18) = initial_position;

% Solver
fprintf("Solver : [");
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
rng('default')
[t, y] = ode45(@drone_fly, [0 T], y0, options);

dydt = zeros([length(y) 18 + 4]);
inputs = zeros([length(y) 16]);
outputs = zeros([length(y) 15]);
refs = zeros([length(y) 18]);
fprintf("] \nForward: [");
progress = 0;
rng('default')

for i = 1:length(y)
    [dydt(i, :), inputs(i, :), outputs(i, :), refs(i, :)] = drone_fly(t(i), y(i, :)');
end

fprintf("] \n");

%% Plot
projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0728\\swarm_allocation\\';
foldername = 'test\\';
filename = 'swarm_allocation';

interval = floor(length(y) / (T * 10));

if interval < 1
    interval = 1;
end

fprintf("Ploting interval: %d\n", interval);
r = 1:interval:length(y);

plotter(t, r, dydt, y, inputs, outputs, refs, ...
    projectpath, foldername, filename);

%% Functions
function [dydt, inputs, outputs, refs] = drone_fly(t, y)
    %% Control input
    global T progress;
    [key, params] = get_swarm_params();
    %[key, params] = get_params_lya();

    traj = TrajectoryPlanner(t);
    %traj = RegulationTrajectory();
    %[u_t, u_d, attitude_d] = Controller(t, params, traj, y);
    %[u_t, u_d, attitude_d] = Controller_MinimumSnap(t, params, traj, y);
    [u_t, u_d, attitude_d] = Controller(params, traj, y);

    u = [u_t; u_d];
    [dydt, commands, meta] = swarm_model(params, u, y);
    %[dydt, commands, meta] = my_model_so3(params, u, y);
    Q = reshape(y(4:12), [3 3]); % 3x3
    thrust = Q * u_t;

    desire = reshape(traj', [12, 1]);
    inputs = [u; commands];
    outputs = [thrust; meta];
    refs = [desire; attitude_d'; [0 0 0]'];

    %% Progress
    current = 40 * t / T;

    if current > progress
        progress = ceil(current);
        fprintf('=')
    end

end

%% Trajectory planner
function traj = TrajectoryPlanner(t)
    global ts
    global desire
    traj = double(subs(desire, ts, t));
    %traj = desire;
end

function traj = RegulationTrajectory()
    traj = zeros([4 3]);
end

%% Controller2
function [u_t, u, attitude_d] = Controller(params, traj, y)
    m = params('m');
    g = params('g');
    r_pg = params('r_pg'); % Leverage length from c.p. to c.g.
    I_b = params('I_b'); % Leverage length from c.p. to c.g.
    l = r_pg(3);

    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);
    eta = y(19);
    xi = y(20);

    % Gain
    Kp = diag([1 1 1]) * 0.2;
    Kd = diag([1 1 1]) * 0.5;

    dd_zeta_com = traj(1:3, 3) + Kd * (traj(1:3, 2) - v) + Kp * (traj(1:3, 1) - p);
    psi_d = traj(4, 1);

    % Trajectory control
    Tf = m * dd_zeta_com + m * [0; 0; g];

    attitude_d = [psi_d, 0, 0];

    % Attitude error
    R_d = getI_R_B(psi_d, 0, 0);

    eRx = 0.5 * (R_d' * R - R' * R_d);
    %eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    eR = [eRx(3, 2); eRx(1, 3); eRx(2, 1)];
    eOmega = w - 0;

    % Attitude control
    Kr = diag([1 1 1]) * 0.2;
    Ko = diag([1 1 1]) * 0.5;
    M_d = I_b * (- Kr * eR - Ko * eOmega) + cross(w, I_b * w);

    u_t = R' * Tf / m;
    u = M_d;
end

function [u_t, u, attitude_d] = Controller_MinimumSnap(t, params, traj, y)
    m = params('m'); % Mass, Kg
    g = params('g');

    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);

    % Gain
    Kp = diag([0.25 1 1]) * 0.2 * 2;
    Kd = diag([0.25 1 1]) * 0.5 * 2;

    mu_d = traj(1:3, 3) + Kd * (traj(1:3, 2) - v) + Kp * (traj(1:3, 1) - p) + [0; 0; g];
    psi_d = traj(4, 1);

    % Trajectory control
    u_t = mu_d' * R(1:3, 3);
    z_B_d = mu_d / norm(mu_d);
    x_C_d = [cos(psi_d); sin(psi_d); 0];
    cross_z_x = cross(z_B_d, x_C_d);
    y_B_d = cross_z_x / norm(cross_z_x);
    x_B_d = cross(y_B_d, z_B_d);

    R_d = [x_B_d y_B_d z_B_d];
    attitude_d = rot2zxy(R_d);

    % Attitude error
    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    eOmega = 0 - w;

    % Attitude control
    Kr = diag([0.25 1 1]) * 0.1 * 2;
    Ko = diag([0.25 1 1]) * 0.05 * 2;
    M_d =- Kr * eR - Ko * eOmega;

    u = M_d;
end

function plotter(t, r, dydt, y, inputs, outputs, refs, projectpath, foldername, filename)
    rotation_matrix = true;

    dirname = strcat(projectpath, foldername);

    if not(isfolder(dirname))
        mkdir(dirname)
    end

    %% Marker style
    makerstyle = false;

    if makerstyle == true
        lineStyle = ':';
        markerStyle = 'o';
    else
        lineStyle = '--';
        markerStyle = 'none';
    end

    %% Extract parameters
    u = inputs(:, 1:6);
    F = inputs(:, 7:16);

    % States
    dW = dydt(:, 1:3);
    W = y(:, 1:3); % Angular velocity
    % Translational
    ddP = dydt(:, 13:15);
    dP = y(:, 13:15);
    P = y(:, 16:18);
    xi = y(:, 19);
    eta = y(:, 20);

    thrust = outputs(:, 1:3);
    B_M_f = outputs(:, 4:6);
    B_M_d = outputs(:, 7:9);
    B_M_a = outputs(:, 10:12);
    B_M_g = outputs(:, 13:15);
    traj = reshape(refs(:, 1:12), [length(y), 3, 4]);
    traj = permute(traj, [1, 3, 2]);
    Q_d = refs(:, 13:15);
    beta = refs(:, 16:18);
    %tilde_mu = outputs(:, 26:28);
    tilde_mu = zeros([length(y) 3]);

    % Rotational
    if rotation_matrix == true
        % Rotation matrix
        R = reshape(y(:, 4:12), [length(y) 3 3]); % 3x3
        eulZXY = rot2zxy_crossover(R);
        attitude_d = Q_d(:, 1:3);
        R = R(r, :, :);
    else
        % Quaternion
        R = zeros([length(r) 3 3]);
        Qs = y(:, 4:7); % Orientation
        eulZXY = Qs(r, 2:4); % Euler angles

        for i = 1:length(r)
            R(i, :, :) = Q2R(Qs(r(i), :));
        end

        attitude_d = Q_d(:, 1:3);
    end

    CoP = P(:, 1:3);

    key = {'projectpath', 'foldername', 'filename', 'lineStyle', 'markerStyle'};
    value = {projectpath, foldername, filename, lineStyle, markerStyle};
    options = containers.Map(key, value);

    %plot_3d(t, r, P, CoP, traj, thrust, R, options);
    plot_state(t, P, dP, traj, W, beta, eulZXY, attitude_d, options);
    plot_error(t, P, dP, traj, W, beta, eulZXY, attitude_d, tilde_mu, options);
    %plot_command(t, Tf, u, options);
    plot_command_6dof(t, u, options)
    %plot_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, theta1, theta_a, options);
    plot_torque(t, B_M_f, B_M_d, B_M_g, B_M_a, options);
    %plot_motor_command(t, w_m1, w_m2, xi, eta, xi_d, eta_d, options);
    %plot_estimation(t, theta1, theta2, theta3, theta_a, theta_b, options);
    %plot_animation(t, r, P, traj, options, R);

    plot_swarm_animation(t, r, P, traj, options, R, thrust);
    %plot_swarm_3d(t, r, P, CoP, traj, thrust, R, options);
end

function plot_swarm_3d(t, r, P, CoP, traj, thrust, R, options)
    figure
    scatter3(P(r, 1), P(r, 2), P(r, 3), 40, t(r)); hold on
    %quiver3(CoP(r, 1), CoP(r, 2), CoP(r, 3), thrust(r, 1), thrust(r, 2), thrust(r, 3), 'magenta')

    % Draw coordinates of the agent
    q1 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 1), R(:, 2, 1), R(:, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 2), R(:, 2, 2), R(:, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(r, 1), P(r, 2), P(r, 3), R(:, 1, 3), R(:, 2, 3), R(:, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';

    % Draw desired trajectory
    scatter3(traj(r, 1, 1), traj(r, 2, 1), traj(r, 3, 1), "red", 'Marker', '.'); hold on

    % Draw agent body
    for i = 1:length(r)
        plot_3kg_swarm(P(r(i), :), R(i, :, :), 1 + 256 * t(r(i)) / t(end), thrust(r, :)); hold on
    end

    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    colorbar
    set(gca, 'DataAspectRatio', [1 1 1])
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.png'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.fig'));
end

function plot_swarm_animation(t, r, P, traj, options, R, thrust)
    warning('off', 'MATLAB:hg:ColorSpec_None')
    figure('Position', [10 10 1200 1200])
    set(gca, 'DataAspectRatio', [1 1 1])
    animation_name = strcat(options('projectpath'), options('foldername'), options('filename'), '_3d.gif');

    scatter3(P(r, 1), P(r, 2), P(r, 3), 'Color', 'none'); hold on
    scatter3([0 max(P(:, 1)) + 2], [0 max(P(:, 2)) + 2], [0 max(P(:, 3)) + 2], 'Color', 'none'); hold on
    ss_traj = scatter3(traj(1, 1, 1), traj(1, 2, 1), traj(1, 3, 1), "red", 'Marker', '.'); hold on
    ss_state = scatter3(P(1, 1), P(1, 2), P(1, 3), "blue"); hold on
    patch_obj = plot_3kg_swarm(P(r(1), :), R(r(1), :, :), 1 + 256 * t(r(1)) / t(end), [0 0 0]); hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')

    for i = 1:length(r)
        ss_traj.XData = traj(1:r(i), 1, 1);
        ss_traj.YData = traj(1:r(i), 2, 1);
        ss_traj.ZData = traj(1:r(i), 3, 1);

        ss_state.XData = P(1:r(i), 1);
        ss_state.YData = P(1:r(i), 2);
        ss_state.ZData = P(1:r(i), 3);

        set(gca, 'DataAspectRatio', [1 1 1])
        draw_3kg_swarm_animation(patch_obj, P(r(i), :), R(i, :, :), 1 + 256 * t(r(i)) / t(end), thrust(r(i), :));

        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);

        if i == 1
            imwrite(imind, cm, animation_name, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(imind, cm, animation_name, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end

    end

end

function plot_command_6dof(t, u, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');
    %% Draw torque
    figure(6)
    subplot(2, 1, 1);
    plot(t, u(:, 1), 'LineStyle', '-', 'DisplayName', 'u_x', 'Color', '#0072BD', 'LineWidth', 2); hold on
    plot(t, u(:, 2), 'LineStyle', '-', 'DisplayName', 'u_y', 'Color', '#D95319', 'LineWidth', 2); hold on
    plot(t, u(:, 3), 'LineStyle', '-', 'DisplayName', 'u_z', 'Color', '#EDB120', 'LineWidth', 2); hold on
    ylabel('N')
    title('Forces')

    subplot(2, 1, 2);
    plot(t, u(:, 4), 'LineStyle', '-', 'DisplayName', '\tau_x', 'Color', '#0072BD', 'LineWidth', 2); hold on
    plot(t, u(:, 5), 'LineStyle', '-', 'DisplayName', '\tau_y', 'Color', '#D95319', 'LineWidth', 2); hold on
    plot(t, u(:, 6), 'LineStyle', '-', 'DisplayName', '\tau_z', 'Color', '#EDB120', 'LineWidth', 2); hold on
    ylabel('Nm')
    xlabel('time')
    title('Moments')

    legend()
    sgtitle('Command profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_command.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_command.fig'));
end

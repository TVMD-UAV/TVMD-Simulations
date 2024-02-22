close all;
rng('default')

addpath('model')
addpath('viz')
addpath('../helper_functions')

projectpath = 'H:\\我的雲端硬碟\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\outputs\\1003_single_agent_distrurbance\\';
foldername = 'test\\';
filename = 'my_model';

global use_my_model;
use_my_model = true;

% Simulation parameters
global T progress;
T = 50;
progress = 0;

% region [reference generation]
% Desired trajectory
syms ts
global desire_t

%% Circular trajectory
w0 = 2 * pi / 10;
%                x,          y, z, psi
zeta = [0.1 * ts; 1 * sin(0.1 * ts); 1 * cos(0.1 * ts); 0 * ts];
%zeta = [cos(w0*ts); sin(w0*ts); 0.1*ts; 0.1*ts];
%zeta = [cos(w0*ts); sin(w0*ts); 1; 0];
d_zeta = diff(zeta);
dd_zeta = diff(d_zeta);
desire = [zeta d_zeta dd_zeta];
desire_t = matlabFunction(desire);
% endregion [reference generation]

% region [Initial conditions]
%                              psi, phi, theta
initial_orientation = getI_R_B(pi / 8, 0, 0);

W0 = [0 0 0]';
R0 = reshape(eye(3) * initial_orientation, [9 1]);
P0 = [1 -1 -1]';
dP0 = [0 0 0]';
vtheta0 = [0 0 0 0]';
y0 = [W0; R0; dP0; P0; vtheta0];

noises_p = wgn(T * 100 + 2, 3, 1e-1, 'linear');
noises_a = wgn(T * 100 + 2, 3, 1e-2, 'linear');
noises = [noises_p noises_a];
% endregion [Initial conditions]

% region [ODE solver]
fprintf("Solver : [");
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
rng('default')
[t, y] = ode45(@(t, y)drone_fly(t, y, noises), [0 T], y0, options);
fprintf("] \nForward: [");
progress = 0;
% endregion [ODE solver]

% region [Reconstruction]
rng('default')
[dydt, inputs, outputs, refs] = cellfun(@(t, y) drone_fly(t, y.', noises), num2cell(t), num2cell(y, 2), 'UniformOutput', false);
dydt = cell2mat(dydt')';
inputs = cell2mat(inputs')';
outputs = cell2mat(outputs')';
refs = cell2mat(refs')';
fprintf("] \n");
% endregion [Reconstruction]

% region [Visualization]
num_slot = 20;
if T * 1 < num_slot; num_slot = T * 0.2; end
interval = floor(length(y) / num_slot);
fprintf("Ploting interval: %d\n", interval);
r = 1:interval:length(y);

plotter(t, r, dydt, y, inputs, outputs, refs, ...
    projectpath, foldername, filename);
% endregion [Visualization]

% region [drone_fly]
function [dydt, inputs, outputs, refs] = drone_fly(t, y, noises)
    % Control input
    global T progress;

    global use_my_model;

    if use_my_model
        [key, params] = get_params_lya();
    else
        [key, params] = get_birotor_params();
    end

    base = floor(t * 100);
    tau = 100 * t - base;
    y(1:3) = y(1:3) + (1 - tau) * noises(base + 1, 1:3)' + tau * noises(base + 2, 1:3)';
    att_noise = ((1 - tau) * noises(base + 1, 4:6) + tau * noises(base + 2, 4:6));
    Q = reshape(y(4:12), [3 3]); % 3x3
    Q = Q * getI_R_B(att_noise(1), att_noise(2), att_noise(3));
    y(4:12) = reshape(Q, [9 1]);

    traj = TrajectoryPlanner(t);
    % traj = RegulationTrajectory();

    %[u_t, u_d, attitude_d] = Controller(t, params, traj, y);
    [u_t, u_d, attitude_d, eR, eOmega] = Controller_MinimumSnap(t, params, traj, y);
    %[u_t, u_d, attitude_d] = Birotor_PID_Controller(params, traj, y);

    u = [u_t; u_d];

    if use_my_model
        [dydt, commands, meta] = my_model_so3(params, t, u, y);
    else
        [dydt, commands, meta] = bi_rotor_model(u, y);
    end

    %
    Q = reshape(y(4:12), [3 3]); % 3x3
    thrust = Q * [0; 0; u_t];

    desire = reshape(traj', [12, 1]);
    inputs = [u; commands];
    outputs = [thrust; meta; eR; eOmega];
    refs = [desire; attitude_d'; [0 0 0]'];

    % Progress
    current = 40 * t / T;

    if current > progress
        progress = ceil(current);
        fprintf('=')
    end

end

% end region [drone_fly]

% region [Trajectory planner]
function traj = TrajectoryPlanner(t)
    global desire_t
    traj = desire_t(t);
end

function traj = RegulationTrajectory()
    traj = zeros([4 3]);
end

% endregion [Trajectory planner]

% region [Birotor_PID_Controller]
function [u_t, u, attitude_d] = Birotor_PID_Controller(params, traj, y)
    m = params('m'); % Mass, Kg
    g = params('g');

    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);

    % Gain
    Kp = diag([0.5 1 1]) * 0.2;
    Kd = diag([0.5 1 1]) * 0.5;

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
    Kr = diag([0.5 1 1]) * 0.1;
    Ko = diag([0.5 1 1]) * 0.05;
    M_d =- Kr * eR - Ko * eOmega;

    u = -M_d;
end

% endregion [Birotor_PID_Controller]

% region [Controller]
function [u_t, u, attitude_d] = Controller(t, params, traj, y)
    m_a = params('m_a'); % Mass, Kg
    m_fm = params('m_fm');
    m = m_a + m_fm;
    g = params('g');
    r_pg = params('r_pg'); % Leverage length from c.p. to c.g.
    l = r_pg(3);
    I_b = params('I_fm');

    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);
    eta = y(19);
    xi = y(20);

    % Gain
    % Kp = diag([1 1 1]) * 0.01;
    % Kd = diag([1 1 1]) * 0.1;
    Kp = diag([1 1 1]) * 1;
    Kd = diag([1 1 1]) * 4;

    dd_zeta_com = traj(1:3, 3) + Kd * (traj(1:3, 2) - v) + Kp * (traj(1:3, 1) - p);
    psi_d = traj(4, 1);

    psi = atan2(-R(1, 2), R(2, 2));
    psi = psi_d;

    % Trajectory control
    Tf = (m * dd_zeta_com(3) + m * g) / (cos(eta) * cos(xi));
    tp = ([cos(psi) sin(psi); sin(psi) -cos(psi)]' * ...
        [m * dd_zeta_com(1) / Tf - xi * cos(psi) - eta * sin(psi); ...
            m * dd_zeta_com(2) / Tf - xi * sin(psi) + eta * cos(psi)]);

    theta_d = tp(1);
    phi_d = tp(2);

    attitude_d = [psi_d, phi_d, theta_d];

    % Attitude error
    R_d = getI_R_B(psi_d, phi_d, theta_d);

    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(3, 2); eRx(1, 3); eRx(2, 1)];
    eOmega = w - 0;

    % Attitude control
    Kr = diag([1 1 1]) * 10;
    Ko = diag([1 1 1]) * 10;
    M_d = I_b * (- Kr * eR - Ko * eOmega) + cross(w, I_b * w);

    u_t = Tf / m;
    u = M_d;
end

% endregion [Controller]

% region [Controller_MinimumSnap]
function [u_t, u, attitude_d, eR, eOmega] = Controller_MinimumSnap(t, params, traj, y)
    m = params('m'); % Mass, Kg
    g = params('g');
    %I_b = params('I_fm');
    I_b = params('I_b');

    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);

    % Gain

    % For tracking
    Kp = diag([1 1 1]) * 1;
    Kd = diag([1 1 1]) * 4;
    % For regulation
    % Kp = diag([1 1 1]) * 0.1;
    % Kd = diag([1 1 1]) * 1;
    %Kp = diag([1 1 1]) * 0.05;
    %Kd = diag([1 1 1]) * 0.5;

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
    %eRx = 0.5 * (R_d' * R - R' * R_d);
    %eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    %eOmega = 0 - w;

    % Attitude control
    %Kr = diag([0.25 1 1]) * 0.1 * 2;
    %Ko = diag([0.25 1 1]) * 0.05 * 2;
    %M_d =- Kr * eR - Ko * eOmega;

    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(3, 2); eRx(1, 3); eRx(2, 1)];
    eOmega = w - 0;

    % Attitude control
    global use_my_model;

    if use_my_model
        % Kr = diag([1 1 1]) * 15;
        % Ko = diag([1 1 1]) * 15;
        Kr = diag([1 1 1]) * 10;
        Ko = diag([1 1 1]) * 10;
    else
        Kr = diag([5 1 1]) * 3;
        Ko = diag([5 1 1]) * 3;
    end

    %Kr = diag([1 1 1]) * 1;
    %Ko = diag([1 1 1]) * 1;
    M_d = I_b * (- Kr * eR - Ko * eOmega) + cross(w, I_b * w);

    u = M_d;
end

% endregion [Controller_MinimumSnap]

% region [plotter]
function plotter(t, r, dydt, y, inputs, outputs, refs, projectpath, foldername, filename)
    rotation_matrix = true;

    dirname = strcat(projectpath, foldername);

    if not(isfolder(dirname))
        mkdir(dirname)
    end

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
    Tf = inputs(:, 1);
    u = inputs(:, 2:4);
    w_m1 = inputs(:, 5);
    w_m2 = inputs(:, 6);
    eta_d = inputs(:, 7);
    xi_d = inputs(:, 8);

    % States
    dW = dydt(:, 1:3);
    W = y(:, 1:3); % Angular velocity
    % Translational
    ddP = dydt(:, 13:15);
    dP = y(:, 13:15);
    P = y(:, 16:18);
    eta = y(:, 19);
    xi = y(:, 20);

    thrust = outputs(:, 1:3);
    B_M_f = outputs(:, 4:6);
    B_M_d = outputs(:, 7:9);
    B_M_a = outputs(:, 10:12);
    B_M_g = outputs(:, 13:15);
    eR = outputs(:, 16:18);
    eOmega = outputs(:, 19:21);
    traj = reshape(refs(:, 1:12), [length(y), 3, 4]);
    traj = permute(traj, [1, 3, 2]);
    Q_d = refs(:, 13:15);
    beta = refs(:, 16:18);
    %tilde_mu = outputs(:, 26:28);
    tilde_mu = zeros([length(y) 3]);

    M_total = B_M_f + B_M_d - B_M_a - B_M_g;

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

    f = figure;
    f.Position = [100 100 540 170];
    plot(t, xi, 'DisplayName', '$$\xi$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, eta, 'DisplayName', '$$\eta$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319'); hold on
    plot(t, xi_d, 'DisplayName', '$$\xi_d$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
    plot(t, eta_d, 'DisplayName', '$$\eta_d$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
    ylabel('Shaft angle (rad)', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time', 'FontName', 'Times New Roman', 'FontSize', 12)
    hl = legend('show');
    set(hl, 'Interpreter', 'latex')
    legend('FontSize', 10)

    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_motor_command.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_motor_command.fig'));

    f = figure;
    f.Position = [100 100 540 300];
    plot(t, B_M_f(:, 1), 'r-'); hold on
    plot(t, B_M_f(:, 2), 'r-'); hold on
    plot(t, B_M_f(:, 3), 'r-'); hold on
    plot(t, B_M_d(:, 1), 'g-'); hold on
    plot(t, B_M_d(:, 2), 'g-'); hold on
    plot(t, B_M_d(:, 3), 'g-'); hold on
    plot(t, B_M_g(:, 1), 'b-'); hold on
    plot(t, B_M_g(:, 2), 'b-'); hold on
    plot(t, B_M_g(:, 3), 'b-'); hold on
    plot(t, B_M_a(:, 1), 'c-'); hold on
    plot(t, B_M_a(:, 2), 'c-'); hold on
    plot(t, B_M_a(:, 3), 'c-'); hold on

    interval = ceil(length(t) / 40);
    delta = ceil(interval / 4);
    rr = 1:interval:length(t) - interval;
    s1 = scatter(t(rr), B_M_f(rr, 1), 'r^', 'DisplayName', 'M_{f,x}', 'LineWidth', 1.5); hold on
    s2 = scatter(t(rr), B_M_f(rr, 2), 'r', 'DisplayName', 'M_{f,y}', 'LineWidth', 1.5); hold on
    s3 = scatter(t(rr), B_M_f(rr, 3), 'r+', 'DisplayName', 'M_{f,z}', 'LineWidth', 1.5); hold on
    s4 = scatter(t(rr + delta), B_M_d(rr + delta, 1), 'g^', 'DisplayName', 'M_{d,x}', 'LineWidth', 1.5); hold on
    s5 = scatter(t(rr + delta), B_M_d(rr + delta, 2), 'g', 'DisplayName', 'M_{d,y}', 'LineWidth', 1.5); hold on
    s6 = scatter(t(rr + delta), B_M_d(rr + delta, 3), 'g+', 'DisplayName', 'M_{d,z}', 'LineWidth', 1.5); hold on
    s7 = scatter(t(rr + delta * 2), B_M_g(rr + delta * 2, 1), 'b^', 'DisplayName', 'M_{g,x}', 'LineWidth', 1.5); hold on
    s8 = scatter(t(rr + delta * 2), B_M_g(rr + delta * 2, 2), 'b', 'DisplayName', 'M_{g,y}', 'LineWidth', 1.5); hold on
    s9 = scatter(t(rr + delta * 2), B_M_g(rr + delta * 2, 3), 'b+', 'DisplayName', 'M_{g,z}', 'LineWidth', 1.5); hold on
    s10 = scatter(t(rr + delta * 3), B_M_a(rr + delta * 3, 1), 'c^', 'DisplayName', 'M_{a,x}', 'LineWidth', 1.5); hold on
    s11 = scatter(t(rr + delta * 3), B_M_a(rr + delta * 3, 2), 'c', 'DisplayName', 'M_{a,y}', 'LineWidth', 1.5); hold on
    s12 = scatter(t(rr + delta * 3), B_M_a(rr + delta * 3, 3), 'c+', 'DisplayName', 'M_{a,z}', 'LineWidth', 1.5); hold on

    ylabel('Moment (Nm)', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time', 'FontName', 'Times New Roman', 'FontSize', 12)
    legend([s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12], 'NumColumns', 3, 'Orientation', 'horizontal', 'FontSize', 10)
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.fig'));

    f = figure;
    f.Position = [100 100 540 300];
    plot(t, u(:, 1), 'k--', 'DisplayName', 'Desired $$u_x$$', 'LineWidth', 2); hold on
    plot(t, u(:, 2), 'k--', 'DisplayName', 'Desired $$u_y$$', 'LineWidth', 2); hold on
    plot(t, u(:, 3), 'k--', 'DisplayName', 'Desired $$u_z$$', 'LineWidth', 2); hold on

    plot(t, M_total(:, 1), 'k-', 'DisplayName', 'Resulting $$M_x$$', 'LineWidth', 2); hold on
    plot(t, M_total(:, 2), 'k-', 'DisplayName', 'Resulting $$M_x$$', 'LineWidth', 2); hold on
    plot(t, M_total(:, 3), 'k-', 'DisplayName', 'Resulting $$M_x$$', 'LineWidth', 2); hold on

    s1 = scatter(t(rr + delta * 3), u(rr + delta * 3, 1), 'k^', 'DisplayName', 'Desired $$u_x$$', 'LineWidth', 1.5); hold on
    s2 = scatter(t(rr + delta * 3), u(rr + delta * 3, 2), 'k', 'DisplayName', 'Desired $$u_y$$', 'LineWidth', 1.5); hold on
    s3 = scatter(t(rr + delta * 3), u(rr + delta * 3, 3), 'k+', 'DisplayName', 'Desired $$u_z$$', 'LineWidth', 1.5); hold on

    s4 = scatter(t(rr + delta * 3), M_total(rr + delta * 3, 1), 'b^', 'DisplayName', 'Resulting $$u_x$$', 'LineWidth', 1.5); hold on
    s5 = scatter(t(rr + delta * 3), M_total(rr + delta * 3, 2), 'b', 'DisplayName', 'Resulting $$u_y$$', 'LineWidth', 1.5); hold on
    s6 = scatter(t(rr + delta * 3), M_total(rr + delta * 3, 3), 'b+', 'DisplayName', 'Resulting $$u_z$$', 'LineWidth', 1.5); hold on
    ylabel('Moment (Nm)', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time', 'FontName', 'Times New Roman', 'FontSize', 12)
    hl = legend([s1, s2, s3, s4, s5, s6], 'NumColumns', 3, 'Orientation', 'horizontal', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')

    %
    f = figure;
    f.Position = [100 100 300 300];
    labely_pos = -1.5;
    subplot(3, 1, 1)
    plot(t, u(:, 1), 'k--', 'DisplayName', 'Desired', 'LineWidth', 2); hold on
    plot(t, M_total(:, 1), 'k-', 'DisplayName', 'Actual', 'LineWidth', 2); hold on
    labely = ylabel('$M_x$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')
    xlim([0 4]);

    subplot(3, 1, 2)
    plot(t, u(:, 2), 'k--', 'DisplayName', 'Desired', 'LineWidth', 2); hold on
    plot(t, M_total(:, 2), 'k-', 'DisplayName', 'Actual', 'LineWidth', 2); hold on
    labely = ylabel('$M_y$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')
    xlim([0 4]);

    subplot(3, 1, 3)
    plot(t, u(:, 3), 'k--', 'DisplayName', 'Desired', 'LineWidth', 2); hold on
    plot(t, M_total(:, 3), 'k-', 'DisplayName', 'Actual', 'LineWidth', 2); hold on
    labely = ylabel('$M_z$ (Nm)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
    hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')
    xlim([0 4]);
    saveas(gcf, strcat(projectpath, foldername, filename, '_moment_diff.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_moment_diff.fig'));

    save('byrotor_moment_profile.mat', 'B_M_f', 'B_M_d', 'B_M_g', 'B_M_a', 't');

    plot_3d(t, r, P, CoP, traj, thrust, R, options);
    plot_state(t, P, dP, traj, W, beta, eulZXY, attitude_d, options);
    plot_error(t, P, dP, traj, eR, eOmega, tilde_mu, options);
    % plot_command(t, Tf, u, options);
    % plot_state_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, options);
    plot_state_norm(t, dP, P, traj, eR, eOmega, options);
    % plot_torque(t, B_M_f, B_M_d, B_M_g, B_M_a, options);
    % plot_motor_command(t, w_m1, w_m2, xi, eta, xi_d, eta_d, options);
    % %plot_estimation(t, theta1, theta2, theta3, theta_a, theta_b, options);
    % plot_animation(t, r, P, traj, options, R);
end

% endregion [plotter]

% region [plot_state_norm]
function plot_state_norm(t, dP, P, traj, eR, eOmega, options)
    labely_pos = -2;

    % Draw orientation
    figure('Position', [10 10 540 400])
    subplot(4, 1, 1);
    plot(t, vecnorm(eR, 2, 2), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
    labely = ylabel({'$\Vert\mathbf{e}_\mathbf{R}\Vert_2$';'(rad)'} , 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    %xlabel('Time (s)')
    %ylim([-0.1 0.1])
    %title('Norm of orientation error')
    %hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    %set(hl, 'Interpreter', 'latex')
    % Draw angular velocity
    subplot(4, 1, 2);
    plot(t, vecnorm(eOmega, 2, 2), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    %plot(t, vecnorm(W - beta, 2, 2), 'DisplayName', '$$\Vert\tilde{\Omega}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel({'$\Vert\mathbf{e}_\Omega\Vert_2$'; '(rad/s)'}, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    %xlabel('Time (s)')
    %ylim([-0.2 0.2])
    %title('Norm of angular velocity error')
    %hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    %set(hl, 'Interpreter', 'latex')
    % Draw position
    subplot(4, 1, 3);
    plot(t, vecnorm(traj(:, 1:3, 1) - P, 2, 2), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    %plot(t, vecnorm(traj(:, 1:3, 1) - P, 2, 2), 'DisplayName', '$$\Vert\tilde{p}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labley = ylabel({'$\Vert\mathbf{e}_\mathbf{p}\Vert_2$'; '(m)'}, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    %labely.Position(1) = labely_pos;
    labely.Position(1) = labely_pos;
    pos = labely.Position
    pos(1) = labely_pos
    pos(2) = 1.2247
    labley.Position = pos
    %xlabel('Time (s)')
    %title('Norm of position error')
    %hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    %set(hl, 'Interpreter', 'latex')
    % Draw velocity
    subplot(4, 1, 4);
    plot(t, vecnorm(traj(:, 1:3, 2) - dP, 2, 2), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    %plot(t, vecnorm(traj(:, 1:3, 2) - dP, 2, 2), 'DisplayName', '$$\Vert\tilde{v}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    labely = ylabel({'$\Vert\mathbf{e}_\mathbf{v}\Vert_2$';'(m/s)'}, 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
    %ylim([-10 10])
    %title('Norm of velocity error')
    %hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    %set(hl, 'Interpreter', 'latex')
    % Draw acceleration

    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state_norm.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state_norm.fig'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state_norm.eps'));
end

% endregion [plot_state_norm]

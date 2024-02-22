close all;
rng('default')

addpath('model')
addpath('viz')
addpath('helper_functions')

%% Simulation parameters
global T progress;
T = 1;
progress = 0;

% Desired trajectory
syms ts
global ts;
global desire

%% Circular trajectory
w0 = 2 * pi / 10;
%                x,          y, z, psi
zeta = [0.1 * ts; 1 * sin(0.1 * ts); 1 * cos(0.1 * ts); 0 * ts];
%zeta = [cos(w0*ts); sin(w0*ts); 0.1*ts; 0.1*ts];
%zeta = [cos(w0*ts); sin(w0*ts); 1; 0];
d_zeta = diff(zeta);
dd_zeta = diff(d_zeta);
desire = [zeta d_zeta dd_zeta];

% Initial conditions
%                   x y z
initial_position = [0 0 0];
%                              psi, phi, theta
initial_orientation = getI_R_B(0, 0, 0);

y0 = zeros([18 + 4 1]);
y0(4:12) = reshape(eye(3) * initial_orientation, [9 1]);
y0(16:18) = initial_position;

% Solver
fprintf("Solver : [");
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
rng('default')
[t, y] = ode45(@drone_fly, [0 T], y0, options);

dydt = zeros([length(y) 18 + 4]);
inputs = zeros([length(y) 8]);
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
projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0805\\model_verification\\';
foldername = 'test\\';
filename = 'my_model';

num_slot = 500;

if T * 10 < num_slot
    num_slot = T * 10;
end

interval = floor(length(y) / num_slot);
fprintf("Ploting interval: %d\n", interval);
r = 1:interval:length(y);

plotter(t, r, dydt, y, inputs, outputs, refs, ...
    projectpath, foldername, filename);

%% Functions
function [dydt, inputs, outputs, refs] = drone_fly(t, y)
    %% Control input
    global T progress;
    %[key, params] = get_birotor_params();
    [key, params] = get_params_lya();

    %traj = TrajectoryPlanner(t);
    traj = RegulationTrajectory();
    [w_m, vartheta_d, attitude_d] = Controller_direct_angular(t, params, traj, y);
    %[u_t, u_d, attitude_d] = Controller_MinimumSnap(t, params, traj, y);
    %[u_t, u_d, attitude_d] = Birotor_PID_Controller(params, traj, y);

    u = [0; 0; 0; 0];
    %[dydt, commands, meta] = bi_rotor_model(u, y);
    [dydt, commands, meta] = my_model_so3_direct(params, w_m, vartheta_d, y);
    Q = reshape(y(4:12), [3 3]); % 3x3
    thrust = Q * [0; 0; 0];

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

function [w_m, vartheta_d, attitude_d] = Controller_direct(t, params, traj, y)
    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm
    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient
    I_fm = params('I_fm'); % upper propeller thrust coefficient
    I_a = params('I_a'); % lower propeller thrust coefficient
    r_pg = params('r_pg'); % Leverage length from c.p. to c.g.

    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed

    angle = 5 * pi / 180;
    Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    Te = Tf * r_pg(3) * sin(angle);
    %            x, y
    vartheta_d = [angle; 0];
    %alpha = [0; angle];
    %           psi,                                       phi, theta
    attitude_d = [0, 0.5 * Te / (I_a(1, 1) + I_fm(1, 1)) * t^2, 0];
    w_m = [w_prop_u; w_prop_l];
end

function [w_m, vartheta_d, attitude_d] = Controller_direct_angular(t, params, traj, y)
    m = params('m'); % kg/m3
    g = params('g'); % kg/m3
    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm
    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient
    I_fm = params('I_fm'); % upper propeller thrust coefficient
    I_a = params('I_a'); % lower propeller thrust coefficient
    r_pg = params('r_pg'); % Leverage length from c.p. to c.g.

    Tf = m * g;
    Td = 0.01;
    %Td = 0;
    beta = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    w_m = sqrt(beta \ [Tf; Td] / (rho * prop_d^4));

    w_prop_u = w_m(1); % Upper rotor speed
    w_prop_l = w_m(2); % Lower rotor speed

    angle = 5 * pi / 180;
    Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    Te = Tf * r_pg(3) * sin(angle);
    %            x, y
    vartheta_d = [angle; 0];
    %alpha = [0; angle];
    %           psi,                                       phi, theta
    attitude_d = [0, 0.5 * Te / (I_a(1, 1) + I_fm(1, 1)) * t^2, 0];
    w_m = [w_prop_u; w_prop_l];
end

function [dydt, commands, meta] = my_model_so3_direct(params, w_m, vartheta_d, y)
    %% Parameters
    g = params('g'); % gravity

    % Drone
    m = params('m');
    m_a = params('m_a'); % Mass, Kg
    m_fm = params('m_fm');
    r_pg = params('r_pg'); % Leverage length from c.p. to c.g.
    r_fm = params('r_fm'); % Leverage length from c.fm. to c.g.

    I_fm = params('I_fm'); % Body Inertial
    I_a = params('I_a'); % Actuator Inertial

    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient

    mKp = params('mKp'); % Servo motor gain
    mKd = params('mKd'); % Servo motor gain

    %% State variables
    W = y(1:3);
    Q = reshape(y(4:12), [3 3]); % 3x3
    dP = y(13:15);
    P = y(16:18);
    vartheta = y(19:22);

    w_m1 = w_m(1);
    w_m2 = w_m(2);

    % Servo motor dynamics
    d_vartheta = [0 0 1 0;
            0 0 0 1;
            -mKp 0 -mKd 0;
            0 -mKp 0 -mKd] * vartheta + [0 0; 0 0; mKp 0; 0 mKp] * vartheta_d;

    eta = vartheta(1);
    xi = vartheta(2);
    %eta = vartheta_d(1);
    %xi = vartheta_d(2);
    dd_eta = d_vartheta(3);
    dd_xi = d_vartheta(4);

    % Translational thrust
    T_f = rho * w_m1^2 * prop_d^4 * CT_u + rho * w_m2^2 * prop_d^4 * CT_l;
    % Drag torque
    T_d = rho * w_m1^2 * prop_d^5 * CP_u - rho * w_m2^2 * prop_d^5 * CP_l;

    %% System start
    % Gyroscopic moment
    A_w_P = [0; 0; w_m1 - w_m2];
    B_w_A = [vartheta(3); vartheta(4); 0];

    B_R_A = Rx(eta) * Ry(xi);
    B_M_g = B_R_A * I_a * (cross(B_R_A' * B_w_A, A_w_P));
    B_M_a = 0 + B_R_A * I_a * B_R_A' * [dd_eta dd_xi 0]';
    B_I_a = B_R_A * I_a * B_R_A';

    % Thrust torque
    thrust = B_R_A * [0; 0; T_f];
    B_M_f = cross(r_pg, thrust);

    % Drag torque from motor
    B_M_d = B_R_A * [0; 0; T_d];

    % Varying Inertia
    I_b = B_I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];

    %[F_d, M_d] = aerial_drag(params, u, y, true);
    F_d = 0;
    M_d = 0;

    %% Newton-Euler equation
    I_thrust = Q * thrust / m;
    B_M = -cross(W, I_b * W) + B_M_f + B_M_d - B_M_g - B_M_a + M_d;

    ddP = [0; 0; -g] + I_thrust + F_d / m;
    dW = I_b \ B_M;
    dQ = reshape(Q * skew(W), [9 1]);

    dydt = [dW; dQ; ddP; dP; d_vartheta];
    commands = [w_m1; w_m2; vartheta_d];
    meta = [B_M_f; B_M_d; -B_M_a; -B_M_g];
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

    ylabel('Torque (Nm)', 'FontName', 'Times New Roman', 'FontSize', 12)
    xlabel('Time', 'FontName', 'Times New Roman', 'FontSize', 12)
    legend([s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12], 'NumColumns', 3, 'Orientation', 'horizontal', 'FontSize', 10)
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_torque.fig'));

    % plot_3d(t, r, P, CoP, traj, thrust, R, options);
    plot_state(t, P, dP, traj, W, beta, eulZXY, attitude_d, options);
    % plot_error(t, P, dP, traj, W, beta, eulZXY, attitude_d, tilde_mu, options);
    % plot_command(t, Tf, u, options);
    % plot_state_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, options);
    % %plot_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, theta1, theta_a, options);
    % plot_torque(t, B_M_f, B_M_d, B_M_g, B_M_a, options);
    % plot_motor_command(t, w_m1, w_m2, xi, eta, xi_d, eta_d, options);
    % %plot_estimation(t, theta1, theta2, theta3, theta_a, theta_b, options);
    % plot_animation(t, r, P, traj, options, R);
end

function plot_state(t, P, dP, traj, W, beta, eulZXY, attitude_d, options)
    lineStyle = options('lineStyle');
    markerStyle = options('markerStyle');

    labely_pos = -0.05;

    %% Draw orientation
    figure('Position', [10 10 540 600])
    subplot(4, 1, 1);
    plot(t, eulZXY(:, 1), 'DisplayName', '$$\phi$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, eulZXY(:, 2), 'DisplayName', '$$\theta$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319'); hold on
    plot(t, eulZXY(:, 3), 'DisplayName', '$$\psi$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120'); hold on
    % Draw desired trajectory

    % plot(t, attitude_d(:, 1),'DisplayName','Yaw $$\psi_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, attitude_d(:, 2),'DisplayName','Roll $$\phi_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, attitude_d(:, 3),'DisplayName','Pitch $$\theta_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on

    labely = ylabel('Attitude (rad)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    % xlabel('time', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylim([-0.5 0.5])
    % title('Orientation')
    hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')

    %% Draw angular velocity
    %figure('Position', [10 10 800 400])
    subplot(4, 1, 2);
    plot(t, W(:, 1), 'DisplayName', '$$\omega_x$$', 'LineWidth', 2); hold on
    plot(t, W(:, 2), 'DisplayName', '$$\omega_y$$', 'LineWidth', 2); hold on
    plot(t, W(:, 3), 'DisplayName', '$$\omega_z$$', 'LineWidth', 2); hold on

    % plot(t, beta(:, 1),'DisplayName','Yaw $${\omega_x}_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, beta(:, 2),'DisplayName','Roll $${\omega_y}_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, beta(:, 3),'DisplayName','Pitch $${\omega_z}_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on

    labely = ylabel('Angular Velocity (rad/s)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    labely.Position(2) = 0;
    % xlabel('time')
    ylim([-2 2])
    % title('Angular velocity w.r.t. body frame')
    hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')

    %% Draw positions
    subplot(4, 1, 3);
    plot(t, P(:, 1), 'DisplayName', '$$P_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, P(:, 2), 'DisplayName', '$$P_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319'); hold on
    plot(t, P(:, 3), 'DisplayName', '$$P_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120'); hold on

    % % Draw desired trajectory
    % plot(t, traj(:, 1, 1),'DisplayName','$$P_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, traj(:, 2, 1),'DisplayName','$$P_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, traj(:, 3, 1),'DisplayName','$$P_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on

    labely = ylabel('Position (m)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    % xlabel('time', 'FontName', 'Times New Roman', 'FontSize', 12)
    % title('Position')
    hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')

    %% Draw velocity
    subplot(4, 1, 4);
    plot(t, dP(:, 1), 'DisplayName', '$$V_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
    plot(t, dP(:, 2), 'DisplayName', '$$V_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319'); hold on
    plot(t, dP(:, 3), 'DisplayName', '$$V_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120'); hold on

    % Draw desired trajectory

    % plot(t, traj(:, 1, 2),'DisplayName','$$V_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, traj(:, 2, 2),'DisplayName','$$V_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on
    % plot(t, traj(:, 3, 2),'DisplayName','$$V_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on

    labely = ylabel('Velocity (m/s)', 'FontName', 'Times New Roman', 'FontSize', 12);
    labely.Position(1) = labely_pos;
    xlabel('time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylim([-10 10])
    % title('Velocity w.r.t. inertial frame')
    hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(hl, 'Interpreter', 'latex')

    % sgtitle('State profile')
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state.fig'));
end

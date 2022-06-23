close all;
rng('default')

addpath('model')
addpath('viz')
addpath('helper_functions')

%% Simulation parameters
global T progress;
T = 20;
progress = 0;

% Initial states

% Desired trajectory
syms ts
global ts;
global desire

%% Regulation
%desire = zeros([4, 3]);

%% Circular trajectory
w0 = 2*pi/10;
%                x,          y, z, psi
%zeta = [cos(w0*ts); sin(w0*ts); 0.1*ts; 0.1*ts];
%zeta = [cos(w0*ts); sin(w0*ts); 1; 0];
zeta = [0*ts; 0*ts; 0*ts; 0*ts];
d_zeta = diff(zeta);
dd_zeta = diff(d_zeta);
desire = [zeta d_zeta dd_zeta];

%% Command generation
% Initial conditions
%                   x y z
initial_position = [1 1 0];
%                              psi, phi, theta
initial_orientation = getI_R_B(  0,   0, 0);

y0 = zeros([18+4 1]);
y0(4:12) = reshape(eye(3) * initial_orientation, [9 1]);
y0(16:18) = initial_position;

% Solver
fprintf("Solver : [");
options = odeset('RelTol',1e-5,'AbsTol',1e-7);
rng('default')
[t, y] = ode45(@drone_fly, [0 T], y0, options);

dydt = zeros([length(y) 18+4]);
inputs = zeros([length(y) 8]);
outputs = zeros([length(y) 15]);
refs = zeros([length(y) 18]);
fprintf("] \nForward: [");
progress = 0;
rng('default')
for i=1:length(y)
    [dydt(i, :), inputs(i, :), outputs(i, :), refs(i, :)] = drone_fly(t(i), y(i, :)');
end
fprintf("] \n");

%% Plot
projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0606_birotor\\model_verification\\';
foldername = 'test\\';
filename = 'birotor_veri';

interval = floor(length(y) / (T * 10));
fprintf("Ploting interval: %d\n", interval);
r = 1:interval:length(y);

plotter(t, r, dydt, y, inputs, outputs, refs, ...
    projectpath, foldername, filename);

%% Functions
function [dydt, inputs, outputs, refs] = drone_fly(t, y)
    %% Control input
    global T progress;
    [key, params] = get_birotor_params();

    %traj = TrajectoryPlanner(t);
    traj = RegulationTrajectory();
    %[T_f, T_d, alpha, attitude_d] = Controller(t, params, traj, P, dP, Q, W, xi, eta);
    [u_t, u_d, attitude_d] = Birotor_PID_Controller(params, traj, y);
    
    %[u_t, u_d, attitude_d] = ControllerTiltedBody(params);
    %[T_f, T_d, alpha, attitude_d] = ControllerDragTorque(t, params, traj, P, dP, Q, xi, eta);
    %[T_f, T_d, alpha, attitude_d] = ControllerTiltedRotor(t, params, traj, P, dP, Q, xi, eta);
    %[T_f, T_d, alpha, attitude_d] = ControllerSinsoid(t, params, traj, P, dP, Q, xi, eta);
    %[w_m1, w_m2, vartheta_d, attitude_d] = ControllerTiltedRotor2(t, params, traj, P, dP, Q, vartheta(2), vartheta(1));
    %[w_m1, w_m2, vartheta_d, attitude_d] = ControllerGyro(t, params, traj, P, dP, Q, vartheta(2), vartheta(1));
    u = [u_t; u_d];
    [dydt, commands, meta] = bi_rotor_model(u, y);
    Q = reshape(y(4:12), [3 3]); % 3x3
    thrust = Q * [0; 0; u_t];

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

%% Controllers
function [u_t, u, attitude_d] = Birotor_PID_Controller(params, traj, y)
    m = params('m');     % Mass, Kg
    g = params('g');

    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);
    
    % Gain
    Kp = diag([1 1 1]) * 0.2;
    Kd = diag([1 1 1]) * 0.5;

    mu_d = traj(1:3, 3) + Kd * (traj(1:3, 2) - v) + Kp * (traj(1:3, 1) - p) + [0;0;g];
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
    %attitude_d = [psi_d phi_d theta_d];

    % Attitude error
    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    eOmega = 0 - w;

    % Attitude control
    Kr = diag([1 1 1]) * 0.1;
    Ko = diag([1 1 1]) * 0.05;
    M_d = - Kr * eR - Ko * eOmega;

    u = -M_d;
end

%% Controller2
function [w_m1, w_m2, alpha, attitude_d] = Controller2(t, params, traj, p, v, R, w, xi, eta)
    m_a = params('m_a');     % Mass, Kg
    m_fm = params('m_fm');
    m = m_a + m_fm;
    g = params('g');
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
    l = r_pg(3);

    % Gain
    Kp = diag([1 1 1]) * 0.2;
    Kd = diag([1 1 1]) * 0.5;

    dd_zeta_com = traj(1:3, 3) + Kd * (traj(1:3, 2) - v) + Kp * (traj(1:3, 1) - p);
    psi_d = traj(4, 1);

    psi = atan2(-R(1, 2), R(2, 2));
    psi = psi_d;

    % Trajectory control
    Tf = (m * dd_zeta_com(3) + m*g) / (cos(eta) * cos(xi));
    %theta_d = m * dd_zeta_com(1) / Tf - xi*cos(psi_d) + eta*sin(psi_d);
    %phi_d = m * dd_zeta_com(2) / Tf - xi*sin(psi_d) + eta*cos(psi_d);
    tp = ([cos(psi) sin(psi); sin(psi) -cos(psi)]' * ...
        [m * dd_zeta_com(1) / Tf - xi*cos(psi) - eta*sin(psi); ...
         m * dd_zeta_com(2) / Tf - xi*sin(psi) + eta*cos(psi)]);

    theta_d = tp(1);
    phi_d = tp(2);

    attitude_d = [psi_d, phi_d, theta_d];

    % Attitude error
    %R_d = Ry(theta_d) * Rx(phi_d) * Rz(psi_d);
    R_d = getI_R_B(psi_d, phi_d, theta_d);
    
    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    eOmega = 0 - w;

    % Attitude control
    Kr = diag([1 1 1]) * 0.1;
    Ko = diag([1 2 1]) * 0.01;
    M_d = - Kr * eR - Ko * eOmega;

    % Control allocator
    Td = -M_d(3);
    A = [l*Tf Td; -Td l*Tf];
    alpha = -A \ [M_d(1); M_d(2)];

    % Motor commands
    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient
    
    beta = [CT_u CT_l; prop_d*CP_u -prop_d*CP_l];
    w_m = sqrt(beta \ [Tf; Td] / (rho * prop_d^4));
    w_m1 = w_m(1);
    w_m2 = w_m(2);
end

%% Controller Tilted Body
function [u_t, u, attitude_d] = ControllerTiltedBody(params)
    % Parameters
    g = params('g');     % gravity
    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient

    % Control input
    % Thrust command
    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed

    % Translational thrust
    %Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    % Drag torque
    %Td = rho * w_prop_u^2 * prop_d^5 * CP_u - rho * w_prop_l^2 * prop_d^5 * CP_l;
    u_t = g;
    u = [0;0;0.005];
    attitude_d = [0, 0, 0];
end

function [Tf, Td, alpha, attitude_d] = ControllerDragTorque(t, params, traj, p, v, R, xi, eta)
    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm
    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient
    I_fm = params('I_fm'); % upper propeller thrust coefficient
    I_a = params('I_a'); % lower propeller thrust coefficient

    % Control input
    % Thrust command
    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed

    Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    Td = 0.01;
    alpha = [0; 0];
    attitude_d = [0.5 * Td / (I_a(3, 3) + I_fm(3, 3)) * t^2, 0, 0];
end

function [Tf, Td, alpha, attitude_d] = ControllerTiltedRotor(t, params, traj, p, v, R, xi, eta)
    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm
    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient
    I_fm = params('I_fm'); % upper propeller thrust coefficient
    I_a = params('I_a'); % lower propeller thrust coefficient
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.

    % Control input
    % Thrust command
    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed

    Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    Td = 0;
    angle = 5*pi/180;
    Te = Tf * r_pg(3) * sin(angle);
    %            x, y
    alpha = [angle; 0];
    %alpha = [0; angle];
    %           psi,                                       phi, theta
    attitude_d = [0, 0.5 * Te / (I_a(3, 3) + I_fm(3, 3)) * t^2, 0];
    %attitude_d = [0, 0, 0.5 * Te / (I_a(2, 2) + I_fm(2, 2)) * t^2];
end

function [w_m1, w_m2, alpha, attitude_d] = ControllerTiltedRotor2(t, params, traj, p, v, R, xi, eta)
    % Control input
    % Thrust command
    w_m1 = 198.2746; % Upper rotor speed
    w_m2 = 198.2746; % Lower rotor speed
    %        x, y
    alpha = [0; 0];
    attitude_d = [0, 0, 0];
end

function [w_m1, w_m2, alpha, attitude_d] = ControllerGyro(t, params, traj, p, v, R, xi, eta)
    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm
    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient
    I_fm = params('I_fm'); % upper propeller thrust coefficient
    I_a = params('I_a'); % lower propeller thrust coefficient
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.

    % Control input
    % Thrust command
    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed
    Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    Td = 1;

    beta = [CT_u CT_l; prop_d*CP_u -prop_d*CP_l];
    w_m = sqrt(beta \ [Tf; Td] / (rho * prop_d^4));
    w_m1 = w_m(1);
    w_m2 = w_m(2);

    %        x, y
    alpha = [0; 0];
    attitude_d = [0, 0, 0];
end

function [Tf, Td, alpha, attitude_d] = ControllerSinsoid(t, params, traj, p, v, R, xi, eta)
    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm
    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient
    I_fm = params('I_fm'); % upper propeller thrust coefficient
    I_a = params('I_a'); % lower propeller thrust coefficient
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.

    % Control input
    % Thrust command
    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed

    Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    Td = 0;

    amp = 1*pi/180;
    w0 = pi / 2;
    alpha = amp * [cos(w0*t); sin(w0*t)];
    %           psi, phi, theta
    attitude_d = [0, 0, 0];
end
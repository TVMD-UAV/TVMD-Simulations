close all;
rng('default')

addpath('model')
addpath('viz')
addpath('helper_functions')

[keys, params] = get_params_lya();
parameter_chack(params);
%return
%% Simulation parameters
global T progress;
T = 50;
progress = 0;

% Initial states
y0 = zeros([13+4+9 1]);
%Q2 = theta_vector2Q(pi, [1;0;0]);
y0(4:7) = [1; 0; 0; 0]; % Initial orientation
%y0(4:7) = Q2;
y0(11:13) = [-20 -10 -20];    % Initial position
%y0(11:13) = [0 0 0];    % Initial position
y0(8:10) = [0 0 0];    % Initial velocity

% Reference
syms ts
%zeta = [10*ts; 30*sin(0.1*ts+3.48); 20*sin(0.1*ts+4.71); 0*ts];
zeta = [0*ts; 0*ts; 0*ts; 0*ts];
d_zeta = diff(zeta);
dd_zeta = diff(d_zeta);
ddd_zeta = diff(dd_zeta);
dddd_zeta = diff(ddd_zeta);
desire = [zeta d_zeta dd_zeta ddd_zeta dddd_zeta];

% Solver
fprintf("Solver : [");
options = odeset('RelTol',1e-5,'AbsTol',1e-7);
[t, y] = ode45(@drone_fly, [0 T], y0, options);

dydt = zeros([length(y) 13+4+9]);
inputs = zeros([length(y) 4+4]);
outputs = zeros([length(y) 25+3+12]);
fprintf("] \nForward: [");
progress = 0;
rng('default')
for i=1:length(y)
    [dydt(i, :), inputs(i, :), outputs(i, :)] = drone_fly(t(i), y(i, :)');
end
fprintf("] \n");

%% Plot
projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Courses\\110-2\\AdaptiveControl\\FinalProject\\Simulations\\ExtendToMyModel\\';
foldername = 'test\\';
filename = 'PositionRegulation';

interval = floor(length(y) / (T * 2));
fprintf("Ploting interval: %d\n", interval);
r = 1:interval:length(y);

plotter_quaternion(t, r, dydt, [y zeros([length(y) 9])], inputs, outputs, ...
    projectpath, foldername, filename);

%% Functions
function [dydt, inputs, outputs] = drone_fly(t, y)
    %% Parameters
    [key, params] = get_params_lya();
    
    %% Control input
    global T progress;

    %% Controller
    %traj = TrajectoryPlanner(t);
    traj = zeros([4, 5]);
    [u_t, u, attitude_d, beta, tilde_mu] = Controller(t, params, traj, y);
    us = [u_t; u];
    %[dydt_m, commands, meta] = my_model(params, us, y);
    [dydt_m, commands, meta] = my_model_simplified(params, us, y);
    dydt = [dydt_m; zeros([9 1])];

    %% Newton-Euler equation
    Q = y(4:7);
    R = Q2R(Q);
    I_thrust = R * [0; 0; u_t];
    
    inputs = [u_t; u; commands];
    desire = reshape(traj(1:4, 1:3)', [12, 1]);
    outputs = [I_thrust ; u; desire; attitude_d; beta; tilde_mu; meta];

    % Progress 
    current = 40 * t / T;
    if current > progress
        progress = ceil(current);
        fprintf('=')
    end
end

%% Trajectory Planner
function traj = TrajectoryPlanner(t)
    global ts
    global desire
    traj = double(subs(desire, ts, t));
    %traj = desire;
end

%% Controller
function [Tf, u, attitude_d, beta, tilde_mu] = Controller(t, params, traj, y)
    %% States 
    W = y(1:3);
    Q = y(4:7);
    v = y(8:10);
    p = y(11:13);  

    %% Parameters
    g = params('g');
    m = params('m');     % Mass, Kg
    
    % Varying Inertia
    m_a = params('m_a');     % Mass, Kg
    m_fm = params('m_fm');
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
    r_fm = params('r_fm'); % Leverage length from c.fm. to c.g. 

    I_fm = params('I_fm'); % Body Inertial
    I_a  = params('I_a'); % Actuator Inertial
    I_b = I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];
    %I_b = diag([0.6 0.6 0.3]); % Body Inertial

    %% Gains
    Kp = params('Kp');
    Kv = params('Kv');
    Kq = params('Kq');
    Kw = params('Kw');
    Gamma_v = params('Gamma_v');
    gamma_q = params('gamma_q');
    k_theta = params('k_theta');

    k_theta = 0;
    Kv = 1;
    K1 = Kp * inv(Gamma_v);
    K2 = (Kv + k_theta) * eye(3);

    %% Errors
    tilde_p = p - traj(1:3, 1);
    tilde_v = v - traj(1:3, 2);

    %% Desired acceleration
    mu_d = traj(1:3, 3) - Kv*h(tilde_v) - Kp*inv(Gamma_v)*h(tilde_p);

    %% Attitude extraction
    z_hat = [0 0 1]';
    u_t = norm(mu_d - g*z_hat);
    q0_d = sqrt(0.5 * (1 + (g-mu_d(3)) / norm(mu_d - g*z_hat)));
    if g-mu_d(3) < 0
        g-mu_d(3)
    end
    q_d = [mu_d(2); -mu_d(1); 0] / (2 * u_t *q0_d);
    Q_d = [q0_d; q_d];

    %% Constructing M matrix
    norm_mu = u_t;
    c1 = norm_mu + g - mu_d(3);
    M = [-mu_d(1)*mu_d(2)      -mu_d(2)^2 + norm_mu*c1  mu_d(2)*c1;
         mu_d(1)^2-norm_mu*c1  mu_d(1)*mu_d(2)          -mu_d(1)*c1;
         mu_d(2)*norm_mu       -mu_d(1)*norm_mu         0] / (norm_mu^2 * c1);

    %% Desired angular velocity
    tilde_Q = quaternion_multiplication(quaternion_inverse(Q_d), Q);
    omega_beta = Kp * Kv * phi_h(tilde_v) / Gamma_v * h(tilde_p);
    beta = M * (traj(1:3, 4) + omega_beta) - Kq * tilde_Q(2:4);
    %beta = - Kq * tilde_Q(2:4);
    tilde_W = W - beta;

    % fb
    R = Q2R(Q);
    bar_q = skew(z_hat)*tilde_Q(2:4) + tilde_Q(1)*z_hat;
    W1 = -2*u_t*skew(bar_q)*R';
    tilde_mu = W1' * tilde_Q(2:4);
    %W2 = Kv^2 * phi_h(tilde_v);
    %W3 = -gamma_theta * gamma_q * (k_theta + Kv) * phi_h(tilde_v) * M' - Kv * phi_h(tilde_v) * W1';
    %W4 = -gamma_theta * Gamma_v - Kp * Gamma_v \ phi_h(tilde_p);
    W2 = Kv^2 * phi_h(tilde_v);
    W3 = -Kv * phi_h(tilde_v)*W1';
    W4 = -Kp * Gamma_v \ phi_h(tilde_p);
    Z1 = get_Z1(params, mu_d, traj(1:3, 4) + omega_beta, M, u_t);
    f_mu_d = traj(1:3, 4) + omega_beta + ...
             (W2 + k_theta*K2*phi_h(tilde_v))*h(tilde_v) + ...
             W3 * tilde_Q(2:4) + ...
             W4 * tilde_v;
    f_tilde_v2 = -K1 * h(tilde_p) - K2 * h(tilde_v) + tilde_mu;
    f_wb = Kp*Kv*f_phi_h(tilde_v, Gamma_v\h(tilde_p)) * f_tilde_v2 + ...
           Kp*Kv*phi_h(tilde_v)/Gamma_v*phi_h(tilde_p) * tilde_v;
    f_tilde_q = 0.5*(tilde_Q(1)*eye(3) + skew(tilde_Q(2:4))) * W + ...
                0.5*(skew(tilde_Q(2:4))-tilde_Q(1)*eye(3))*(M*f_mu_d);
    fb2 = Z1 * f_mu_d + M * (traj(1:3, 5) + f_wb) - Kq * f_tilde_q;

    %% Control law for torque
    u = -gamma_q * tilde_Q(2:4) + skew(W) * I_b * W  - Kw * tilde_W + I_b * fb2;
    Tf = u_t;

    attitude_d = Q_d;
end

%% Controller helper functions
function y = get_Z1(params, mu_d, v, M, u_t)
    g = params('g');
    c1 = u_t + g - mu_d(3);
    gamma_M = 1/(c1 * u_t^2);
    f_gamma = gamma_M^2 * ([0;0;g]-mu_d)' * (3*c1*eye(3) + skew([0;0;1])*skew([0;0;g]-mu_d));
    alpha1 = (mu_d - [0;0;g]) / norm(mu_d - [0;0;g]);
    alpha2 = alpha1 - [0;0;1];
    lambda1 = [c1 * v(2); -c1 * v(1); mu_d(2)*v(1)-mu_d(1)*v(2)] * alpha1' + ...
            [-mu_d(2)*v(1)                       -mu_d(1)*v(1)-2*v(2)*mu_d(2)+c1*v(3)  0;
             2*v(1)*mu_d(1)+v(2)*mu_d(2)-c1*v(3)                         mu_d(1)*v(2)  0;
             -u_t*v(2)                                                       u_t*v(1)  0] + ...
            [u_t*v(2)+mu_d(2)*v(3); -u_t*v(1)-mu_d(1)*v(3); 0] * alpha2';
    y = M' * v * f_gamma / gamma_M + gamma_M * lambda1;
end

function parameter_chack(params)
    g = params('g');
    Kp = params('Kp');
    Kv = params('Kv');
    Kq = params('Kq');
    k_theta = params('k_theta');
    Gamma_v = params('Gamma_v');
    gamma_theta1 = params('gamma_theta1');
    gamma_q = params('gamma_q');
    delta_a = params('delta_a');
    esp_alpha = 0.1;
    
    z_hat = [0;0;1];
    disp('Checking desired virtual acceleration')
    delta_r = 1;
    delta_rz = 1;
    c1 = (g-delta_rz-delta_a) - (Kp*norm(z_hat'/Gamma_v)+2*k_theta+Kv+esp_alpha);

    bar_u_t = g + delta_r + delta_a + Kp*norm(inv(Gamma_v)) + Kv + 2*k_theta + esp_alpha;
    delta_mu_d = g - delta_rz - delta_a - Kp*norm(z_hat'/Gamma_v) - Kv - 2*k_theta - esp_alpha;

    bar_u_t
    delta_mu_d

    esp1 = 0.1;
    esp2 = 15;
    delta_1 = (2*bar_u_t+sqrt(2)*gamma_q*gamma_theta1/delta_mu_d)*norm(Gamma_v) ...
        + sqrt(2)*gamma_q*Kp/delta_mu_d*norm(inv(Gamma_v));
    
    c2 = min(diag(Kq)) - (2*sqrt(2)*Kv*bar_u_t*delta_mu_d+2*gamma_theta1*gamma_q*(Kv+k_theta))/delta_mu_d^2 ...
         - Kv^2/(2*esp1) - delta_1^2 / (2*gamma_q*esp2);
    c3 = min(diag(Gamma_v)) - gamma_q * Kv * esp1 / delta_mu_d^2;

    c1
    c2
    c3
end


%% plotter
function plotter_quaternion(t, r, dydt, y, inputs, outputs, projectpath, foldername, filename)    
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
    eta = inputs(:, 7);
    xi = inputs(:, 8);

    thrust = outputs(:, 1:3);
    B_M = outputs(:, 4:6);
    traj = reshape(outputs(:, 7:18), [length(outputs), 3, 4]);
    traj = permute(traj, [1, 3, 2]);
    Q_d = outputs(:, 19:22);
    beta = outputs(:, 23:25);
    tilde_mu = outputs(:, 26:28);
    theta_a = outputs(:, 29:31);
    theta_b = outputs(:, 32:34);

    theta1 = y(:, 18:20);
    theta2 = y(:, 21:23);
    theta3 = y(:, 24:26);

    % Rotational
    dW = dydt(:, 1:3);
    W = y(:, 1:3);        % Angular velocity
    Qs = y(:, 4:7);       % Orientation
    eulZXY = Qs(:, 2:4);  % Euler angles
    attitude_d = Q_d(:, 2:4);
    R = zeros([length(Qs) 3 3]);
    for i=1:length(r)
        R(i, :, :) = Q2R(Qs(r(i), :));
    end
    
    % Translational
    ddP = dydt(:, 8:10);
    dP = y(:, 8:10);
    P = y(:, 11:13);
    CoP = P(:, 1:3);

    key = {'projectpath', 'foldername', 'filename', 'lineStyle', 'markerStyle'};
    value = {projectpath, foldername, filename, lineStyle, markerStyle};
    options = containers.Map(key, value);

    plot_state(t, P, dP, traj, W, beta, eulZXY, attitude_d, options);
    plot_error(t, P, dP, traj, W, beta, eulZXY, attitude_d, tilde_mu, options);
    plot_command(t, Tf, u, options);
    plot_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, theta1, theta_a, options);
    plot_estimation(t, theta1, theta2, theta3, theta_a, theta_b, options);
    %plot_torque(t, B_M_f, B_M_d, B_M_g, B_M_a, options);
    plot_motor_command(t, w_m1, w_m2, xi, eta, xi, eta, options);
    plot_3d(t, r, P, CoP, traj, thrust, R, options);
    plot_animation(t, r, P, traj, options, R);
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

%% ControllerTiltedBody
function [w_m1, w_m2, alpha, attitude_d] = ControllerTiltedBody(t, params, traj, p, v, Q, w, xi, eta)
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
    w_m1 = 198.2746; % Upper rotor speed
    w_m2 = 198.2746; % Lower rotor speed
    alpha = [0; 0];
    attitude_d = [0, 0, 5*pi/180];
end

%% ControllerDragTorque
function [Tf, Td, alpha, attitude_d] = ControllerDragTorque(t, params, traj, p, v, Q, w, xi, eta)
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

%% ControllerTiltedRotor
function [w_m1, w_m2, alpha, attitude_d] = ControllerTiltedRotor(t, params, traj, p, v, Q, w, xi, eta)
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
    w_m1 = 198.2746; % Upper rotor speed
    w_m2 = 198.2746; % Lower rotor speed
    Tf = rho * w_m1^2 * prop_d^4 * CT_u + rho * w_m2^2 * prop_d^4 * CT_l;

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
close all;

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
zeta = [cos(w0*ts); sin(w0*ts); 0.1*ts; 0.1*ts];
%zeta = [cos(w0*ts); sin(w0*ts); 1; 0];
d_zeta = diff(zeta);
dd_zeta = diff(d_zeta);
desire = [zeta d_zeta dd_zeta];

%% Command generation
% Initial conditions
%                   x y z
initial_position = [1 0 0];
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
inputs = zeros([length(y) 4+4]);
outputs = zeros([length(y) 25+12+3]);
fprintf("] \nForward: [");
progress = 0;
rng('default')
for i=1:length(y)
    [dydt(i, :), inputs(i, :), outputs(i, :)] = drone_fly(t(i), y(i, :)');
end
fprintf("] \n");

%% Plot
r = 1:40:length(y);
%r = 1:4:length(y);

projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0606_birotor\\model_verification\\';
foldername = 'test\\';
filename = 'birotor_veri';
plotter(t, r, dydt, y, inputs, outputs);

%% Functions
function [dydt, inputs, outputs] = drone_fly(t, y)
    %% Parameters
    [key, params] = get_params();
    g = params('g');     % gravity
    
    % Drone
    m_a = params('m_a');     % Mass, Kg
    m_fm = params('m_fm');
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
    r_fm = params('r_fm'); % Leverage length from c.fm. to c.g. 

    I_fm = params('I_fm'); % Body Inertial
    I_a  = params('I_a'); % Actuator Inertial

    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient

    mKp = params('mKp');   % Servo motor gain
    mKd = params('mKd');   % Servo motor gain

    %% State variables
    W = y(1:3);
    Q = reshape(y(4:12), [3 3]); % 3x3
    dP = y(13:15);
    P = y(16:18);
    vartheta = y(19:22);

    %% Control input
    global T progress;

    traj = TrajectoryPlanner(t);
    %traj = zeros([4, 3]);
    %[T_f, T_d, alpha, attitude_d] = Controller(t, params, traj, P, dP, Q, W, xi, eta);
    [w_m1, w_m2, vartheta_d, attitude_d] = Controller2(t, params, traj, P, dP, Q, W, vartheta(2), vartheta(1));
    
    %[T_f, T_d, alpha, attitude_d] = ControllerTiltedBody(t, params, traj, P, dP, Q, xi, eta);
    %[T_f, T_d, alpha, attitude_d] = ControllerDragTorque(t, params, traj, P, dP, Q, xi, eta);
    %[T_f, T_d, alpha, attitude_d] = ControllerTiltedRotor(t, params, traj, P, dP, Q, xi, eta);
    %[T_f, T_d, alpha, attitude_d] = ControllerSinsoid(t, params, traj, P, dP, Q, xi, eta);
    %[w_m1, w_m2, vartheta_d, attitude_d] = ControllerTiltedRotor2(t, params, traj, P, dP, Q, vartheta(2), vartheta(1));
    %[w_m1, w_m2, vartheta_d, attitude_d] = ControllerGyro(t, params, traj, P, dP, Q, vartheta(2), vartheta(1));

    
    % Servo motor dynamics
    d_vartheta = [0 0 1 0;
                 0 0 0 1;
                 -mKp 0 -mKd 0;
                 0 -mKp 0 -mKd] * vartheta + [0 0;0 0;mKp 0; 0 mKp] * vartheta_d;

    eta = vartheta(1);
    xi = vartheta(2);
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

    %thrust_noise = (rand([3 1]) - 0.5) * 1e-3;
    thrust_noise = 0;
    %torque_noise = (rand([3 1]) - 0.5) * 0;
    torque_noise = 0;
    % Thrust torque
    thrust = B_R_A * [0; 0; T_f] + thrust_noise;
    B_M_f = cross(r_pg, thrust);

    % Drag torque from motor
    B_M_d = B_R_A * [0; 0; T_d];

    % Variable Inertia
    I_b = B_I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];

    %% Newton-Euler equation
    B_M = -cross(W, I_b * W) + B_M_f + B_M_d - B_M_g - B_M_a + torque_noise;

    dW = I_b \ B_M;
    dQ = reshape(Q * skew(W), [9 1]);

    I_thrust = Q * thrust/(m_a + m_fm);

    ddP = [0; 0; -g] + I_thrust;
    dydt = [u_t; u; dW; dQ; ddP; dP; d_vartheta];
    inputs = [w_m1, w_m2, vartheta_d];
    desire = reshape(traj', [12, 1]);

    beta = [0;0;0];
    meta = [B_M_f; + B_M_d; - B_M_g; - B_M_a];
    outputs = [I_thrust ; u; desire; attitude_d; beta; [0;0;0]; meta];
    
    % Progress 
    current = 40 * t / T;
    if current > progress
        progress = ceil(current);
        fprintf('=')
    end
end

function r = Rx(t)
    r = [1 0      0;
         0 cos(t) -sin(t);
         0 sin(t) cos(t)];
end

function r = Rz(t)
    r = [cos(t)  -sin(t) 0;
         sin(t) cos(t)  0;
         0       0       1];
end

function r = Ry(t)
    r = [cos(t)  0 sin(t);
         0       1 0; 
         -sin(t) 0 cos(t)];
end

function X = skew(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end

function traj = TrajectoryPlanner(t)
    global ts
    global desire
    traj = double(subs(desire, ts, t));
    %traj = desire;
end

function [Tf, Td, alpha, attitude_d] = Controller(t, params, traj, p, v, R, w, xi, eta)
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
    Ko = diag([1 1 1]) * 0.01;
    M_d = - Kr * eR - Ko * eOmega;

    % Control allocator
    Td = -M_d(3);
    A = [l*Tf Td; -Td l*Tf];
    alpha = -A \ [M_d(1); M_d(2)];
end

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

    %% Motor commands
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

function [Tf, Td, alpha, attitude_d] = ControllerTiltedBody(t, params, traj, p, v, R, xi, eta)
    %% Parameters
    g = params('g');     % gravity
    rho = params('rho');   % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u');   % upper propeller drag coefficient
    CP_l = params('CP_l');   % lower propeller drag coefficient

    %% Control input
    % Thrust command
    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed

    % Translational thrust
    Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    % Drag torque
    Td = rho * w_prop_u^2 * prop_d^5 * CP_u - rho * w_prop_l^2 * prop_d^5 * CP_l;

    alpha = [0; 0];
    attitude_d = [0, 0, 5*pi/180];
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

    %% Control input
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

    %% Control input
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
    %% Control input
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

    %% Control input
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

    %% Control input
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
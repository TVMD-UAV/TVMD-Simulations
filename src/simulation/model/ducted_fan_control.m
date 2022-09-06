close all;
rng('default')

addpath('model')
addpath('viz')
addpath('../helper_functions')

%% Simulation parameters
global ts desire T progress;
T = 50;
progress = 0;

% Initial conditions
y0 = zeros([13+9 1]);
%Q2 = theta_vector2Q(pi, [1;0;0]);
y0(4:7) = [0; 1; 0; 0]; % Initial orientation
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

dydt = zeros([length(y) 13+9]);
inputs = zeros([length(y) 4]);
outputs = zeros([length(y) 25+3+6]);
fprintf("] \nForward: [");
progress = 0;
rng('default')
for i=1:length(y)
    [dydt(i, :), inputs(i, :), outputs(i, :)] = drone_fly(t(i), y(i, :)');
end
fprintf("] \n");

%% Plot
projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Courses\\110-2\\AdaptiveControl\\FinalProject\\Simulations\\';
foldername = 'test\\';
filename = 'PositionRegulation';

interval = floor(length(y) / (T * 10));
fprintf("Ploting interval: %d\n", interval);
r = 1:interval:length(y);

plotter_quaternion(t, r, dydt, [y zeros([length(y) 9])], inputs, outputs, ...
    projectpath, foldername, filename);

%% Functions
function [dydt, inputs, outputs] = drone_fly(t, y)
    %% Parameters
    [key, params] = get_ducted_fan_params();

    %% Control input
    global T progress;

    %% Controller
    %traj = TrajectoryPlanner(t);
    traj = zeros([4, 5]);
    [Tf, u, attitude_d, beta, tilde_mu] = Controller(t, params, traj, y);
    us = [Tf; u];
    [dydt_m, commands, meta] = ducted_fan_model(params, us, y);
    dydt = [dydt_m; zeros([9 1])];

    %% Newton-Euler equation
    Q = y(4:7);
    R = Q2R(Q);
    I_thrust = -R * [0; 0; Tf];
    
    inputs = [Tf; u];
    desire = reshape(traj(1:4, 1:3)', [12, 1]);
    outputs = [I_thrust ; u; desire; attitude_d; beta; tilde_mu; meta(4:9)];
    
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

%% Controllers
function [Tf, u, attitude_d, beta, tilde_mu] = Controller(t, params, traj, y)
    %% States 
    W = y(1:3);
    Q = y(4:7);
    v = y(8:10);
    p = y(11:13);  

    %% Parameters
    g = params('g');
    m = params('m');     % Mass, Kg
    I_b = params('I_b'); % Body Inertial

    %% Gains
    Kp = params('Kp');
    Kv = params('Kv');
    Kq = params('Kq');
    Kw = params('Kw');
    Gamma_v = params('Gamma_v');
    gamma_q = params('gamma_q');
    
    gamma_theta = 0;
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
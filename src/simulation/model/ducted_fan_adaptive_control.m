close all;
rng('default')

addpath('model')
addpath('viz')
addpath('helper_functions')

[keys, params] = get_ducted_fan_params();
parameter_chack(params);

%% Simulation parameters
global ts desire T progress;
T = 10;
progress = 0;

% Initial conditions
y0 = zeros([13+9 1]);
%Q2 = theta_vector2Q(pi, [1;0;0]);
y0(4:7) = [0; 1; 0; 0]; % Initial orientation
%y0(4:7) = Q2;
%y0(11:13) = [-20 -10 -20];    % Initial position
y0(11:13) = [50 10 0];    % Initial position
%y0(11:13) = [0 0 0];    % Initial position
y0(8:10) = [0 0 0];    % Initial velocity

% Reference
syms ts
zeta = [10*ts; 30*sin(0.1*ts+3.48); 20*sin(0.1*ts+4.71); 0*ts];
%zeta = [0*ts; 0*ts; 0*ts; 0*ts];
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

interval = floor(length(y) / (T * 10));
fprintf("Ploting interval: %d\n", interval);
r = 1:interval:length(y);

plotter_quaternion(t, r, dydt, y, inputs, outputs, ...
    projectpath, foldername, filename);

%% Functions
function [dydt, inputs, outputs] = drone_fly(t, y)
    %% Parameters
    [key, params] = get_ducted_fan_params();

    %% Control input
    global T progress;

    %% Controller
    traj = TrajectoryPlanner(t);
    %traj = zeros([4, 5]);
    [Tf, u, attitude_d, beta, tilde_mu, adaptive] = Controller(t, params, traj, y);
    us = [Tf; u];
    [dydt_m, commands, meta] = ducted_fan_model(params, us, y);
    dydt = [dydt_m; adaptive];

    Q = y(4:7);
    I_R_B = Q2R(Q);
    I_thrust = -I_R_B * [0; 0; Tf];
    
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
function [Tf, u, attitude_d, beta, tilde_mu, adaptive] = Controller(t, params, traj, y)
    %% States 
    W = y(1:3);
    Q = y(4:7);
    v = y(8:10);
    p = y(11:13);  
    theta1 = y(14:16);
    theta2 = y(17:19);
    theta3 = y(20:22);

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
    k_theta = params('k_theta');

    % Adaptive laws
    % theta1
    gamma_theta1 = params('gamma_theta1');
    gamma_theta2 = params('gamma_theta2');
    gamma_theta3 = params('gamma_theta3');
    delta_a = params('delta_a');
    delta_b = params('delta_b');

    K1 = Kp * inv(Gamma_v);
    K2 = (Kv + k_theta) * eye(3);

    %% Errors
    tilde_p = p - traj(1:3, 1);
    tilde_v = v - traj(1:3, 2);

    %% Desired acceleration
    mu_d = traj(1:3, 3) - Kp*inv(Gamma_v)*h(tilde_p) - (Kv+k_theta)*h(tilde_v) - theta1;

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
    R = Q2R(Q);
    tilde_Q = quaternion_multiplication(quaternion_inverse(Q_d), Q);
    bar_q = skew(z_hat)*tilde_Q(2:4) + tilde_Q(1)*z_hat;

    tau2 = Gamma_v*tilde_v + k_theta/gamma_theta1*Kp*phi_h(tilde_v)/Gamma_v*h(tilde_p) + ...
           k_theta*Kv/gamma_theta1*phi_h(tilde_v)*h(tilde_v) + ...
           (gamma_q*(k_theta+Kv)*phi_h(tilde_v)*M' - 2*u_t*k_theta/gamma_theta1*phi_h(tilde_v)*R*skew(bar_q)) * tilde_Q(2:4);

    omega_beta = Kp * Kv * phi_h(tilde_v) / Gamma_v * h(tilde_p) - ...
        gamma_theta1 * leakage(theta1, delta_a+k_theta, tau2, delta_a);
    beta = M * (traj(1:3, 4) + omega_beta) - Kq * tilde_Q(2:4);
    %beta = - Kq * tilde_Q(2:4);
    tilde_W = W - beta;

    % fb
    % fb2
    W1 = -2*u_t*skew(bar_q)*R';
    tilde_mu = W1' * tilde_Q(2:4);
    W2 = Kv^2 * phi_h(tilde_v);
    W3 = -gamma_theta1 * gamma_q * (k_theta + Kv) * phi_h(tilde_v) * M' - Kv * phi_h(tilde_v) * W1';
    W4 = -gamma_theta2 * Gamma_v - Kp * Gamma_v \ phi_h(tilde_p);
    Z1 = get_Z1(params, mu_d, traj(1:3, 4) + omega_beta, M, u_t);
    
    f_tilde_v1 = -K1 * h(tilde_p) - Kv * h(tilde_v) + tilde_mu;
    f_tilde_v2 = -K1 * h(tilde_p) - K2 * h(tilde_v) + tilde_mu - theta1;
    
    % f_mu_d
    f_mu_d = traj(1:3, 4) + omega_beta + ...
        (W2 + k_theta*K2*phi_h(tilde_v))*h(tilde_v) + ...
        W3 * tilde_Q(2:4) + ...
        W4 * tilde_v + ...
        K2*phi_h(tilde_v)*theta1;
    bar_f_mu_d = -K2 * phi_h(tilde_v);
    
    % f_tilde_mu
    f_tilde_mu = -(eye(3) + R*z_hat*(mu_d-[0;0;g])'/u_t)*f_mu_d - u_t*R*skew(W)*z_hat;
    bar_f_tilde_mu = -(eye(3) + R*z_hat*(mu_d-[0;0;g])'/u_t)*bar_f_mu_d;
    
    % f_tilde_q
    f_tilde_q = 0.5*(tilde_Q(1)*eye(3) + skew(tilde_Q(2:4))) * W + ...
                0.5*(skew(tilde_Q(2:4))-tilde_Q(1)*eye(3))*(M*f_mu_d);
    bar_f_tilde_q = 0.5*(skew(tilde_Q(2:4))-tilde_Q(1)*eye(3)) * M * bar_f_tilde_mu;

    % f_tau2
    Z2 = get_Z2(params, mu_d, tilde_Q(2:4), M, u_t);
    f_tau2 = Gamma_v*f_tilde_v2 - k_theta/gamma_theta1*(f_phi_h(tilde_v, f_tilde_v1)*f_tilde_v2 ...
            - phi_h(tilde_v)*K1*phi_h(tilde_p) *tilde_v ...
            - Kv*phi_h(tilde_v)^2*f_tilde_v2 + phi_h(tilde_v)*f_tilde_mu) ...
            + gamma_q*(f_phi_h(tilde_v, K2*M'*tilde_Q(2:4))*f_tilde_v2 + phi_h(tilde_v)*K2*Z2*f_mu_d ...
                + phi_h(tilde_v)*K2*M'*f_tilde_q);
    bar_f_tau2 = Gamma_v - k_theta/gamma_theta1*( f_phi_h(tilde_v, f_tilde_v1) ...
                - Kv^2*phi_h(tilde_v)^2 + phi_h(tilde_v)*bar_f_tilde_mu ) ...
            + gamma_q*( f_phi_h(tilde_v, K2*M'*tilde_Q(2:4))+phi_h(tilde_v)*K2*Z2*bar_f_mu_d ) ...
            + phi_h(tilde_v)*K2*M'*bar_f_tilde_q;

    % Adaptive law 1, theta1
    d_hat_theta1 = gamma_theta1 * (tau2 + leakage(theta1, delta_a + k_theta, tau2, delta_a));

    % f_wb
    [f_alpha, bar_f_alpha] = get_leakage_derivative(theta1, delta_a+k_theta, tau2, d_hat_theta1, f_tau2, bar_f_tau2, delta_a);

    f_wb = Kp*Kv*f_phi_h(tilde_v, Gamma_v\h(tilde_p)) * f_tilde_v2 ...
           + Kp*Kv*phi_h(tilde_v)/Gamma_v*phi_h(tilde_p) * tilde_v ...
           - gamma_theta1*f_alpha;
    bar_f_wb = Kp*Kv*f_phi_h(tilde_v, Gamma_v\h(tilde_p)) - gamma_theta1*bar_f_alpha;

    % fb
    fb2 = Z1 * f_mu_d + M * (traj(1:3, 5) + f_wb) - Kq * f_tilde_q;
    bar_fb2 = Z1 * bar_f_mu_d + M * bar_f_wb - Kq * bar_f_tilde_q;

    %% Control law for torque
    u = -gamma_q * tilde_Q(2:4) + skew(W) * I_b * W  - Kw * tilde_W + ...
        I_b * (fb2 + bar_fb2*theta2) - skew(z_hat)*R'*theta3;
    Tf = u_t;

    attitude_d = Q_d;

    % theta2
    d_hat_theta2 = gamma_theta2 * (-bar_fb2'*I_b*tilde_W + leakage(theta2, delta_a, -bar_fb2'*I_b*tilde_W, delta_a));

    % theta3
    d_hat_theta3 = gamma_theta3 * (-R*skew(z_hat)*tilde_W + leakage(theta3, delta_b, -R*skew(z_hat)*tilde_W, delta_a));
    adaptive = [d_hat_theta1; d_hat_theta2; d_hat_theta3];
end

%% Controller helper functions
function Z1 = get_Z1(params, mu_d, v, M, u_t)
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

    Z1 = M' * v * f_gamma / gamma_M + gamma_M * lambda1;
end

function Z2 = get_Z2(params, mu_d, v, M, u_t)
    g = params('g');
    c1 = u_t + g - mu_d(3);
    gamma_M = 1/(c1 * u_t^2);
    f_gamma = gamma_M^2 * ([0;0;g]-mu_d)' * (3*c1*eye(3) + skew([0;0;1])*skew([0;0;g]-mu_d));
    alpha1 = (mu_d - [0;0;g]) / norm(mu_d - [0;0;g]);
    alpha2 = alpha1 - [0;0;1];

    lambda2 = [(2*v(2)*mu_d(1)-mu_d(2)*v(1)) (u_t*v(3)-mu_d(1)*v(1)) 0;
               (mu_d(2)*v(2)-u_t*v(3)) (mu_d(1)*v(2)-2*mu_d(2)*v(1)) 0;
               -c1*v(2) c1*v(2) 0] + ...
              [mu_d(2)*v(3)-c1*v(2); c1*v(1)-mu_d(1)*v(3); 0] * alpha1' + ...
              [-u_t*v(2); u_t*v(1); mu_d(2)*v(1)-mu_d(1)*v(2)] * alpha2';

    Z2 = M' * v * f_gamma / gamma_M + gamma_M * lambda2;
end

% Adaptive 
function alpha = leakage(hat, delta, tau, delta_a)
    esp = 0.1;
    ka = 1 / (2 * (esp^2+2*esp*delta)^2 * delta^2);
    eta1 = 0;
    if hat' * hat > delta^2
        eta1 = (hat' * hat - delta^2)^2;
    end
    eta2 = hat' * tau + sqrt((hat' * tau)^2 + delta_a^2);
    alpha = - ka * eta1 * eta2 * hat;
end

function [f_alpha, bar_f_alpha] = get_leakage_derivative(hat, delta, tau, d_hat, f_tau2, bar_f_tau2, delta_a)
    esp = 0.1;
    ka = 1 / (2 * (esp^2+2*esp*delta)^2 * delta^2);
    eta1 = 0;
    d_eta1 = 0;
    if hat' * hat > delta^2
        eta1 = (hat' * hat - delta^2)^2;
        d_eta1 = 4 * (hat'*hat - delta^2) * hat' * d_hat;
    end
    eta2 = hat' * tau + sqrt((hat' * tau)^2 + delta_a^2);

    f_alpha = -ka * d_eta1 * eta2 * hat - ka * eta1 * eta2 * d_hat - ...
               ka * eta1 * eta2 / (eta2 - hat'*tau) * (tau' * d_hat + hat' * f_tau2) * hat;
    bar_f_alpha = -ka * eta1 * eta2 / (eta2 - hat'*tau) * hat * hat' * bar_f_tau2;
end

% Parameter Checking
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
    esp1 = 0.1;
    esp2 = 10;
    delta_1 = (2*bar_u_t+sqrt(2)*gamma_q*gamma_theta1/delta_mu_d)*norm(Gamma_v) ...
        + sqrt(2)*gamma_q*Kp/delta_mu_d*norm(inv(Gamma_v));
    
    c2 = min(diag(Kq)) - (2*sqrt(2)*Kv*bar_u_t*delta_mu_d+2*gamma_theta1*gamma_q*(Kv+k_theta))/delta_mu_d^2 ...
         - Kv^2/(2*esp1) - delta_1^2 / (2*gamma_q*esp2);
    c3 = min(diag(Gamma_v)) - gamma_q * Kv * esp1 / delta_mu_d^2;

    c1
    c2
    c3
end
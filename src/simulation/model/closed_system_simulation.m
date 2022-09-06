close all;
rng('default')

addpath('model')
addpath('viz')
addpath('../helper_functions')


%% Simulation parameters
global T progress;
T = 20;
progress = 0;

% Desired trajectory
syms ts
global ts;
global desire

%% Circular trajectory
w0 = 2*pi/10;
%                x,          y, z, psi
zeta = [10*ts; 30*sin(0.1*ts+3.48); 20*sin(0.1*ts+4.71); 0*ts];
%zeta = [cos(w0*ts); sin(w0*ts); 0.1*ts; 0.1*ts];
%zeta = [cos(w0*ts); sin(w0*ts); 1; 0];
d_zeta = diff(zeta);
dd_zeta = diff(d_zeta);
desire = [zeta d_zeta dd_zeta];

% Initial conditions
%                   x y z
initial_position = [10 -10 -10];
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
    %[key, params] = get_birotor_params();
    [key, params] = get_params_lya();

    traj = TrajectoryPlanner(t);
    %traj = RegulationTrajectory();
    [u_t, u_d, attitude_d] = Controller(t, params, traj, y);
    %[u_t, u_d, attitude_d] = Controller_MinimumSnap(t, params, traj, y);
    %[u_t, u_d, attitude_d] = Birotor_PID_Controller(params, traj, y);
    
    u = [u_t; u_d];
    %[dydt, commands, meta] = bi_rotor_model(u, y);
    [dydt, commands, meta] = my_model_so3(params, u, y);
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

    % Attitude error
    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    eOmega = 0 - w;

    % Attitude control
    Kr = diag([1 1 1]) * 0.1;
    Ko = diag([1 1 1]) * 0.1;
    M_d = - Kr * eR - Ko * eOmega;

    u = -M_d;
end

%% Controller2
function [u_t, u, attitude_d] = Controller(t, params, traj, y)
    m_a = params('m_a');     % Mass, Kg
    m_fm = params('m_fm');
    m = m_a + m_fm;
    g = params('g');
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
    l = r_pg(3);
    
    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);
    eta = y(19);
    xi = y(20);

    % Gain
    Kp = diag([0.25 1 1]) * 0.2*2;
    Kd = diag([0.25 1 1]) * 0.5*2;

    dd_zeta_com = traj(1:3, 3) + Kd * (traj(1:3, 2) - v) + Kp * (traj(1:3, 1) - p);
    psi_d = traj(4, 1);

    psi = atan2(-R(1, 2), R(2, 2));
    psi = psi_d;

    % Trajectory control
    Tf = (m * dd_zeta_com(3) + m*g) / (cos(eta) * cos(xi));
    tp = ([cos(psi) sin(psi); sin(psi) -cos(psi)]' * ...
        [m * dd_zeta_com(1) / Tf - xi*cos(psi) - eta*sin(psi); ...
         m * dd_zeta_com(2) / Tf - xi*sin(psi) + eta*cos(psi)]);

    theta_d = tp(1);
    phi_d = tp(2);

    attitude_d = [psi_d, phi_d, theta_d];

    % Attitude error
    R_d = getI_R_B(psi_d, phi_d, theta_d);
    
    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    eOmega = 0 - w;

    % Attitude control
    Kr = diag([0.25 1 1]) * 0.1*2;
    Ko = diag([0.25 1 1]) * 0.05*2;
    M_d = - Kr * eR - Ko * eOmega;

    u_t = Tf / m;
    u = M_d;
end

function [u_t, u, attitude_d] = Controller_MinimumSnap(t, params, traj, y)
    m = params('m');     % Mass, Kg
    g = params('g');

    % States
    p = y(16:18);
    v = y(13:15);
    R = reshape(y(4:12), [3 3]); % 3x3
    w = y(1:3);
    
    % Gain
    Kp = diag([1 1 1]) * 0.02;
    Kd = diag([1 1 1]) * 0.05;

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

    % Attitude error
    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = [eRx(2, 3); eRx(3, 1); eRx(1, 2)];
    eOmega = 0 - w;

    % Attitude control
    Kr = diag([1 1 1]) * 0.01;
    Ko = diag([1 1 1]) * 0.05;
    M_d = - Kr * eR - Ko * eOmega;

    u = M_d;
end
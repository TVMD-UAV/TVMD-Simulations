close all;
rng('default')

%% Simulation parameters
global ts desire T progress;
T = 50;
progress = 0;

% Initial conditions
y0 = zeros([13 1]);
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
desire = [zeta d_zeta dd_zeta ddd_zeta];

% Solver
fprintf("Solver : [");
options = odeset('RelTol',1e-5,'AbsTol',1e-7);
[t, y] = ode45(@drone_fly, [0 T], y0, options);

dydt = zeros([length(y) 13]);
inputs = zeros([length(y) 4]);
outputs = zeros([length(y) 25+3]);
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
plotter_quaternion(t, r, dydt, y, inputs, outputs);

%% Functions
function [dydt, inputs, outputs] = drone_fly(t, y)
    %% Parameters
    [key, params] = get_ducted_fan_params();
    g = params('g');     % gravity
    
    % Drone parameters
    m = params('m');   % Mass, Kg
    I_b  = params('I_b');  % Actuator Inertial

    %% State variables
    W = y(1:3);
    Q = y(4:7);
    dP = y(8:10);
    P = y(11:13);

    I_R_B = Q2R(Q);

    %% Control input
    global T progress;

    %% Controller
    traj = TrajectoryPlanner(t);
    %traj = zeros([4, 3]);
    [Tf, u, attitude_d, beta, tilde_mu] = Controller(t, params, traj, P, dP, Q, W);

    %% Newton-Euler equation
    I_thrust = -I_R_B * [0; 0; Tf];
    B_M = -cross(W, I_b * W) + u;

    ddP = [0; 0; g] + I_thrust;
    dW = I_b \ B_M;
    dQ = 0.5 * [-Q(2:4)'; 
                Q(1)*eye(3) + skew(Q(2:4))] * W;

    dydt = [dW; dQ; ddP; dP];
    inputs = [Tf; u];
    desire = reshape(traj(1:4, 1:3)', [12, 1]);
    outputs = [I_thrust ; B_M; desire; attitude_d; beta; tilde_mu];
    
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
function [Tf, u, attitude_d, beta, tilde_mu] = Controller(t, params, traj, p, v, Q, W)
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
    gamma_theta = params('gamma_theta');
    k_theta = params('k_theta');

    K1 = Kp * inv(Gamma_v);
    K2 = (Kv + k_theta) * eye(3);

    %% Errors
    tilde_p = p - traj(1:3, 1);
    tilde_v = v - traj(1:3, 2);

    %% Desired acceleration
    mu_d = traj(1:3, 3) - (Kv + k_theta) * h(tilde_v) - Kp * Gamma_v \ h(tilde_p);

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
    W2 = Kv^2 * phi_h(tilde_v);
    W3 = -gamma_theta * gamma_q * (k_theta + Kv) * phi_h(tilde_v) * M' - Kv * phi_h(tilde_v) * W1';
    W4 = -gamma_theta * Gamma_v - Kp * Gamma_v \ phi_h(tilde_p);
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
    fb2 = Z1 * f_mu_d + M * (traj(1:3, 4) + f_wb) - Kq * f_tilde_q;

    %% Control law for torque
    u = -gamma_q * tilde_Q(2:4) + skew(W) * I_b * W  - Kw * tilde_W;% + I_b * fb2;
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

function plotter_quaternion(t, r, dydt, y, inputs, outputs)    
    projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Courses\\110-2\\AdaptiveControl\\FinalProject\\Simulations\\DisturbanceFree\\';
    foldername = 'PositionRegulation_t50\\';
    filename = 'PositionRegulation';

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

    thrust = outputs(:, 1:3);
    B_M = outputs(:, 4:6);
    traj = reshape(outputs(:, 7:18), [length(outputs), 3, 4]);
    traj = permute(traj, [1, 3, 2]);
    Q_d = outputs(:, 19:22);
    beta = outputs(:, 23:25);
    tilde_mu = outputs(:, 26:28);
    detailed = true;

    % Rotational
    dW = dydt(:, 1:3);
    W = y(:, 1:3);        % Angular velocity
    Qs = y(:, 4:7);       % Orientation
    eulZXY = Qs(:, 2:4);  % Euler angles
    attitude_d = Q_d(:, 2:4);
    R = zeros([length(Qs) 3 3]);
    for i=1:length(Qs)
        R(i, :, :) = Q2R(Qs(i, :));
    end
    
    % Translational
    ddP = dydt(:, 8:10);
    dP = y(:, 8:10);
    P = y(:, 11:13);

    CoP = P(:, 1:3);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw 3D view
    figure(1)
    scatter3(P(:, 1), P(:, 2), P(:, 3), 40, t); hold on
    quiver3(CoP(:, 1), CoP(:, 2), CoP(:, 3), thrust(:, 1), thrust(:, 2), thrust(:, 3), 'magenta')
    
    % Draw coordinates of the agent
    q1 = quiver3(P(:, 1), P(:, 2), P(:, 3), R(:, 1, 1), R(:, 2, 1), R(:, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(:, 1), P(:, 2), P(:, 3), R(:, 1, 2), R(:, 2, 2), R(:, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(:, 1), P(:, 2), P(:, 3), R(:, 1, 3), R(:, 2, 3), R(:, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';

    % Draw desired trajectory
    if detailed
        scatter3(traj(:, 1, 1), traj(:, 2, 1), traj(:, 3, 1), "red", 'Marker','.'); hold on
    end
    
    % Draw agent body
    for i=1:length(r)
        draw_agent_quad(P(r(i), :), R(r(i), :, :), 1+256*t(r(i))/t(end)); hold on
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    colorbar
    set(gca,'DataAspectRatio',[1 1 1])
    saveas(gcf, strcat(projectpath, foldername, filename, '_3d.png'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_3d.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_3d.fig'));

    %% Draw animation
    figure('Position', [10 10 1200 1200])
    set(gca,'DataAspectRatio',[1 1 1])
    animation_name = strcat(projectpath, foldername, filename, '_3d.gif');

    scatter3(P(:, 1), P(:, 2), P(:, 3), 'Color', 'none'); hold on
    scatter3([0 max(P(:, 1))+2], [0 max(P(:, 2))+2], [0 max(P(:, 3))+2], 'Color', 'none'); hold on
    ss_traj = scatter3(traj(1, 1, 1), traj(1, 2, 1), traj(1, 3, 1), "red", 'Marker','.'); hold on
    ss_state = scatter3(P(1, 1), P(1, 2), P(1, 3), "blue"); hold on
    patch_obj = draw_agent_quad(P(r(1), :), R(r(1), :, :), 1+256*t(r(1))/t(end)); hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    for i=1:length(r)
        ss_traj.XData = traj(1:r(i), 1, 1);
        ss_traj.YData = traj(1:r(i), 2, 1);
        ss_traj.ZData = traj(1:r(i), 3, 1);

        ss_state.XData = P(1:r(i), 1);
        ss_state.YData = P(1:r(i), 2);
        ss_state.ZData = P(1:r(i), 3);

        draw_agent_quad_animation(patch_obj, P(r(i), :), R(r(i), :, :), 1+256*t(r(i))/t(end));
        
        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i == 1
            imwrite(imind,cm,animation_name,'gif', 'Loopcount',inf, 'DelayTime',0.1);
        else
            imwrite(imind,cm,animation_name,'gif', 'WriteMode', 'append', 'DelayTime',0.1);
        end
    end

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw states
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw orientation
    figure('Position', [10 10 800 800])
    Ax = subplot(4, 1, 1);
    plot(t, eulZXY(:, 1),'DisplayName','Yaw $$\psi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, eulZXY(:, 2),'DisplayName','Roll $$\phi$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, eulZXY(:, 3),'DisplayName','Pitch $$\theta$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    % Draw desired trajectory
    if detailed
        plot(t, attitude_d(:, 1),'DisplayName','Yaw $$\psi_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, attitude_d(:, 2),'DisplayName','Roll $$\phi_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, attitude_d(:, 3),'DisplayName','Pitch $$\theta_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    ylabel('rad')
    xlabel('time')
    ylim([-0.5 0.5])
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw angular velocity
    %figure('Position', [10 10 800 400])
    Ax = subplot(4, 1, 2);
    plot(t, W(:, 3),'DisplayName','Yaw $$\omega_z$$','LineWidth',2); hold on 
    plot(t, W(:, 2),'DisplayName','Roll $$\omega_y$$','LineWidth',2); hold on 
    plot(t, W(:, 1),'DisplayName','Pitch $$\omega_x$$','LineWidth',2); hold on 
    if detailed
        plot(t, beta(:, 1),'DisplayName','Yaw $${\omega_x}_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, beta(:, 2),'DisplayName','Roll $${\omega_y}_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, beta(:, 3),'DisplayName','Pitch $${\omega_z}_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    ylabel('rad')
    xlabel('time')
    ylim([-2 2])
    title('Angular velocity w.r.t. body frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw positions
    Ax = subplot(4, 1, 3);
    plot(t, P(:, 1),'DisplayName','$$P_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, P(:, 2),'DisplayName','$$P_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, P(:, 3),'DisplayName','$$P_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 

    % Draw desired trajectory
    if detailed
        plot(t, traj(:, 1, 1),'DisplayName','$$P_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 2, 1),'DisplayName','$$P_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 3, 1),'DisplayName','$$P_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    
    ylabel('m')
    xlabel('time')
    title('Position')
    hl = legend('show');
    set(hl, 'Interpreter','latex')  
    %% Draw velocity
    Ax = subplot(4, 1, 4);
    plot(t, dP(:, 1),'DisplayName','$$V_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, dP(:, 2),'DisplayName','$$V_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, dP(:, 3),'DisplayName','$$V_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 

    % Draw desired trajectory
    if detailed
        plot(t, traj(:, 1, 2),'DisplayName','$$V_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 2, 2),'DisplayName','$$V_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 3, 2),'DisplayName','$$V_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    ylabel('m/s')
    xlabel('time')
    ylim([-10 10])
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    sgtitle('State profile')
    saveas(gcf, strcat(projectpath, foldername, filename, '_state.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_state.fig'));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw errors
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Draw orientation
    figure('Position', [10 10 800 1000])
    Ax = subplot(5, 1, 1);
    plot(t, attitude_d(:, 1) - eulZXY(:, 1),'DisplayName','Yaw $$\psi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, attitude_d(:, 2) - eulZXY(:, 2),'DisplayName','Roll $$\phi$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, attitude_d(:, 3) - eulZXY(:, 3),'DisplayName','Pitch $$\theta$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('rad')
    xlabel('time')
    ylim([-0.1 0.1])
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw angular velocity
    Ax = subplot(5, 1, 2);
    plot(t, W(:, 1) - beta(:, 1),'DisplayName','Yaw $$\omega_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, W(:, 2) - beta(:, 2),'DisplayName','Roll $$\omega_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, W(:, 3) - beta(:, 3),'DisplayName','Pitch $$\omega_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('rad/s')
    xlabel('time')
    ylim([-0.2 0.2])
    title('Angular velocity')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw position
    Ax = subplot(5, 1, 3);
    plot(t, traj(:, 1, 1) - P(:, 1),'DisplayName','$$P_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 1) - P(:, 2),'DisplayName','$$P_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 1) - P(:, 3),'DisplayName','$$P_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m')
    xlabel('time')
    title('Position')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw velocity
    Ax = subplot(5, 1, 4);
    plot(t, traj(:, 1, 2) - dP(:, 1),'DisplayName','$$V_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 2) - dP(:, 2),'DisplayName','$$V_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 2) - dP(:, 3),'DisplayName','$$V_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s')
    xlabel('time')
    ylim([-10 10])
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    sgtitle('Error profile')
    %% Draw acceleration
    Ax = subplot(5, 1, 5);
    plot(t, tilde_mu(:, 1),'DisplayName','$$\mu_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, tilde_mu(:, 2),'DisplayName','$$\mu_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, tilde_mu(:, 3),'DisplayName','$$\mu_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s^2')
    xlabel('time')
    ylim([-2 2])
    title('Acceleration w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    sgtitle('Error profile')
    saveas(gcf, strcat(projectpath, foldername, filename, '_error.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_error.fig'));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw torque
    figure(6)
    yyaxis left
    plot(t, u(:, 1), 'LineStyle','-', 'DisplayName','u_x', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, u(:, 2), 'LineStyle','-', 'DisplayName','u_y', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, u(:, 3), 'LineStyle','-', 'DisplayName','u_z', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('Nm')
    ylim([-10 4])
    
    yyaxis right
    plot(t, Tf, 'LineStyle','-', 'DisplayName','u_t', 'Color', '#000000','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    ylim([8 16])
    title('Command profile')
    legend()
    saveas(gcf, strcat(projectpath, foldername, filename, '_command.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_command.fig'));
end

%% Drawing
function p = draw_agent_quad(P, R, ci)
    R = squeeze(R);
    radius = 0.5;
    height = 0.2;
    k=1:8;
    kk = k*pi/2;
    vertices = zeros(8, 3);
    vertices(:, 1) = radius*cos(kk);
    vertices(:, 2) = radius*sin(kk);
    vertices(1:4, 3) = height;
    vertices(5:8, 3) = -height;
    vertices = P + vertices * R';

    % Colormap
    cmap = parula(256);
    uint8(ci);
    c = cmap(uint8(ci), :);

    k=1:4;
    face = ones(4,1) * [0 4 5 1] + k';
    face(4, 3) = 5;
    face(4, 4) = 1;
    p = patch('Vertices',vertices,'Faces',face, 'EdgeColor',c,'FaceColor','none','LineWidth',1);
    alpha(0.3);
end

function draw_agent_quad_animation(patch_obj, P, R, ci)
    R = squeeze(R);
    radius = 0.5;
    height = 0.2;
    k=1:8;
    kk = k*pi/2;
    vertices = zeros(8, 3);
    vertices(:, 1) = radius*cos(kk);
    vertices(:, 2) = radius*sin(kk);
    vertices(1:4, 3) = height;
    vertices(5:8, 3) = -height;
    vertices = P + vertices * R';

    % Colormap
    cmap = parula(256);
    uint8(ci);
    c = cmap(uint8(ci), :);

    k=1:4;
    face = ones(4,1) * [0 4 5 1] + k';
    face(4, 3) = 5;
    face(4, 4) = 1;
    patch_obj.Vertices = vertices;
    patch_obj.Faces = face;
    patch_obj.EdgeColor = c;
    alpha(0.3);
end

%% Math tools
function X = skew(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end

% Quaternion toolbox
function Q2 = quaternion_inverse(Q1)
    Q2 = Q1;
    Q2(2:4) = -Q2(2:4);
end

function D = quaternion_multiplication(Q, P)
    D = [0; 0; 0; 0];
    D(1) = P(1)*Q(1) - Q(2:4)' * P(2:4);
    D(2:4) = Q(1) * P(2:4) + P(1) * Q(2:4) + skew(Q(2:4))*P(2:4);
end

function R = Q2R(Q)
    R = (eye(3) + 2 * skew(Q(2:4)) * skew(Q(2:4)) - 2 * Q(1) * skew(Q(2:4)))';
end

function [theta, u] = Q2theta_vector(Q)
    theta = 2 * acos(Q(1));
    u = Q(2:4) / sin(theta / 2);
end

function [theta, u] = Q2theta_vector_batch(Q)
    theta = 2 * acos(Q(:, 1));
    u = Q(:, 2:4) / sin(theta / 2);
end

% Bounded function
function y = h(x)
    y = x / sqrt(1 + norm(x)^2);
end

function y = phi_h(x)
    y = (eye(3) - skew(x)^2) / sqrt(1 + norm(x)^2)^3;
end

function y = f_phi_h(u, v)
    y = 3 * (skew(u)^2 - eye(3)) * v * u' / sqrt(1 + norm(u)^2)^5 + ...
        (1 + norm(u)^2) * (2*skew(u)*skew(v)-skew(v)*skew(u));
end


function Q = theta_vector2Q(theta, u)
    Q = [cos(theta/2); sin(theta/2)*u/norm(u) ];
end
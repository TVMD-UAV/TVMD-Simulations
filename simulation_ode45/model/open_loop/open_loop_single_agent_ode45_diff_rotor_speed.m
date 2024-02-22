close all;

%% Simulation parameters
T = 1;
r_pg = [0; 0; 0.03];  % Leverage length from c.p. to c.g.
r_fm = [0; 0; -0.02]; % Leverage length from c.fm. to c.g. 

%% Command generation
% Initial conditions
y0 = zeros([18 1]);
y0(4:12) = reshape(eye(3), [9 1]);

% Solver
options = odeset('RelTol',1e-7,'AbsTol',1e-9);
[t, y] = ode45(@drone_fly, [0 T], y0, options);

dydt = zeros([length(y) 18]);
inputs = zeros([length(y) 2]);
outputs = zeros([length(y) 6]);
for i=1:length(y)
    [dydt(i, :), inputs(i, :), outputs(i, :)] = drone_fly(t(i), y(i, :)');
end

%% Extract parameters
dd_xi = inputs(:, 1);
dd_eta = inputs(:, 2);
thrust = outputs(:, 1:3);
torque = outputs(:, 4:6);

dW = dydt(:, 1:3);
W = y(:, 1:3);

Q = reshape(y(:, 4:12), [length(y) 3 3]); % 3x3

ddP = dydt(:, 13:15);
dP = y(:, 13:15);
P = y(:, 16:18);

% Euler angles
QT = permute(Q, [2 3 1]);
eulZYX = rotm2eul(QT,'ZYX');

%% Plot
r = 1:5:length(y);

%figure(1) 
% scatter3(sB_I_a(:, 1), sB_I_a(:, 2), sB_I_a(:, 3), 40, 1:sampling_rate*T);
% xlabel('x')
% ylabel('y')
% zlabel('z')
% title('Forces')
% colorbar
% set(gca,'DataAspectRatio',[1 1 1])
% 
figure(2)
scatter3(P(:, 1), P(:, 2), P(:, 3), 40, t); hold on
quiver3(P(:, 1), P(:, 2), P(:, 3), thrust(:, 1), thrust(:, 2), thrust(:, 3), 'magenta')

% Draw coordinates of the agent
q1 = quiver3(P(:, 1), P(:, 2), P(:, 3), Q(:, 1, 1), Q(:, 2, 1), Q(:, 3, 1), 0.1, 'red'); hold on
q2 = quiver3(P(:, 1), P(:, 2), P(:, 3), Q(:, 1, 2), Q(:, 2, 2), Q(:, 3, 2), 0.1, 'green'); hold on
q3 = quiver3(P(:, 1), P(:, 2), P(:, 3), Q(:, 1, 3), Q(:, 2, 3), Q(:, 3, 3), 0.1, 'blue'); hold on
q1.ShowArrowHead = 'off';
q2.ShowArrowHead = 'off';
q3.ShowArrowHead = 'off';

% Draw agent body
for i=1:length(y)
    draw_agent(P(i, :), Q(i, :, :)); hold on
end
xlabel('x')
ylabel('y')
zlabel('z')
title('Positions')
colorbar
set(gca,'DataAspectRatio',[1 1 1])


%% Draw torque y-axis
figure(3)

% Draw acceleration and torque
subplot(3, 1, 1)
title('rotating acceleration v.s. gyroscopic moment on y-axis')
yyaxis left
plot(t, dd_xi,'DisplayName','$\ddot{\xi}$'); hold on
ylabel('rad/s^2')

yyaxis right
plot(t, torque(:, 2),'DisplayName','$M_y$')
ylabel('Nm')
hl = legend('show');
set(hl, 'Interpreter','latex')

% Draw euler angles
Ax = subplot(3, 1, 2);
plot(t, eulZYX(:, 2),'DisplayName','Euler')
ylabel('rad')

xlabel('time')
hl = legend('show');
set(hl, 'Interpreter','latex')
PosVec = Ax.Position;
Ax.Position = PosVec+[0 0.07 0 0];

% Draw x-z view
Ax = subplot(3, 1, 3);
q = quiver(P(:, 1), P(:, 3), Q(:, 1, 3), Q(:, 3, 3), 1, 'blue'); hold on 
q.ShowArrowHead = 'off';
q = quiver(P(:, 1), P(:, 3), Q(:, 1, 1), Q(:, 3, 1), 1, 'red'); hold on 
q.ShowArrowHead = 'off';
quiver(P(:, 1), P(:, 3), zeros(length(y), 1), -9.8*ones(length(y), 1), 0.1, 'cyan'); hold on 
%CoP = P(:, 1:3) + squeeze(permute(pagemtimes(permute(Q, [2 3 1]), r_pg), [2 3 1]));
CoP = P(:, 1:3) + squeeze(pagemtimes(permute(Q, [2 3 1]), r_pg))';
quiver(CoP(:, 1), CoP(:, 3), thrust(:, 1), thrust(:, 3), 0.1, 'magenta'); hold on 
text(CoP(r, 1), CoP(r, 3)-0.1, '\uparrow' + string(t(r)) + 's')
xlabel('x(m)')
ylabel('z(m)')
PosVec = Ax.Position;
Ax.Position = PosVec+[-0.25 -0.01 0.5 0.08];
set(gca,'DataAspectRatio',[1 1 1])
legend('z-axis', 'x-axis', 'gravity', 'thrust')

%% Draw torque x-axis
figure(4)
% Draw acceleration and torque
subplot(3, 1, 1)
title('rotating acceleration v.s. gyroscopic moment on x-axis')
yyaxis left
plot(t, dd_eta,'DisplayName','$\ddot{\eta}$'); hold on
ylabel('rad/s^2')

yyaxis right
plot(t, torque(:, 1),'DisplayName','$M_x$')
ylabel('Nm')
hl = legend('show');
set(hl, 'Interpreter','latex')

% Draw euler angles
Ax = subplot(3, 1, 2);
plot(t, eulZYX(:, 3),'DisplayName','Euler')
ylabel('rad')

xlabel('time')
hl = legend('show');
set(hl, 'Interpreter','latex')
PosVec = Ax.Position;
Ax.Position = PosVec+[0 0.07 0 0];

%figure(5)
% Draw y-z view
Ax = subplot(3, 1, 3);
q = quiver(P(:, 2), P(:, 3), Q(:, 2, 3), Q(:, 3, 3), 1, 'blue'); hold on 
q.ShowArrowHead = 'off';
q = quiver(P(:, 2), P(:, 3), Q(:, 2, 2), Q(:, 3, 2), 1, 'green'); hold on 
q.ShowArrowHead = 'off';
quiver(P(:, 2), P(:, 3), zeros(length(y), 1), -9.8*ones(length(y), 1), 0.1, 'cyan'); hold on 

quiver(CoP(:, 2), CoP(:, 3), thrust(:, 2), thrust(:, 3), 0.1, 'magenta'); hold on 
text(CoP(r, 2), CoP(r, 3)-0.1, '\uparrow' + string(t(r)) + 's')
xlabel('y(m)')
ylabel('z(m)')
PosVec = Ax.Position;
Ax.Position = PosVec+[-0.25 -0.01 0.5 0.08];
set(gca,'DataAspectRatio',[1 1 1])
legend('z-axis', 'y-axis', 'gravity', 'thrust')

%% Draw z-axis rotation
figure(5)
Ax = subplot(2, 1, 1);
plot(t, eulZYX(:, 1),'DisplayName','Yaw angle','LineWidth',2)
ylabel('rad')
xlabel('time')
title('Z-axis rotation')
hl = legend('show');
set(hl, 'Interpreter','latex')

Ax = subplot(2, 1, 2);
plot(t, P(:, 3),'DisplayName','Height','LineWidth',2)
ylabel('m')
xlabel('time')
title('Drone height')
hl = legend('show');
set(hl, 'Interpreter','latex')


%% Functions
function [dydt, inputs, outputs] = drone_fly(t, y)
    %% Parameters
    % Environment
    g = 9.818;     % gravity
    rho = 1.225;   % kg/m3
    prop_d = 0.0254*9; % 8 inch = 20.3 cm


    CT_u = 0.020231; % upper propeller thrust coefficient
    CT_l = 0.020231; % lower propeller thrust coefficient
    CP_u = 0.0188;   % upper propeller drag coefficient
    CP_l = 0.0188;   % lower propeller drag coefficient
    
    % Drone
    m_a = 0.2;     % Mass, Kg
    m_fm = 0.3;
    r_pg = [0; 0; 0.03];  % Leverage length from c.p. to c.g.
    r_fm = [0; 0; -0.02]; % Leverage length from c.fm. to c.g. 

    w_prop_u = 190.4374 + 50; % Upper rotor speed
    w_prop_l = 190.4374 - 50; % Lower rotor speed
    
    T_f = (m_a + m_fm) * g + 0;      % Translational thrust
    T_f = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;

    T_d = 0;                         % Drag torque
    T_d = rho * w_prop_u^2 * prop_d^5 * CP_u - rho * w_prop_l^2 * prop_d^5 * CP_l;

    I_fm = [1 0 0; 0 1 0; 0 0 1]; % Body Inertial
    I_a  = [1 0 0; 0 1 0; 0 0 1]; % Actuator Inertial
    
    w0 = pi;
    amp = 10 * pi / 180;
    w_a = 0;

    %% State variables
    W = y(1:3);
    Q = reshape(y(4:12), [3 3]); % 3x3
    dP = y(13:15);
    P = y(16:18);

    %% Generate input signal
%     xi = sin(w0*t+pi/2) * amp;
%     d_xi = w0*cos(w0*t+pi/2) * amp;
%     dd_xi = -w0*w0*sin(w0*t+pi/2) * amp;
    xi = 0*t;
    d_xi = 0*t;
    dd_xi = 0*t;
    
%     eta = sin(w0*t+pi/2) * amp;
%     d_eta = w0*cos(w0*t+pi/2) * amp;
%     dd_eta = -w0*w0*sin(w0*t+pi/2) * amp;
    eta = 0*t;
    d_eta = 0*t;
    dd_eta = 0*t;

    % System start
    A_w_a = [0; 0; w_a];
    
    A1_M_m = I_a * [dd_eta; 0; 0] + cross([d_eta; 0; 0], I_a * A_w_a);
    A1_H_m = Rx(eta)' * (I_a * A_w_a);
    A1_I_a = Rx(eta)' * I_a;
    
    %B_M_m = Ry(xi(i))' * A1_M_m + A1_I_a * [0; dd_xi(i); 0] + cross([0; d_xi(i); 0], A1_H_m);
    B_M_m = Rx(eta)' * I_a * [0; dd_xi; 0] + Ry(xi)' * I_a * [dd_eta; 0; 0];
    B_H_m = Ry(xi)' * A1_H_m;
    B_I_a = Ry(xi)' * A1_I_a;

    B_R_A = Ry(xi)' * Rx(eta)';

    % torque from thrust
    thrust = B_R_A * [0; 0; T_f];
    B_M_f = cross(r_pg, thrust);

    % Drag torque from motor
    B_M_d = B_R_A * [0; 0; T_d];

    % Inertia
    I_b = B_I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];

    % Newton-Euler equation
    B_M = -cross(W, I_b * W) + B_M_f - B_M_d - B_M_m;

    dW = I_b \ B_M;
    dQ = reshape(Q * skew(W), [9 1]);

    ddP = [0; 0; -g] + Q * thrust/(m_a + m_fm);
    dydt = [dW; dQ; ddP; dP];
    inputs = [dd_xi dd_eta];
    outputs = [thrust; -B_M_m];
end

function r = Rx(t)
    r = [1 0      0;
         0 cos(t) -sin(t);
         0 sin(t) cos(t)];
end

function r = Ry(t)
    r = [cos(t)  0 sin(t);
         0       1 0; 
         -sin(t) 0 cos(t)];
end

function X = skew(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end

function draw_agent(P, R)
    R = squeeze(R);
    radius = 0.5;
    height = 0.2;
    k=1:12;
    kk = k*pi/3 + pi/6;
    vertices = zeros(12, 3);
    vertices(:, 1) = radius*cos(kk);
    vertices(:, 2) = radius*sin(kk);
    vertices(1:6, 3) = height;
    vertices(7:12, 3) = -height;
    vertices = P + vertices * R';

    k=1:6;
    face = ones(6,1) * [0 6 7 1] + k';
    face(6, 3) = 7;
    face(6, 4) = 1;
    patch('Vertices',vertices,'Faces',face, 'EdgeColor','black','FaceColor','none','LineWidth',1)
end
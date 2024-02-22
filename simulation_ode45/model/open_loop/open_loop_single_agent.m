close all;

%% Simulation parameters
T = 5;
sampling_rate = 100;
t = linspace(0, T, sampling_rate*T);
dt = 1 / sampling_rate;

P = zeros(sampling_rate*T, 9);
Q = zeros(3, 3, 2, sampling_rate*T);
Q(:, :, 1, 1) = eye(3);
Q(:, :, 2, 1) = eye(3);
W = zeros(sampling_rate*T, 6);
%z = zeros(sampling_rate*T, 2);

omega = [0; 0; 0];

%% Parameters
% Environment
g = 9.818;
rho = 1.225;   % kg/m3

% Drone
m_a = 0.2;     % Mass, Kg
m_fm = 0.3;
g = 9.8;       % gravity
r_pg = [0; 0; 0.03];  % Leverage length from c.p. to c.g.
r_fm = [0; 0; -0.02]; % Leverage length from c.fm. to c.g. 

T_f = (m_a + m_fm) * g + 0;      % Translational thrust
T_d = 0;                         % Drag torque
I_fm = [10 0 0; 0 10 0; 0 0 10]; % Body Inertial
I_a  = [10 0 0; 0 10 0; 0 0 10]; % Actuator Inertial

w0 = pi;
amp = 20 * pi / 180;
w_a = 0;

sB_I_a = zeros(sampling_rate*T, 3);
sB_M_a = zeros(sampling_rate*T, 3);

%% Command generation
xi = -sin(w0*t+pi/2) * amp;
d_xi = -w0*cos(w0*t+pi/2) * amp;
dd_xi = w0*w0*sin(w0*t+pi/2) * amp;
% xi = 0*t;
% d_xi = 0*t;
% dd_xi = 0*t;

eta = sin(w0*t+pi/2) * amp;
d_eta = w0*cos(w0*t+pi/2) * amp;
dd_eta = -w0*w0*sin(w0*t+pi/2) * amp;
% eta = 0*t;
% d_eta = 0*t;
% dd_eta = 0*t;

%% Simulation
for i=1:sampling_rate*T-1
    ti = i / sampling_rate;

    A_w_a = [0; 0; w_a];
    
    A1_M_m = I_a * [dd_eta(i); 0; 0] + cross([d_eta(i); 0; 0], I_a * A_w_a);
    A1_H_m = Rx(eta(i))' * (I_a * A_w_a);
    A1_I_a = Rx(eta(i))' * I_a;
    
    %B_M_m = Ry(xi(i))' * A1_M_m + A1_I_a * [0; dd_xi(i); 0] + cross([0; d_xi(i); 0], A1_H_m);
    B_M_m = Rx(eta(i))' * I_a * [0; dd_xi(i); 0] + Ry(xi(i))' * I_a * [dd_eta(i); 0; 0];
    B_H_m = Ry(xi(i))' * A1_H_m;
    B_I_a = Ry(xi(i))' * A1_I_a;

    B_R_A = Ry(xi(i))' * Rx(eta(i))';

    % torque from thrust
    B_M_f = cross(r_pg, B_R_A * [0; 0; T_f]);

    % Drag torque from motor
    B_M_d = B_R_A * [0; 0; T_d];

    % Inertia
    I_b = B_I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];
    %I_b = B_I_a + I_fm;

    % Newton-Euler equation
    B_M = -cross(W(i, 4:6), I_b * W(i, 4:6)')' + B_M_f - B_M_d - B_M_m;

    W(i+1, 1:3) = I_b \ B_M;
    W(i+1, 4:6) = W(i, 4:6) + W(i+1, 1:3) * dt;

    Q(:, :, 1, i+1) = Q(:, :, 2, i) * skew(W(i+1, 4:6));
    Q(:, :, 2, i+1) = Q(:, :, 2, i) + Q(:, :, 1, i+1) * dt;

    P(i+1, 1:3) = [0; 0; -g] + Q(:, :, 2, i) * B_R_A * [0; 0; T_f]/(m_a + m_fm);
    P(i+1, 4:6) = P(i, 4:6) + P(i+1, 1:3) * dt;
    P(i+1, 7:9) = P(i, 7:9) + P(i+1, 4:6) * dt;

    sB_I_a(i, :) = Q(:, :, 2, i) * B_R_A * [0; 0; T_f]/(m_a + m_fm); %diag(B_I_a);
    sB_M_a(i, :) = -B_M_m;
end

r = 1:10:sampling_rate*T;
rr = 1:5:length(r);

%% Plot
figure(1) 
scatter3(sB_I_a(:, 1), sB_I_a(:, 2), sB_I_a(:, 3), 40, 1:sampling_rate*T);
xlabel('x')
ylabel('y')
zlabel('z')
title('Forces')
colorbar
set(gca,'DataAspectRatio',[1 1 1])

figure(2)
scatter3(P(:, 7), P(:, 8), P(:, 9), 40, 1:sampling_rate*T); hold on
quiver3(P(r, 7), P(r, 8), P(r, 9), sB_I_a(r, 1), sB_I_a(r, 2), sB_I_a(r, 3), 'magenta')

% Draw coordinates of the agent
coord = squeeze (permute(Q(:, :, 2, r), [4 3 1 2]));
q1 = quiver3(P(r, 7), P(r, 8), P(r, 9), coord(:, 1, 1), coord(:, 2, 1), coord(:, 3, 1), 0.1, 'red'); hold on
q2 = quiver3(P(r, 7), P(r, 8), P(r, 9), coord(:, 1, 2), coord(:, 2, 2), coord(:, 3, 2), 0.1, 'green'); hold on
q3 = quiver3(P(r, 7), P(r, 8), P(r, 9), coord(:, 1, 3), coord(:, 2, 3), coord(:, 3, 3), 0.1, 'blue'); hold on
q1.ShowArrowHead = 'off';
q2.ShowArrowHead = 'off';
q3.ShowArrowHead = 'off';

% Draw agent body
for i=1:length(r)
    draw_agent(P(r(i), 7:9), coord(i, :, :)); hold on
end
xlabel('x')
ylabel('y')
zlabel('z')
title('Positions')
colorbar
set(gca,'DataAspectRatio',[1 1 1])

% Euler angles
eulZYX = zeros(length(Q), 3);
for i=1:length(Q)
    eulZYX(i, :) = rotm2eul(Q(:, :, 2, i),'ZYX');
end

%% Draw torque y-axis
figure(3)
% Draw acceleration and torque
subplot(3, 1, 1)
title('rotating acceleration v.s. gyroscopic moment on y-axis')
yyaxis left
plot(t, dd_xi,'DisplayName','$\ddot{\xi}$'); hold on
ylabel('rad/s^2')

yyaxis right
plot(t, sB_M_a(:, 2),'DisplayName','$M_y$')
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
q = quiver(P(r, 7), P(r, 9), coord(:, 1, 3), coord(:, 3, 3), 0.05, 'blue'); hold on 
q.ShowArrowHead = 'off';
q = quiver(P(r, 7), P(r, 9), coord(:, 1, 1), coord(:, 3, 1), 0.05, 'red'); hold on 
q.ShowArrowHead = 'off';
quiver(P(r, 7), P(r, 9), zeros(length(r), 1), -g*ones(length(r), 1), 0.1, 'cyan'); hold on 
CoP = (P(r, 7:9)' + squeeze(pagemtimes(squeeze(Q(:, :, 2, r)), r_pg)))';
quiver(CoP(:, 1), CoP(:, 3), sB_I_a(r, 1), sB_I_a(r, 3), 0.1, 'magenta'); hold on 
text(CoP(rr, 1), CoP(rr, 3)-1, '\uparrow' + string(r(rr)/ sampling_rate) + 's')
xlabel('x(m)')
ylabel('z(m)')
PosVec = Ax.Position;
%Ax.Position = PosVec+[-0.25 -0.01 0.5 0.08];
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
plot(t, sB_M_a(:, 1),'DisplayName','$M_x$')
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
q = quiver(P(r, 8), P(r, 9), coord(:, 2, 3), coord(:, 3, 3), 0.05, 'blue'); hold on 
q.ShowArrowHead = 'off';
q = quiver(P(r, 8), P(r, 9), coord(:, 2, 2), coord(:, 3, 2), 0.05, 'green'); hold on 
q.ShowArrowHead = 'off';
quiver(P(r, 8), P(r, 9), zeros(length(r), 1), -g*ones(length(r), 1), 0.1, 'cyan'); hold on 

quiver(CoP(:, 2), CoP(:, 3), sB_I_a(r, 2), sB_I_a(r, 3), 0.1, 'magenta'); hold on 
text(CoP(rr, 2), CoP(rr, 3)-1, '\uparrow' + string(r(rr)/ sampling_rate) + 's')
xlabel('y(m)')
ylabel('z(m)')
PosVec = Ax.Position;
Ax.Position = PosVec+[-0.25 -0.01 0.5 0.08];
set(gca,'DataAspectRatio',[1 1 1])
legend('z-axis', 'y-axis', 'gravity', 'thrust')

%% Functions
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
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

    w_prop_u = 134.6596; % Upper rotor speed
    w_prop_l = 134.6596; % Lower rotor speed
    
    T_f = (m_a + m_fm) * g + 0;      % Translational thrust
    T_f = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;

    T_d = 0;                         % Drag torque
    T_d = rho * w_prop_u^2 * prop_d^5 * CP_u - rho * w_prop_l^2 * prop_d^5 * CP_l;

    I_fm = [1 0 0; 0 1 0; 0 0 10]; % Body Inertial
    I_a  = [1 0 0; 0 1 0; 0 0 10]; % Actuator Inertial
    
    w0 = pi;
    amp = 10 * pi / 180;
    w_a = 0;

    %% State variables
    W = y(1:3);
    Q = reshape(y(4:12), [3 3]); % 3x3
    dP = y(13:15);
    P = y(16:18);

    %% Generate input signal
    xi = sin(w0*t+pi/2) * amp;
    d_xi = w0*cos(w0*t+pi/2) * amp;
    dd_xi = -w0*w0*sin(w0*t+pi/2) * amp;
    % xi = 0*t;
    % d_xi = 0*t;
    % dd_xi = 0*t;
    
    eta = sin(w0*t+pi/2) * amp;
    d_eta = w0*cos(w0*t+pi/2) * amp;
    dd_eta = -w0*w0*sin(w0*t+pi/2) * amp;
    % eta = 0*t;
    % d_eta = 0*t;
    % dd_eta = 0*t;

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

function [dydt, inputs, outputs] = drone_fly_diff_rotor(t, y)
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

    w_prop_u = 190.4374+50; % Upper rotor speed
    w_prop_l = 190.4374-50; % Lower rotor speed
    
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
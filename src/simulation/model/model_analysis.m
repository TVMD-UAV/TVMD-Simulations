syms xi d_xi dd_xi eta d_eta dd_eta
syms T_f T_d
syms w1 w2 w3
syms l_pg l_rm m g
syms phi theta psi


assume(phi, 'real')
assume(theta, 'real')
assume(psi, 'real')
I_R_B = Ry(theta) * Rx(phi) * Rz(psi);
latex(simplify(I_R_B'))

W = [w1; w2; w3];
assume(xi, 'real')
assume(eta, 'real')

%% Parameters
[key, params] = get_params();
g = params('g');     % gravity
rho = params('rho');   % kg/m3
prop_d = params('prop_d'); % 8 inch = 20.3 cm

CT_u = params('CT_u'); % upper propeller thrust coefficient
CT_l = params('CT_l'); % lower propeller thrust coefficient
CP_u = params('CP_u');   % upper propeller drag coefficient
CP_l = params('CP_l');   % lower propeller drag coefficient

% Drone
m_a = params('m_a');     % Mass, Kg
m_fm = params('m_fm');
r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
r_fm = params('r_fm'); % Leverage length from c.fm. to c.g. 

I_fm = params('I_fm'); % Body Inertial
I_a  = params('I_a'); % Actuator Inertial

%% Modeling parameter assumptions
g = 9.818;     % gravity
m_a = 0.2;     % Mass, Kg
m_fm = 0.3;
%r_pg = [0; 0; 0.03];  % Leverage length from c.p. to c.g.
%r_fm = [0; 0; -0.02]; % Leverage length from c.fm. to c.g. 
%r_pg = [0; 0; l_pg];  % Leverage length from c.p. to c.g.
%r_fm = [0; 0; l_rm]; % Leverage length from c.fm. to c.g. 

% thrust coefficient
thrust = 0.47;        % Kg
drag_torque = 0.1;    % Kg m
prop_d = 0.0254*9;    % m
rho = 1.225;          % Kg / m^3
rotation_speed = 5000 / 60; % revolution / sec
CT = thrust / (rotation_speed^2 * prop_d^4 * rho)

% drag torque coefficient
CP = drag_torque / (rotation_speed^2 * prop_d^5 * rho)

%% Model analysis
B_R_A = Ry(xi) * Rx(eta);
B_M_m = Rx(eta) * I_a * [0; dd_xi; 0] + Ry(xi) * I_a * [dd_eta; 0; 0];
B_I_a = B_R_A * I_a;

% torque from thrust
thrust = B_R_A * [0; 0; T_f];
B_M_f = cross(r_pg, thrust);

% Drag torque from motor
B_M_d = B_R_A * [0; 0; T_d];

% Inertia
I_b = B_I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];

T_all = B_M_f - B_M_d - B_M_m;
B_M = -cross(W, I_b * W) + T_all;
ddP = [0; 0; -g] + eye(3) * thrust/(m_a + m_fm);

latex(simplify(T_all) )
latex(simplify(ddP))
latex(simplify(cross(W, I_b * W)))
latex(simplify(I_b)*10000)

%% Solve for equilibrum points
r_pg = [0; 0; l_pg];  % Leverage length from c.p. to c.g.
syms g
simplify(skew(r_pg) * I_R_B' * [0;0;1] * m * g + B_R_A * [0;0;1] * T_d)
latex(simplify(skew(r_pg) * I_R_B' * [0;0;1] * m * g))

% translational
trans = subs(m*g*[0;0;1] + I_R_B*B_R_A*T_f*[0;0;1], [phi, theta], [0, 0]);
latex(simplify(trans))

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

function r = Rz(t)
    r = [cos(t)  -sin(t) 0;
         sin(t) cos(t)  0;
         0       0       1];
end

function X = skew(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end
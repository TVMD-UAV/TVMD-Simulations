function [u_t, u, attitude_d] = ControllerTiltedBody(params)
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
    w_prop_u = 198.2746; % Upper rotor speed
    w_prop_l = 198.2746; % Lower rotor speed

    % Translational thrust
    %Tf = rho * w_prop_u^2 * prop_d^4 * CT_u + rho * w_prop_l^2 * prop_d^4 * CT_l;
    % Drag torque
    %Td = rho * w_prop_u^2 * prop_d^5 * CP_u - rho * w_prop_l^2 * prop_d^5 * CP_l;
    u_t = g;
    u = [0;0;0.005];
    attitude_d = [0, 0, 0];
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

    % Control input
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

    % Control input
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
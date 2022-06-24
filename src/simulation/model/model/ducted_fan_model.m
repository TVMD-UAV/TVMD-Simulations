function [dydt, commands, meta] = ducted_fan_model(params, u, y)
    %% Parameters
    g = params('g');     % gravity
    % Drone parameters
    m = params('m');   % Mass, Kg
    l = params('l');   % m
    I_b  = params('I_b');  % Actuator Inertial

    %% State variables
    W = y(1:3);
    Q = y(4:7);
    dP = y(8:10);
    P = y(11:13);

    R = Q2R(Q);

    %% Aerial Dynamics
    [F_d, M_d] = aerial_drag(params, u, y, false);

    %% Newton-Euler equation
    I_thrust = -R * [0; 0; u(1)];
    B_M = -cross(W, I_b * W) + u(2:4) + M_d;

    ddP = [0; 0; g] + I_thrust - R*skew([0;0;1])*u(2:4)/(m*l) + F_d/m;
    dW = I_b \ B_M;
    dQ = 0.5 * [-Q(2:4)'; 
                Q(1)*eye(3) + skew(Q(2:4))] * W;

    dydt = [dW; dQ; ddP; dP];
    commands = [0; 0; 0; 0];
    meta = [u(2:4); F_d/m; M_d; [0;0;0]];
end
function [dydt, commands, meta] = swarm_model(params, u, y)
    %% Parameters
    g = params('g');     % gravity
    
    % Drone
    m = params('m');
    I_b = params('I_b'); % Body Inertial
    
    pos = params('pos');

    %% State variables
    W = y(1:3);
    Q = reshape(y(4:12), [3 3]); % 3x3
    dP = y(13:15);
    P = y(16:18);
    vartheta = y(19:22);

    [eta, xi, F] = controlAllocation(params, u);

    R = Rx(eta) * Ry(xi);
    % System 
    % R and F required
    vec = zeros(6,1);
    for i=1:length(F)
        vec = vec + ...
            [R * [0;0;1] * F(i);
            cross(pos(:, i), R * [0;0;1] * F(i))];
    end

    %% Newton-Euler equation
    I_thrust = Q * vec(1:3);
    B_M = -cross(W, I_b * W) + vec(4:6);
    B_M_f = vec(4:6);
    B_M_d = [0;0;0];

    ddP = [0; 0; -g] + I_thrust;
    dW = I_b \ B_M;
    dQ = reshape(Q * skew(W), [9 1]);

    dydt = [dW; dQ; ddP; dP; 0;0;0;0];
    commands = F;
    %meta = [B_M_f; B_M_d;zeros(6, 1)];%zeros(12, 1); %[B_M_f; B_M_d; -B_M_a; -B_M_g];
    meta = [B_M_f; B_M_d; B_M; zeros(3, 1)];%zeros(12, 1); %[B_M_f; B_M_d; -B_M_a; -B_M_g];
end

function [eta, xi, F] = controlAllocation(params, u)
    pos = params('pos');

    % Control allocator
    eta = atan2(-u(2), u(3));
    xi = atan2(cos(eta)*u(1), u(3));
    R = Rx(eta) * Ry(xi);
    bar_P = R' * pos;
    bar_u = [R zeros(3);
            zeros(3) R]' * u;

    B = skew([0;0;-1]) * bar_P;
    B = [ones(1, length(pos)); 
        B(1:2, :)];
    opts1=  optimset('display','off');
    F = lsqlin(B, bar_u(3:5), -eye(10), zeros(10, 1),[],[],[],[],[], opts1);
end
function [F_d, M_d] = aerial_drag(params, u, y, using_so3)
    % Drone parameters
    rho = params('rho');
    v_w = params('v_w');
    C_d = params('C_d');
    I_xy = params('I_xy');
    A_cs = params('A_cs');
    esp_M = params('esp_M');

    %% State variables
    if using_so3
        Q = y(4:7);
        dP = y(8:10);
        R = Q2R(Q);
    else
        R = reshape(y(4:12), [3 3]); % 3x3
        dP = y(13:15);
    end

    %% Aerial Dynamics
    F_drag = norm(v_w - dP)*R*C_d*R'*(v_w - dP);
    F_ram = sqrt(u(1)*rho*A_cs/2)*R*I_xy*R'*(v_w - dP);
    F_d = F_drag + F_ram;
    M_d = esp_M*skew([0;0;1])*R'*F_d;
end
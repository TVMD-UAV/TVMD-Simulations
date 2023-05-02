function [dxdt, dzdt] = swarm_model(x, z, z_d, env_params, drone_params)
    % region [Parameters]
    g = env_params.g; % gravity
    rho = env_params.rho; % kg/m3
    prop_d = env_params.prop_d; % 8 inch = 20.3 cm
    
    CT_u = env_params.CT_u; % upper propeller thrust coefficient
    CT_l = env_params.CT_l; % lower propeller thrust coefficient
    CP_u = env_params.CP_u; % upper propeller drag coefficient
    CP_l = env_params.CP_l; % lower propeller drag coefficient
    
    mKp = drone_params.mKp; % Servo motor gain
    mKd = drone_params.mKd; % Servo motor gain
    pKp = drone_params.pKp; % BLDC motor gain
    
    % Bounds
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    r_sigma_x = drone_params.r_sigma_a;
    r_sigma_y = drone_params.r_sigma_b;
    
    % TODO:
    sigma_w0 = drone_params.sigma_w0;
    sigma_w = drone_params.prop_max;
    r_sigma_w = drone_params.r_sigma_w;
    r_f = drone_params.r_f;
    
    % Drone
    m = drone_params.m;     % Body Mass
    I_b = drone_params.I_b; % Body Inertial
    % end region [Parameters]
    
    pos = drone_params.pos;
    psi = drone_params.psi;
    n = length(psi);
    
    % State extraction
    W = x(1:3);
    I_R_B = reshape(x(4:12), [3 3]); % 3x3
    dP = x(13:15);
    P = x(16:18);
    
    dzdt = zeros([6 * n 1]);
    Tf = zeros([n 1]);
    eta_x = zeros([n 1]);
    eta_y = zeros([n 1]);

    
    %% Actuator Model
    [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp);
    % Propeller model
    beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    P_prop = rho * prop_d^4 * beta_allo;
    
    for i = 1:n
        % Actuator Model
        zi = z(6 * (i - 1) + 1 : 6 * i);
        z_di = z_d(4*(i-1)+1 : 4*i);

        z_low = [-sigma_x; -r_sigma_x; -sigma_y; -r_sigma_y; sigma_w0; sigma_w0];
        z_upp = [ sigma_x;  r_sigma_x;  sigma_y;  r_sigma_y;  sigma_w(1);  sigma_w(2)];
        
        % Saturation
        %z_low = [-sigma_x; -r_sigma_x; -sigma_y; -r_sigma_y; sigma_w0; -r_sigma_w];
        %z_upp = [ sigma_x;  r_sigma_x;  sigma_y;  r_sigma_y;  sigma_w;  r_sigma_w];
        zi = min(z_upp, max(z_low, zi));
        dzdti = A_z * zi + B_z * z_di;

        %r_prop_max = sqrt(inv(beta_allo) / (rho * prop_d^4)) * [r_f / (2 * sqrt(abs(Tf_d(i)))); 0];
        r_prop_max = r_sigma_w;
        dz_low = [-r_sigma_x; -inf; -r_sigma_x; -inf; -r_prop_max; -r_prop_max];
        dz_upp = [ r_sigma_x;  inf;  r_sigma_x;  inf;  r_prop_max;  r_prop_max];
        dzdti = min(dz_upp, max(dz_low, dzdti));
        reaching_bound_x = ((zi(1) < -sigma_x) && (dzdti(1) < 0)) || ((zi(1) > sigma_x) && (dzdti(1) > 0));
        reaching_bound_y = ((zi(3) < -sigma_y) && (dzdti(3) < 0)) || ((zi(3) > sigma_y) && (dzdti(3) > 0));
        reaching_bound_w = ((zi(5:6) <= 0) & (dzdti(5:6) <= 0)) | ((zi(5:6) >= sigma_w) & (dzdti(5:6) >= 0));
        dzdti(1) = (~reaching_bound_x) * dzdti(1);
        dzdti(3) = (~reaching_bound_y) * dzdti(3);
        dzdti(5:6) = (~reaching_bound_w) .* dzdti(5:6);

        %dzdt(6 * (i - 1) + 1 : 6 * i) = A_z * zi + B_z * z_d;
        dzdt(6 * (i - 1) + 1 : 6 * i) = dzdti;
    
        zio = C_z * zi;
        TfTdi = P_prop * zio(3:4, 1).^2;
        Tf(i) = TfTdi(1);
        eta_x(i) = zio(1);
        eta_y(i) = zio(2);
    end
    
    u = full_dof_mixing(pos, psi, eta_x, eta_y, Tf);
    
    % Translational
    I_thrust = I_R_B * u(1:3);
    ddP = ([0; 0; -m * g] + I_thrust) / m;
    
    % Rotational
    B_M = -cross(W, I_b * W) + u(4:6);
    dW = I_b \ B_M;
    dQ = reshape(I_R_B * skew(W), [9 1]);

dxdt = [dW; dQ; ddP; dP];
end

function [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp)
A_z = [   0    1    0    0    0    0;
       -mKp -mKd    0    0    0    0;
          0    0    0    1    0    0;
          0    0 -mKp -mKd    0    0;
          0    0    0    0 -pKp    0;
          0    0    0    0    0 -pKp];
B_z = [   0    0    0    0;
        mKp    0    0    0;
          0    0    0    0;
          0  mKp    0    0;
          0    0  pKp    0;
          0    0    0  pKp];
C_z = [1 0 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
end

function vec = full_dof_mixing(P, psi, a, b, tf)
    n = length(psi);
    M = get_M(n, psi, P);
    fi = get_f(a, b, tf);
    vec = M * fi;
end 
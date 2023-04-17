function [Tf, eta_x, eta_y] = z2raw(n, z, env_params)
    %% Parameters
    g = env_params.g; % gravity
    rho = env_params.rho; % kg/m3
    prop_d = env_params.prop_d; % 8 inch = 20.3 cm
    
    CT_u = env_params.CT_u; % upper propeller thrust coefficient
    CT_l = env_params.CT_l; % lower propeller thrust coefficient
    CP_u = env_params.CP_u; % upper propeller drag coefficient
    CP_l = env_params.CP_l; % lower propeller drag coefficient

    Tf = zeros([n 1]);
    eta_x = zeros([n 1]);
    eta_y = zeros([n 1]);

    C_z = [1 0 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];

    beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    P_prop = rho * prop_d^4 * beta_allo;

    for i = 1:n
        zi = z(6 * (i - 1) + 1 : 6 * i);
        zio = C_z * zi;
        TfTdi = P_prop * zio(3:4, 1).^2;
        Tf(i) = TfTdi(1);
        eta_x(i) = zio(1);
        eta_y(i) = zio(2);
    end
end
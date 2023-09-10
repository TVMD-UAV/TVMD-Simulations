function [Tf, eta_x, eta_y] = calc_shrink_control(drone_params, ctrl_params, dt, Tf, eta_x, eta_y, f1)
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    r_sigma_x = drone_params.r_sigma_a;
    r_sigma_y = drone_params.r_sigma_b;
    f_max = drone_params.f_max;
    r_f = drone_params.r_f;

    % todo: only shrink the agent when the direction is outward admissible set
    n = length(drone_params.psi);
    [Tf1, x1, y1] = inverse_input(n, f1);

    f0 = get_f(eta_x, eta_y, Tf);
    [tf_t, x_t, y_t] = inverse_input_tangent(n, f0, f1);

    ctrl_params.pseudo_gamma = 0.1;

    % disp(size((dt*r_sigma_x.*(eta_x<=-sigma_x).*(x1<=-sigma_x))))
    dr_sigma_x = dt * r_sigma_x;
    dr_sigma_y = dt * r_sigma_y;
    dr_f = dt * r_f;
    % eta_x = sat(eta_x, ...
    %             -sigma_x + (dt*r_sigma_x.*(eta_x-dr_sigma_x<=-sigma_x).*(x1<=-sigma_x))*ctrl_params.pseudo_gamma, ...
    %             sigma_x  - (dt*r_sigma_x.*(eta_x+dr_sigma_x>= sigma_x).*(x1>= sigma_x))*ctrl_params.pseudo_gamma);
    % eta_y = sat(eta_y, ...
    %             -sigma_y + (dt*r_sigma_y.*(eta_y-dr_sigma_y<=-sigma_y).*(y1<=-sigma_y))*ctrl_params.pseudo_gamma, ...
    %             sigma_y  - (dt*r_sigma_y.*(eta_y+dr_sigma_y>= sigma_y).*(y1>= sigma_y))*ctrl_params.pseudo_gamma);
    eta_x = sat(eta_x, ...
                -sigma_x + (dt*r_sigma_x.*(eta_x-dr_sigma_x<=-sigma_x).*(x_t <= 0))*ctrl_params.pseudo_gamma, ...
                sigma_x  - (dt*r_sigma_x.*(eta_x+dr_sigma_x>= sigma_x).*(x_t >= 0))*ctrl_params.pseudo_gamma);
    eta_y = sat(eta_y, ...
                -sigma_y + (dt*r_sigma_y.*(eta_y-dr_sigma_y<=-sigma_y).*(y_t <= 0))*ctrl_params.pseudo_gamma, ...
                sigma_y  - (dt*r_sigma_y.*(eta_y+dr_sigma_y>= sigma_y).*(y_t >= 0))*ctrl_params.pseudo_gamma);

    % eta_x = sat(eta_x, ...
    %             -sigma_x + (dt*r_sigma_x.*(eta_x<=-sigma_x))*ctrl_params.pseudo_gamma, ...
    %             sigma_x  - (dt*r_sigma_x.*(eta_x>= sigma_x))*ctrl_params.pseudo_gamma);
    % eta_y = sat(eta_y, ...
    %             -sigma_y + (dt*r_sigma_y.*(eta_y<=-sigma_y))*ctrl_params.pseudo_gamma, ...
    %             sigma_y  - (dt*r_sigma_y.*(eta_y>= sigma_y))*ctrl_params.pseudo_gamma);
    % Tf = sat(Tf, 0 + r_f*dt, f_max - r_f*dt*(Tf1>=f_max));
    Tf = sat(Tf, 0 + dr_f, f_max - dr_f*(Tf1>0));

    % Tf = sat(Tf, 0 + r_f*dt*(Tf1<=0), f_max - r_f*dt*(Tf1>=f_max));
    % eta_x = sat(eta_x, -sigma_x + r_sigma_x*dt, sigma_x - r_sigma_x*dt);
    % eta_y = sat(eta_y, -sigma_y + r_sigma_y*dt, sigma_y - r_sigma_y*dt);
    % Tf = sat(Tf, 0 + r_f*dt, f_max - r_f*dt);
end

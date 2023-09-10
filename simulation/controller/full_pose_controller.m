function [u_d, R_d, attitude_d, eX, eRv, eOmega, theta, nb3, u_f, b_u_f] = full_pose_controller(t, x_r, R_r, x, env_params, drone_params, ctrl_params)
    m = drone_params.mb; % Mass, Kg
    g = env_params.g;
    I_b = drone_params.I_bb; % Leverage length from c.p. to c.g.
    
    % States
    p = x(16:18);
    v = x(13:15);
    R = reshape(x(4:12), [3 3]); % 3x3
    w = x(1:3);
    
    % Gain
    Kp = ctrl_params.Kp;
    Kd = ctrl_params.Kd;
    Kr = ctrl_params.Kr;
    Ko = ctrl_params.Ko;
    
    % % Translational error
    ep = p - x_r(1:3, 1);
    ev = v - x_r(1:3, 2);
    eX = [ep ev];
    
    f_r = m * x_r(1:3, 3) + m*[0; 0; g] + m * (- Kp * ep - Kd * ev);
    %if f_r(1) < -m * g+1; f_r(1) = -m * g + 1; end
    %if f_r(2) < -m * g+1; f_r(2) = -m * g + 1; end
    %if f_r(3) < -m * g+1; f_r(3) = -m * g + 1; end
    %if f_r(3) < 0.1; f_r(3) = 0.1; end
    u_f = R' * f_r;
    b_u_f = u_f;
    
    if ctrl_params.attitude_planner_non_neg_constraint
        if u_f(3) < 0.1; u_f(3) = 0.1; end
    end
    f_r_sat = force_magnitude_sat(drone_params, t, R * u_f);

    [R_d, theta, nb3] = calc_R_d(drone_params, f_r_sat, R_r, t);

    if ctrl_params.force_projection
        u_f = force_projection_mag_first(drone_params, t, u_f);
    else 
        if u_f(3) < 0.1; u_f(3) = 0.1; end
    end
    
    %R_d = R_r;
    % w_d = vee(R_d' * d_R_d);
    
    % Attitude error
    eRx = 0.5 * (R_d' * R - R' * R_d);
    eR = vee(eRx);
    eRv = 0.5 * trace(eye(3) - R_r' * R);
    % eOmega = w - R' * R_d * w_d;
    eOmega = w;
    attitude_d = rot2zxy(R_d);
    
    u_tau = cross(w, I_b * w) + I_b * ( - Kr * eR - Ko * eOmega);
    u_d = [u_f; u_tau];
end

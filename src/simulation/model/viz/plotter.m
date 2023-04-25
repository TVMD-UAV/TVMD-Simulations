function plotter(t, r, dydt, y, inputs, outputs, refs, projectpath, foldername, filename)
    rotation_matrix = true;

    dirname = strcat(projectpath, foldername);

    if not(isfolder(dirname))
        mkdir(dirname)
    end

    %% Marker style
    makerstyle = false;

    if makerstyle == true
        lineStyle = ':';
        markerStyle = 'o';
    else
        lineStyle = '--';
        markerStyle = 'none';
    end

    %% Extract parameters
    Tf = inputs(:, 1);
    u = inputs(:, 2:4);
    w_m1 = inputs(:, 5);
    w_m2 = inputs(:, 6);
    eta_d = inputs(:, 7);
    xi_d = inputs(:, 8);

    % States
    dW = dydt(:, 1:3);
    W = y(:, 1:3); % Angular velocity
    % Translational
    ddP = dydt(:, 13:15);
    dP = y(:, 13:15);
    P = y(:, 16:18);
    eta = y(:, 19);
    xi = y(:, 20);

    thrust = outputs(:, 1:3);
    B_M_f = outputs(:, 4:6);
    B_M_d = outputs(:, 7:9);
    B_M_a = outputs(:, 10:12);
    B_M_g = outputs(:, 13:15);
    traj = reshape(refs(:, 1:12), [length(y), 3, 4]);
    traj = permute(traj, [1, 3, 2]);
    Q_d = refs(:, 13:15);
    beta = refs(:, 16:18);
    %tilde_mu = outputs(:, 26:28);
    tilde_mu = zeros([length(y) 3]);

    % Rotational
    if rotation_matrix == true
        % Rotation matrix
        R = reshape(y(:, 4:12), [length(y) 3 3]); % 3x3
        eulZXY = rot2zxy_crossover(R);
        attitude_d = Q_d(:, 1:3);
        R = R(r, :, :);
    else
        % Quaternion
        R = zeros([length(r) 3 3]);
        Qs = y(:, 4:7); % Orientation
        eulZXY = Qs(r, 2:4); % Euler angles

        for i = 1:length(r)
            R(i, :, :) = Q2R(Qs(r(i), :));
        end

        attitude_d = Q_d(:, 1:3);
    end

    CoP = P(:, 1:3);

    key = {'projectpath', 'foldername', 'filename', 'lineStyle', 'markerStyle'};
    value = {projectpath, foldername, filename, lineStyle, markerStyle};
    options = containers.Map(key, value);

    plot_3d(t, r, P, CoP, traj, thrust, R, options);
    plot_state(t, P, dP, traj, W, beta, eulZXY, attitude_d, options);
    plot_error(t, P, dP, traj, W, beta, eulZXY, attitude_d, tilde_mu, options);
    plot_command(t, Tf, u, options);
    plot_state_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, options);
    %plot_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, theta1, theta_a, options);
    plot_torque(t, B_M_f, B_M_d, B_M_g, B_M_a, options);
    plot_motor_command(t, w_m1, w_m2, xi, eta, xi_d, eta_d, options);
    %plot_estimation(t, theta1, theta2, theta3, theta_a, theta_b, options);
    plot_animation(t, r, P, traj, options, R);
end

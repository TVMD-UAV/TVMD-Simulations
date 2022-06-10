function plotter_quaternion(t, r, dydt, y, inputs, outputs, projectpath, foldername, filename)    
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

    thrust = outputs(:, 1:3);
    B_M = outputs(:, 4:6);
    traj = reshape(outputs(:, 7:18), [length(outputs), 3, 4]);
    traj = permute(traj, [1, 3, 2]);
    Q_d = outputs(:, 19:22);
    beta = outputs(:, 23:25);
    tilde_mu = outputs(:, 26:28);
    theta_a = outputs(:, 29:31);
    theta_b = outputs(:, 32:34);

    theta1 = y(:, 14:16);
    theta2 = y(:, 17:19);
    theta3 = y(:, 20:22);

    % Rotational
    dW = dydt(:, 1:3);
    W = y(:, 1:3);        % Angular velocity
    Qs = y(:, 4:7);       % Orientation
    eulZXY = Qs(:, 2:4);  % Euler angles
    attitude_d = Q_d(:, 2:4);
    R = zeros([length(Qs) 3 3]);
    for i=1:length(Qs)
        R(i, :, :) = Q2R(Qs(i, :));
    end
    
    % Translational
    ddP = dydt(:, 8:10);
    dP = y(:, 8:10);
    P = y(:, 11:13);
    CoP = P(:, 1:3);

    key = {'projectpath', 'foldername', 'filename', 'lineStyle', 'markerStyle'};
    value = {projectpath, foldername, filename, lineStyle, markerStyle};
    options = containers.Map(key, value);

    plot_3d(t, r, P, CoP, traj, thrust, R, options);
    plot_state(t, P, dP, traj, W, beta, eulZXY, attitude_d, options);
    plot_error(t, P, dP, traj, W, beta, eulZXY, attitude_d, tilde_mu, options);
    plot_command(t, Tf, u, options);
    plot_norm(t, dP, P, traj, eulZXY, attitude_d, W, beta, theta1, theta_a, options);
    plot_estimation(t, theta1, theta2, theta3, theta_a, theta_b, options);
    plot_animation(t, r, P, traj, options, R);
end
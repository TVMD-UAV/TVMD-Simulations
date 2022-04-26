function plotter(t, r, dydt, y, inputs, outputs)
    %% Extract parameters
    [key, params] = get_params();
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
    r_fm = params('r_fm');  % Leverage length from c.fm. to c.g. 
    dd_xi = inputs(:, 1);
    dd_eta = inputs(:, 2);
    thrust = outputs(:, 1:3);
    B_M_f = outputs(:, 4:6);
    B_M_d = outputs(:, 7:9);
    B_M_m = outputs(:, 10:12);

    % Rotational
    % Angular velocity
    dW = dydt(:, 1:3);
    W = y(:, 1:3);
    % Orientation
    Q = reshape(y(:, 4:12), [length(y) 3 3]); % 3x3
    % Euler angles
    QT = permute(Q, [2 3 1]);
    eulZYX = rotm2eul(QT,'ZYX');
    
    % Translational
    ddP = dydt(:, 13:15);
    dP = y(:, 13:15);
    P = y(:, 16:18);

    CoP = P(:, 1:3) + squeeze(pagemtimes(permute(Q, [2 3 1]), r_pg))';
    
    %% Draw 3D view
    figure(1)
    scatter3(P(:, 1), P(:, 2), P(:, 3), 40, t); hold on
    quiver3(CoP(:, 1), CoP(:, 2), CoP(:, 3), thrust(:, 1), thrust(:, 2), thrust(:, 3), 'magenta')
    
    % Draw coordinates of the agent
    q1 = quiver3(P(:, 1), P(:, 2), P(:, 3), Q(:, 1, 1), Q(:, 2, 1), Q(:, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(:, 1), P(:, 2), P(:, 3), Q(:, 1, 2), Q(:, 2, 2), Q(:, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(:, 1), P(:, 2), P(:, 3), Q(:, 1, 3), Q(:, 2, 3), Q(:, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';
    
    % Draw agent body
    for i=1:length(r)
        draw_agent(P(r(i), :), Q(r(i), :, :)); hold on
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    colorbar
    set(gca,'DataAspectRatio',[1 1 1])
    
    
    %% Draw torque y-axis
    figure(2)
    
    % Draw acceleration and torque
    subplot(3, 1, 1)
    title('rotating acceleration v.s. gyroscopic moment on y-axis')
    yyaxis left
    plot(t, dd_xi,'DisplayName','$\ddot{\xi}$'); hold on
    ylabel('rad/s^2')
    
    yyaxis right
    plot(t, B_M_f(:, 2),'DisplayName','$M_y$')
    ylabel('Nm')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    % Draw euler angles
    Ax = subplot(3, 1, 2);
    plot(t, eulZYX(:, 2),'DisplayName','Euler')
    ylabel('rad')
    
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    PosVec = Ax.Position;
    Ax.Position = PosVec+[0 0.07 0 0];
    
    % Draw x-z view
    Ax = subplot(3, 1, 3);
    q = quiver(P(:, 1), P(:, 3), Q(:, 1, 3), Q(:, 3, 3), 1, 'blue'); hold on 
    q.ShowArrowHead = 'off';
    q = quiver(P(:, 1), P(:, 3), Q(:, 1, 1), Q(:, 3, 1), 1, 'red'); hold on 
    q.ShowArrowHead = 'off';
    quiver(P(:, 1), P(:, 3), zeros(length(y), 1), -9.8*ones(length(y), 1), 0.1, 'cyan'); hold on 
    quiver(CoP(:, 1), CoP(:, 3), thrust(:, 1), thrust(:, 3), 0.1, 'magenta'); hold on 
    text(CoP(r, 1), CoP(r, 3)-0.1, '\uparrow' + string(t(r)) + 's')
    xlabel('x(m)')
    ylabel('z(m)')
    PosVec = Ax.Position;
    Ax.Position = PosVec+[-0.25 -0.01 0.5 0.08];
    set(gca,'DataAspectRatio',[1 1 1])
    legend('z-axis', 'x-axis', 'gravity', 'thrust')
    
    %% Draw torque x-axis
    figure(3)
    % Draw acceleration and torque
    subplot(3, 1, 1)
    title('rotating acceleration v.s. gyroscopic moment on x-axis')
    yyaxis left
    plot(t, dd_eta,'DisplayName','$\ddot{\eta}$'); hold on
    ylabel('rad/s^2')
    
    yyaxis right
    plot(t, B_M_f(:, 1),'DisplayName','$M_x$')
    ylabel('Nm')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    % Draw euler angles
    Ax = subplot(3, 1, 2);
    plot(t, eulZYX(:, 3),'DisplayName','Euler')
    ylabel('rad')
    
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    PosVec = Ax.Position;
    Ax.Position = PosVec+[0 0.07 0 0];
    
    %figure(5)
    % Draw y-z view
    Ax = subplot(3, 1, 3);
    q = quiver(P(:, 2), P(:, 3), Q(:, 2, 3), Q(:, 3, 3), 1, 'blue'); hold on 
    q.ShowArrowHead = 'off';
    q = quiver(P(:, 2), P(:, 3), Q(:, 2, 2), Q(:, 3, 2), 1, 'green'); hold on 
    q.ShowArrowHead = 'off';
    quiver(P(:, 2), P(:, 3), zeros(length(y), 1), -9.8*ones(length(y), 1), 0.1, 'cyan'); hold on 
    
    quiver(CoP(:, 2), CoP(:, 3), thrust(:, 2), thrust(:, 3), 0.1, 'magenta'); hold on 
    text(CoP(r, 2), CoP(r, 3)-0.1, '\uparrow' + string(t(r)) + 's')
    xlabel('y(m)')
    ylabel('z(m)')
    PosVec = Ax.Position;
    Ax.Position = PosVec+[-0.25 -0.01 0.5 0.08];
    set(gca,'DataAspectRatio',[1 1 1])
    legend('z-axis', 'y-axis', 'gravity', 'thrust')
    
    %% Draw position and orientation
    figure('Position', [10 10 800 800])
    Ax = subplot(4, 1, 1);
    plot(t, eulZYX(:, 1),'DisplayName','Yaw','LineWidth',2); hold on 
    plot(t, eulZYX(:, 2),'DisplayName','Pitch','LineWidth',2); hold on 
    plot(t, eulZYX(:, 3),'DisplayName','Roll','LineWidth',2); hold on 
    ylabel('rad')
    xlabel('time')
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    %% Draw velocity
    %figure('Position', [10 10 800 400])
    Ax = subplot(4, 1, 2);
    plot(t, W(:, 3),'DisplayName','Yaw $$\psi$$','LineWidth',2); hold on 
    plot(t, W(:, 2),'DisplayName','Pitch $$\theta$$','LineWidth',2); hold on 
    plot(t, W(:, 1),'DisplayName','Roll $$\phi$$','LineWidth',2); hold on 
    ylabel('rad')
    xlabel('time')
    title('Angular velocity w.r.t. body frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    Ax = subplot(4, 1, 3);
    plot(t, P(:, 1),'DisplayName','X','LineWidth',2); hold on 
    plot(t, P(:, 2),'DisplayName','Y','LineWidth',2); hold on 
    plot(t, P(:, 3),'DisplayName','Z','LineWidth',2); hold on 
    ylabel('m')
    xlabel('time')
    title('Position')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    Ax = subplot(4, 1, 4);
    plot(t, dP(:, 1),'DisplayName','X','LineWidth',2); hold on 
    plot(t, dP(:, 2),'DisplayName','Y','LineWidth',2); hold on 
    plot(t, dP(:, 3),'DisplayName','Z','LineWidth',2); hold on 
    ylabel('m/s')
    xlabel('time')
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    %% Draw torque
    figure(6)
    plot(t, B_M_f(:, 1),  'r^','DisplayName','M_f: x'); hold on 
    plot(t, B_M_f(:, 2),  'r-','DisplayName','M_f: y'); hold on 
    plot(t, B_M_f(:, 3),  'r+','DisplayName','M_f: z'); hold on  
    plot(t, B_M_d(:, 1), 'g^','DisplayName','M_d: x'); hold on 
    plot(t, B_M_d(:, 2), 'g-','DisplayName','M_d: y'); hold on 
    plot(t, B_M_d(:, 3), 'g+','DisplayName','M_d: z'); hold on
    plot(t, B_M_m(:, 1), 'b^','DisplayName','M_m: x'); hold on 
    plot(t, B_M_m(:, 2), 'b-','DisplayName','M_m: y'); hold on 
    plot(t, B_M_m(:, 3), 'b+','DisplayName','M_m: z'); hold on
    ylabel('Nm')
    xlabel('time')
    title('Torque profile')
    legend()
end

function draw_agent(P, R)
    R = squeeze(R);
    radius = 0.5;
    height = 0.2;
    k=1:12;
    kk = k*pi/3 + pi/6;
    vertices = zeros(12, 3);
    vertices(:, 1) = radius*cos(kk);
    vertices(:, 2) = radius*sin(kk);
    vertices(1:6, 3) = height;
    vertices(7:12, 3) = -height;
    vertices = P + vertices * R';

    k=1:6;
    face = ones(6,1) * [0 6 7 1] + k';
    face(6, 3) = 7;
    face(6, 4) = 1;
    patch('Vertices',vertices,'Faces',face, 'EdgeColor','black','FaceColor','none','LineWidth',1);
    alpha(0.3);
end
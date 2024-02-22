function plotter(t, r, dydt, y, inputs, outputs)
    makerstyle = false;
    if makerstyle == true
        lineStyle = ':';
        markerStyle = 'o';
    else
        lineStyle = '--';
        markerStyle = 'none';
    end
    %% Extract parameters
    [key, params] = get_params();
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
    r_fm = params('r_fm');  % Leverage length from c.fm. to c.g. 
    w_m1 = inputs(:, 1);
    w_m2 = inputs(:, 2);
    dd_eta = inputs(:, 3);
    dd_xi = inputs(:, 4);
    thrust = outputs(:, 1:3);
    B_M_f = outputs(:, 4:6);
    B_M_d = outputs(:, 7:9);
    B_M_g = outputs(:, 10:12);
    B_M_a = outputs(:, 13:15);

    detailed = false;
    if (length(outputs) > 15)
        traj = reshape(outputs(:, 16:27), [length(outputs), 3, 4]);
        traj = permute(traj, [1, 3, 2]);
        attitude_d = outputs(:, 28:30);
        detailed = true;
    end

    % Rotational
    % Angular velocity
    dW = dydt(:, 1:3);
    W = y(:, 1:3);
    % Orientation
    Q = reshape(y(:, 4:12), [length(y) 3 3]); % 3x3
    % Euler angles
    %QT = permute(Q, [2 3 1])
    %eulZXY = rotm2eul(QT,'ZXY');
    eulZXY = rot2zxy(Q);
    
    % Translational
    ddP = dydt(:, 13:15);
    dP = y(:, 13:15);
    P = y(:, 16:18);

    CoP = P(:, 1:3) + squeeze(pagemtimes(permute(Q, [2 3 1]), r_pg))';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    % Draw desired trajectory
    if detailed
        scatter3(traj(:, 1, 1), traj(:, 2, 1), traj(:, 3, 1), "red", 'Marker','.'); hold on
    end
    
    % Draw agent body
    for i=1:length(r)
        draw_agent_quad(P(r(i), :), Q(r(i), :, :), 1+256*t(r(i))/t(end)); hold on
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    colorbar
    set(gca,'DataAspectRatio',[1 1 1])
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    plot(t, eulZXY(:, 2),'DisplayName','Euler')
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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    plot(t, eulZXY(:, 3),'DisplayName','Euler')
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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw position and orientation
    figure('Position', [10 10 800 800])
    Ax = subplot(4, 1, 1);
    plot(t, eulZXY(:, 1),'DisplayName','Yaw $$\psi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, eulZXY(:, 2),'DisplayName','Roll $$\phi$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, eulZXY(:, 3),'DisplayName','Pitch $$\theta$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    % Draw desired trajectory
    if detailed
        plot(t, attitude_d(:, 1),'DisplayName','Yaw $$\psi_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, attitude_d(:, 2),'DisplayName','Roll $$\phi_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, attitude_d(:, 3),'DisplayName','Pitch $$\theta_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    ylabel('rad')
    xlabel('time')
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    %% Draw velocity
    %figure('Position', [10 10 800 400])
    Ax = subplot(4, 1, 2);
    plot(t, W(:, 3),'DisplayName','Yaw $$\omega_z$$','LineWidth',2); hold on 
    plot(t, W(:, 2),'DisplayName','Roll $$\omega_y$$','LineWidth',2); hold on 
    plot(t, W(:, 1),'DisplayName','Pitch $$\omega_x$$','LineWidth',2); hold on 
    ylabel('rad')
    xlabel('time')
    title('Angular velocity w.r.t. body frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    Ax = subplot(4, 1, 3);
    plot(t, P(:, 1),'DisplayName','$$P_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, P(:, 2),'DisplayName','$$P_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, P(:, 3),'DisplayName','$$P_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 

    % Draw desired trajectory
    if detailed
        plot(t, traj(:, 1, 1),'DisplayName','$$P_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 2, 1),'DisplayName','$$P_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 3, 1),'DisplayName','$$P_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    
    ylabel('m')
    xlabel('time')
    title('Position')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    
    Ax = subplot(4, 1, 4);
    plot(t, dP(:, 1),'DisplayName','$$V_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, dP(:, 2),'DisplayName','$$V_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, dP(:, 3),'DisplayName','$$V_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 

    % Draw desired trajectory
    if detailed
        plot(t, traj(:, 1, 2),'DisplayName','$$V_{x,d}$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 2, 2),'DisplayName','$$V_{y,d}$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, traj(:, 3, 2),'DisplayName','$$V_{z,d}$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    ylabel('m/s')
    xlabel('time')
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    sgtitle('State profile')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw errors
    figure('Position', [10 10 800 800])
    Ax = subplot(3, 1, 1);
    plot(t, attitude_d(:, 1) - eulZXY(:, 1),'DisplayName','Yaw $$\psi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, attitude_d(:, 2) - eulZXY(:, 2),'DisplayName','Roll $$\phi$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, attitude_d(:, 3) - eulZXY(:, 3),'DisplayName','Pitch $$\theta$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('rad')
    xlabel('time')
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    Ax = subplot(3, 1, 2);
    plot(t, traj(:, 1, 1) - P(:, 1),'DisplayName','$$P_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 1) - P(:, 2),'DisplayName','$$P_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 1) - P(:, 3),'DisplayName','$$P_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m')
    xlabel('time')
    title('Position')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    Ax = subplot(3, 1, 3);
    plot(t, traj(:, 1, 2) - dP(:, 1),'DisplayName','$$V_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 2) - dP(:, 2),'DisplayName','$$V_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 2) - dP(:, 3),'DisplayName','$$V_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s')
    xlabel('time')
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    sgtitle('Error profile')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw torque
    figure(6)
    plot(t, B_M_f(:, 1),  'r^','DisplayName','M_f: x'); hold on 
    plot(t, B_M_f(:, 2),  'r-','DisplayName','M_f: y'); hold on 
    plot(t, B_M_f(:, 3),  'r+','DisplayName','M_f: z'); hold on  
    plot(t, B_M_d(:, 1), 'g^','DisplayName','M_d: x'); hold on 
    plot(t, B_M_d(:, 2), 'g-','DisplayName','M_d: y'); hold on 
    plot(t, B_M_d(:, 3), 'g+','DisplayName','M_d: z'); hold on
    plot(t, B_M_g(:, 1), 'b^','DisplayName','M_g: x'); hold on 
    plot(t, B_M_g(:, 2), 'b-','DisplayName','M_g: y'); hold on 
    plot(t, B_M_g(:, 3), 'b+','DisplayName','M_g: z'); hold on
    plot(t, B_M_a(:, 1), 'c^','DisplayName','M_a: x'); hold on 
    plot(t, B_M_a(:, 2), 'c-','DisplayName','M_a: y'); hold on 
    plot(t, B_M_a(:, 3), 'c+','DisplayName','M_a: z'); hold on
    ylabel('Nm')
    xlabel('time')
    title('Torque profile')
    legend()

    %% Draw commands
    figure(7)
    plot(t, w_m1,'DisplayName','$$w_{m1}$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, w_m2,'DisplayName','$$w_{m2}$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    ylabel('rot/s')
    xlabel('time')
    title('Command profile')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    legend()

end

function draw_agent(P, R, ci)
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

    % Colormap
    cmap = parula(256);
    uint8(ci);
    c = cmap(uint8(ci), :);

    k=1:6;
    face = ones(6,1) * [0 6 7 1] + k';
    face(6, 3) = 7;
    face(6, 4) = 1;
    patch('Vertices',vertices,'Faces',face, 'EdgeColor',c,'FaceColor','none','LineWidth',1);
    alpha(0.3);
end

function draw_agent_quad(P, R, ci)
    R = squeeze(R);
    radius = 0.5;
    height = 0.2;
    k=1:8;
    kk = k*pi/2;
    vertices = zeros(8, 3);
    vertices(:, 1) = radius*cos(kk);
    vertices(:, 2) = radius*sin(kk);
    vertices(1:4, 3) = height;
    vertices(5:8, 3) = -height;
    vertices = P + vertices * R';

    % Colormap
    cmap = parula(256);
    uint8(ci);
    c = cmap(uint8(ci), :);

    k=1:4;
    face = ones(4,1) * [0 4 5 1] + k';
    face(4, 3) = 5;
    face(4, 4) = 1;
    patch('Vertices',vertices,'Faces',face, 'EdgeColor',c,'FaceColor','none','LineWidth',1);
    alpha(0.3);
end


function eulers = rot2zxy(R)
    %global psi0 phi0 theta0;

    %phi = asin(R(:, 3, 2)); 
    psi = atan2(-R(:, 1, 2), R(:, 2, 2));    
    phi = atan2(R(:, 3, 2), sqrt(1-R(:, 3, 2).^2));
    theta = atan2(-R(:, 3, 1), R(:, 3, 3));

    psit = psi;
    phit = phi;
    thetat = theta;

    psi_offset = 0;
    phi_offset = 0;
    theta_offset = 0;
    
    for i=2:length(R)
        phi_offset = phi_offset + euler_crossover(phi(i), phi(i-1));
        phit(i) = phi_offset + phi(i);
        
        psi_offset = psi_offset + euler_crossover(psi(i), psi(i-1));
        psit(i) = psi_offset + psi(i);

        theta_offset = theta_offset + euler_crossover(theta(i), theta(i-1));
        thetat(i) = theta_offset + thetat(i);
    end

    % Update base
    eulers = [psit, phit, thetat];
end

function offset = euler_crossover(angle, angle0)
    if angle - angle0 > pi
        offset = -2*pi;
    elseif angle - angle0 < -pi
        offset = 2*pi;
    else
        offset = 0;
    end
end
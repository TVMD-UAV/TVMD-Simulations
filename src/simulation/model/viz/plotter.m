function plotter(t, r, dydt, y, inputs, outputs, refs)
    projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0606_birotor\\model_verification\\';
    foldername = 'test\\';
    filename = 'birotor_veri';
    rotation_matrix=true;

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
    xi_d = inputs(:, 7);
    eta_d = inputs(:, 8);

    % States
    dW = dydt(:, 1:3);
    W = y(:, 1:3);        % Angular velocity
    % Translational
    ddP = dydt(:, 13:15);
    dP = y(:, 13:15);
    P = y(:, 16:18);
    xi = y(:, 19);
    eta = y(:, 20);

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
    detailed = true;

    % Rotational
    if rotation_matrix == true
        % Rotation matrix
        R = reshape(y(:, 4:12), [length(y) 3 3]); % 3x3
        eulZXY = rot2zxy(R);
        attitude_d = Q_d(:, 1:3);
    else
        % Quaternion
        R = zeros([length(y) 3 3]);
        Qs = y(:, 4:7);       % Orientation
        eulZXY = Qs(:, 2:4);  % Euler angles
        for i=1:length(Qs)
            R(i, :, :) = Q2R(Qs(i, :));
        end
        attitude_d = Q_d(:, 1:3);
    end
    CoP = P(:, 1:3);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw 3D view
    figure(1)
    scatter3(P(:, 1), P(:, 2), P(:, 3), 40, t); hold on
    quiver3(CoP(:, 1), CoP(:, 2), CoP(:, 3), thrust(:, 1), thrust(:, 2), thrust(:, 3), 'magenta')
    
    % Draw coordinates of the agent
    q1 = quiver3(P(:, 1), P(:, 2), P(:, 3), R(:, 1, 1), R(:, 2, 1), R(:, 3, 1), 0.1, 'red'); hold on
    q2 = quiver3(P(:, 1), P(:, 2), P(:, 3), R(:, 1, 2), R(:, 2, 2), R(:, 3, 2), 0.1, 'green'); hold on
    q3 = quiver3(P(:, 1), P(:, 2), P(:, 3), R(:, 1, 3), R(:, 2, 3), R(:, 3, 3), 0.1, 'blue'); hold on
    q1.ShowArrowHead = 'off';
    q2.ShowArrowHead = 'off';
    q3.ShowArrowHead = 'off';

    % Draw desired trajectory
    if detailed
        scatter3(traj(:, 1, 1), traj(:, 2, 1), traj(:, 3, 1), "red", 'Marker','.'); hold on
    end
    
    % Draw agent body
    for i=1:length(r)
        draw_agent_quad(P(r(i), :), R(r(i), :, :), 1+256*t(r(i))/t(end)); hold on
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    colorbar
    set(gca,'DataAspectRatio',[1 1 1])
    saveas(gcf, strcat(projectpath, foldername, filename, '_3d.png'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_3d.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_3d.fig'));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw states
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw orientation
    figure('Position', [10 10 800 1000])
    Ax = subplot(5, 1, 1);
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
    %ylim([-0.5 0.5])
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw angular velocity
    %figure('Position', [10 10 800 400])
    Ax = subplot(5, 1, 2);
    plot(t, W(:, 3),'DisplayName','Yaw $$\omega_z$$','LineWidth',2); hold on 
    plot(t, W(:, 2),'DisplayName','Roll $$\omega_y$$','LineWidth',2); hold on 
    plot(t, W(:, 1),'DisplayName','Pitch $$\omega_x$$','LineWidth',2); hold on 
    if detailed
        plot(t, beta(:, 1),'DisplayName','Yaw $${\omega_x}_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, beta(:, 2),'DisplayName','Roll $${\omega_y}_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, beta(:, 3),'DisplayName','Pitch $${\omega_z}_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    ylabel('rad')
    xlabel('time')
    %ylim([-2 2])
    title('Angular velocity w.r.t. body frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw positions
    Ax = subplot(5, 1, 3);
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
    %% Draw velocity
    Ax = subplot(5, 1, 4);
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
    %ylim([-10 10])
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw acceleration
    Ax = subplot(5, 1, 5);
    plot(t, ddP(:, 1),'DisplayName','$$a_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, ddP(:, 2),'DisplayName','$$a_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, ddP(:, 3),'DisplayName','$$a_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s^2')
    xlabel('time')
    %ylim([-2 2])
    title('Acceleration w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')

    sgtitle('State profile')
    saveas(gcf, strcat(projectpath, foldername, filename, '_state.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_state.fig'));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Draw errors
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw orientation
    figure('Position', [10 10 800 1000])
    Ax = subplot(5, 1, 1);
    plot(t, attitude_d(:, 1) - eulZXY(:, 1),'DisplayName','Yaw $$\psi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, attitude_d(:, 2) - eulZXY(:, 2),'DisplayName','Roll $$\phi$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, attitude_d(:, 3) - eulZXY(:, 3),'DisplayName','Pitch $$\theta$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('rad')
    xlabel('time')
    %ylim([-0.1 0.1])
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw angular velocity
    Ax = subplot(5, 1, 2);
    plot(t, W(:, 1) - beta(:, 1),'DisplayName','Yaw $$\omega_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, W(:, 2) - beta(:, 2),'DisplayName','Roll $$\omega_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, W(:, 3) - beta(:, 3),'DisplayName','Pitch $$\omega_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('rad/s')
    xlabel('time')
    %ylim([-0.2 0.2])
    title('Angular velocity')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw position
    Ax = subplot(5, 1, 3);
    plot(t, traj(:, 1, 1) - P(:, 1),'DisplayName','$$P_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 1) - P(:, 2),'DisplayName','$$P_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 1) - P(:, 3),'DisplayName','$$P_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m')
    xlabel('time')
    title('Position')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw velocity
    Ax = subplot(5, 1, 4);
    plot(t, traj(:, 1, 2) - dP(:, 1),'DisplayName','$$V_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, traj(:, 2, 2) - dP(:, 2),'DisplayName','$$V_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, traj(:, 3, 2) - dP(:, 3),'DisplayName','$$V_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s')
    xlabel('time')
    %ylim([-10 10])
    title('Velocity w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    sgtitle('Error profile')
    %% Draw acceleration
    Ax = subplot(5, 1, 5);
    plot(t, tilde_mu(:, 1),'DisplayName','$$\mu_x$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, tilde_mu(:, 2),'DisplayName','$$\mu_y$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, tilde_mu(:, 3),'DisplayName','$$\mu_z$$','LineWidth',2, 'LineStyle','-', 'Color', '#EDB120'); hold on 
    ylabel('m/s^2')
    xlabel('time')
    %ylim([-2 2])
    title('Acceleration w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    sgtitle('Error profile')
    saveas(gcf, strcat(projectpath, foldername, filename, '_error.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_error.fig'));
    
    %% Draw inputs
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw inputs
    figure(6)
    yyaxis left
    plot(t, u(:, 1), 'LineStyle','-', 'DisplayName','u_x', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, u(:, 2), 'LineStyle','-', 'DisplayName','u_y', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, u(:, 3), 'LineStyle','-', 'DisplayName','u_z', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('Nm')
    ylim([-10 4])
    
    yyaxis right
    plot(t, Tf, 'LineStyle','-', 'DisplayName','u_t', 'Color', '#000000','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    ylim([8 16])
    title('Command profile')
    legend()
    saveas(gcf, strcat(projectpath, foldername, filename, '_intpus.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_intpus.fig'));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw torque
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(7)
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
    saveas(gcf, strcat(projectpath, foldername, filename, '_torque.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_torque.fig'));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw commands
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(8)
    Ax = subplot(2, 1, 1);
    plot(t, w_m1,'DisplayName','$$w_{m1}$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, w_m2,'DisplayName','$$w_{m2}$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    ylabel('rot/s')
    xlabel('time')
    title('Propeller speed')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    legend()

    Ax = subplot(2, 1, 2);
    plot(t, xi,'DisplayName','$$\xi$$','LineWidth',2, 'LineStyle','-', 'Color', '#0072BD'); hold on 
    plot(t, eta,'DisplayName','$$\eta$$','LineWidth',2, 'LineStyle','-', 'Color', '#D95319'); hold on 
    plot(t, xi_d,'DisplayName','$$\xi_d$$','LineWidth',2, 'LineStyle','--', 'Color', '#0072BD'); hold on 
    plot(t, eta_d,'DisplayName','$$\eta_d$$','LineWidth',2, 'LineStyle','--', 'Color', '#D95319'); hold on 
    ylabel('rad')
    xlabel('time')
    title('Servo angle')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    legend()

    sgtitle('Command profile')
    saveas(gcf, strcat(projectpath, foldername, filename, '_command.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_command.fig'));

    %% Draw animation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw animation
    figure('Position', [10 10 1200 1200])
    set(gca,'DataAspectRatio',[1 1 1])
    animation_name = strcat(projectpath, foldername, filename, '_3d.gif');

    scatter3(P(:, 1), P(:, 2), P(:, 3), 'Color', 'none'); hold on
    padding = 1;
    xxx = [min(P(:, 1))-padding max(P(:, 1))+padding];
    yyy = [min(P(:, 2))-padding max(P(:, 2))+padding];
    zzz = [min(P(:, 3))-padding max(P(:, 3))+padding];
    [XXX, YYY, ZZZ] = meshgrid(xxx, yyy, zzz);
    XXX = reshape(XXX, [8 1]);
    YYY = reshape(YYY, [8 1]);
    ZZZ = reshape(ZZZ, [8 1]);
    scatter3(XXX, YYY, ZZZ, 'Color', 'none'); hold on
    ss_traj = scatter3(traj(1, 1, 1), traj(1, 2, 1), traj(1, 3, 1), "red", 'Marker','.'); hold on
    ss_state = scatter3(P(1, 1), P(1, 2), P(1, 3), "blue"); hold on
    patch_obj = draw_agent_quad(P(r(1), :), R(r(1), :, :), 1+256*t(r(1))/t(end)); hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('3D view')
    for i=1:length(r)
        ss_traj.XData = traj(1:r(i), 1, 1);
        ss_traj.YData = traj(1:r(i), 2, 1);
        ss_traj.ZData = traj(1:r(i), 3, 1);

        ss_state.XData = P(1:r(i), 1);
        ss_state.YData = P(1:r(i), 2);
        ss_state.ZData = P(1:r(i), 3);

        draw_agent_quad_animation(patch_obj, P(r(i), :), R(r(i), :, :), 1+256*t(r(i))/t(end));
        
        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i == 1
            imwrite(imind,cm,animation_name,'gif', 'Loopcount',inf, 'DelayTime',0.1);
        else
            imwrite(imind,cm,animation_name,'gif', 'WriteMode', 'append', 'DelayTime',0.1);
        end
    end
end

%% Drawing
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

function p = draw_agent_quad(P, R, ci)
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
    p = patch('Vertices',vertices,'Faces',face, 'EdgeColor',c,'FaceColor','none','LineWidth',1);
    alpha(0.3);
end

function draw_agent_quad_animation(patch_obj, P, R, ci)
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
    patch_obj.Vertices = vertices;
    patch_obj.Faces = face;
    patch_obj.EdgeColor = c;
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
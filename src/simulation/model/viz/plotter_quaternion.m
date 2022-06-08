function plotter_quaternion(t, r, dydt, y, inputs, outputs)    
    projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Courses\\110-2\\AdaptiveControl\\FinalProject\\Simulations\\DisturbanceFree\\';
    foldername = 'test\\';
    filename = 'PositionRegulation';

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
    detailed = true;

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
    ylim([-0.5 0.5])
    title('Orientation')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw angular velocity
    %figure('Position', [10 10 800 400])
    Ax = subplot(4, 1, 2);
    plot(t, W(:, 1),'DisplayName','Pitch $$\omega_x$$','LineWidth',2); hold on 
    plot(t, W(:, 2),'DisplayName','Roll $$\omega_y$$','LineWidth',2); hold on 
    plot(t, W(:, 3),'DisplayName','Yaw $$\omega_z$$','LineWidth',2); hold on 
    if detailed
        plot(t, beta(:, 1),'DisplayName','Yaw $${\omega_x}_d$$','LineWidth',2, 'Color', '#0072BD', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, beta(:, 2),'DisplayName','Roll $${\omega_y}_d$$','LineWidth',2, 'Color', '#D95319', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
        plot(t, beta(:, 3),'DisplayName','Pitch $${\omega_z}_d$$','LineWidth',2, 'Color', '#EDB120', 'LineStyle',lineStyle, 'Marker',markerStyle, 'MarkerSize', 3); hold on 
    end
    ylabel('rad')
    xlabel('time')
    ylim([-2 2])
    title('Angular velocity w.r.t. body frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    %% Draw positions
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
    %% Draw velocity
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
    ylim([-10 10])
    title('Velocity w.r.t. inertial frame')
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
    ylim([-0.1 0.1])
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
    ylim([-0.2 0.2])
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
    ylim([-10 10])
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
    ylim([-2 2])
    title('Acceleration w.r.t. inertial frame')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    sgtitle('Error profile')
    saveas(gcf, strcat(projectpath, foldername, filename, '_error.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_error.fig'));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Draw torque
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
    saveas(gcf, strcat(projectpath, foldername, filename, '_command.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_command.fig'));

    %% Draw estimation
    figure(7)
    Ax = subplot(3, 1, 1);
    plot(t, theta1(:, 1), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{1,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta1(:, 2), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{1,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta1(:, 3), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{1,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    
    plot(t, theta_a(:, 1), 'LineStyle','--', 'DisplayName','$$\theta_{a,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta_a(:, 2), 'LineStyle','--', 'DisplayName','$$\theta_{a,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta_a(:, 3), 'LineStyle','--', 'DisplayName','$$\theta_{a,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    title('\theta_1 estimation');
    
    Ax = subplot(3, 1, 2);
    plot(t, theta2(:, 1), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{2,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta2(:, 2), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{2,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta2(:, 3), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{2,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    
    plot(t, theta_a(:, 1), 'LineStyle','--', 'DisplayName','$$\theta_{a,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta_a(:, 2), 'LineStyle','--', 'DisplayName','$$\theta_{a,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta_a(:, 3), 'LineStyle','--', 'DisplayName','$$\theta_{a,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    title('\theta_2 estimation');
    
    Ax = subplot(3, 1, 3);
    plot(t, theta3(:, 1), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{3,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta3(:, 2), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{3,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta3(:, 3), 'LineStyle','-', 'DisplayName','$$\hat{\theta}_{3,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    
    plot(t, theta_b(:, 1), 'LineStyle','--', 'DisplayName','$$\theta_{b,x}$$', 'Color', '#0072BD','LineWidth',2); hold on 
    plot(t, theta_b(:, 2), 'LineStyle','--', 'DisplayName','$$\theta_{b,y}$$', 'Color', '#D95319','LineWidth',2); hold on
    plot(t, theta_b(:, 3), 'LineStyle','--', 'DisplayName','$$\theta_{b,z}$$', 'Color', '#EDB120','LineWidth',2); hold on
    ylabel('N')
    xlabel('time')
    hl = legend('show');
    set(hl, 'Interpreter','latex')
    title('\theta_3 estimation');
    sgtitle('Estimation profile')
    saveas(gcf, strcat(projectpath, foldername, filename, '_estimation.svg'));
    saveas(gcf, strcat(projectpath, foldername, filename, '_estimation.fig'));

    %% Draw animation
    figure('Position', [10 10 1200 1200])
    set(gca,'DataAspectRatio',[1 1 1])
    animation_name = strcat(projectpath, foldername, filename, '_3d.gif');

    scatter3(P(:, 1), P(:, 2), P(:, 3), 'Color', 'none'); hold on
    scatter3([0 max(P(:, 1))+2], [0 max(P(:, 2))+2], [0 max(P(:, 3))+2], 'Color', 'none'); hold on
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
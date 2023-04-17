% Name                  PropagatedName        BlockPath          
% ____________________  ____________________  __________________ 
% 1  [1x1 Signal]      Z_esp                 Z_esp                 SwarmSystem/Record
% 2  [1x1 Signal]      bounds                bounds                SwarmSystem/Record
% 3  [1x1 Signal]      increment             increment             SwarmSystem/Record
% 4  [1x1 Signal]      packed_intersections  packed_intersections  SwarmSystem/Record
% 5  [1x1 Signal]      packed_tw             packed_tw             SwarmSystem/Record
% 6  [1x1 Signal]      raw                   raw                   SwarmSystem/Record
% 7  [1x1 Signal]      sat_order             sat_order             SwarmSystem/Record
% 8  [1x1 Signal]      time                  time                  SwarmSystem/Record
% 9  [1x1 Signal]      z                     z                     SwarmSystem/Record
% 10  [1x1 Signal]      z_d                   z_d                   SwarmSystem/Record

Z_esp = out.recordout{1}.Values.Data;
bounds = out.recordout{2}.Values.Data;
increment = out.recordout{3}.Values.Data;
intersections = out.recordout{4}.Values.Data;
tw = out.recordout{5}.Values.Data;
raw = out.recordout{6}.Values.Data;
sat_order = out.recordout{7}.Values.Data;
t = out.recordout{8}.Values.Data;
z = out.recordout{9}.Values.Data;
z_d = out.recordout{10}.Values.Data;

n = size(Z_esp, 1);
data_len = size(Z_esp, 3);

inspect_time = 2.13;
[m, inspect_index] = min(abs(t - inspect_time));
Z_esp(:, 1, inspect_index)

plot_allocation_result_at_time(env_params, drone_params, n, inspect_index, bounds(:, :, inspect_index), ...
    z(:, 1, inspect_index), z_d(:, 1, inspect_index), raw(:, :, inspect_index), tw(:, :, :, inspect_index), intersections(:, :, :, :, inspect_index), sat_order(:, 1, inspect_index))

function plot_allocation_result_at_time(env_params, drone_params, n, idx, bounds, zs, z_ds, raw, tw, intersections, sat_order)
    % region [params]
    rho = env_params.rho; % kg/m3
    prop_d = env_params.prop_d; % 8 inch = 20.3 cm
    
    CT_u = env_params.CT_u; % upper propeller thrust coefficient
    CT_l = env_params.CT_l; % lower propeller thrust coefficient
    CP_u = env_params.CP_u; % upper propeller drag coefficient
    CP_l = env_params.CP_l; % lower propeller drag coefficient

    f_max = drone_params.f_max;

    beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    P_prop = rho * prop_d^4 * beta_allo;

    C_z = [1 0 0 0 0 0;
            0 0 1 0 0 0;
            0 0 0 0 1 0;
            0 0 0 0 0 1];
    % end region [params]

    [Tf_0, eta_x0, eta_y0] = inverse_input(n, raw(1, :)');

    % region [boundary visualization]
    figure('Position', [10 10 1600 800])
    
    set(gcf, 'Renderer', 'painters')
    cmap = turbo(n); 

    for i = 7:7
        % subplot(2, ceil(n / 2), i);
        bound = bounds(i, :);
        lower_x = bound(1);
        upper_x = bound(2); 
        lower_y = bound(3); 
        upper_y = bound(4);

        % roof
        funcx = @(a, b, r) r .* cos(a) .* sin(b);
        funcy = @(a, b, r) - r .* sin(a);
        funcz = @(a, b, r) r .* cos(a) .* cos(b);

        funcx_r = @(a, b) funcx(a, b, f_max); 
        funcy_r = @(a, b) funcy(a, b, f_max); 
        funcz_r = @(a, b) funcz(a, b, f_max); 
        fsurf(funcx_r, funcy_r, funcz_r, [lower_x upper_x lower_y upper_y], 'FaceColor', '#77AC30', 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

        % positive y
        funcx_r = @(r, b) funcx(upper_x, b, r); 
        funcy_r = @(r, b) funcy(upper_x, b, r); 
        funcz_r = @(r, b) funcz(upper_x, b, r); 
        fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 
        % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

        funcx_r = @(r, b) funcx(-upper_x, b, r); 
        funcy_r = @(r, b) funcy(-upper_x, b, r); 
        funcz_r = @(r, b) funcz(-upper_x, b, r); 
        % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

        % negative y
        funcx_r = @(r, b) funcx(lower_x, b, r); 
        funcy_r = @(r, b) funcy(lower_x, b, r); 
        funcz_r = @(r, b) funcz(lower_x, b, r); 
        fsurf(funcx_r, funcy_r, funcz_r, [0 f_max lower_y upper_y], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 
        % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

        funcx_r = @(r, b) funcx(-lower_x, b, r); 
        funcy_r = @(r, b) funcy(-lower_x, b, r); 
        funcz_r = @(r, b) funcz(-lower_x, b, r); 
        % fsurf(funcx_r, funcy_r, funcz_r, [0 f_max 0 2*pi], 'FaceColor', "#4DBEEE", 'FaceAlpha', 0.1, 'EdgeColor', 'none'); hold on 

        title(i);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        axis equal

        zi = zs(6*i-5:6*i, 1);
        zio = C_z * zi;
        TfTdi = P_prop * zio(3:4, 1).^2;
        Tf = TfTdi(1);
        eta_x = zio(1);
        eta_y = zio(2);
        u = get_f(eta_x, eta_y, Tf);
        % raw(i, :, idx)
        % eta_x = raw(i, 1, idx);
        % eta_y = raw(i, 2, idx);
        % Tf = raw(i, 3, idx);
        % u = get_f(eta_x, eta_y, Tf);

        z_do = z_ds(4*i-3:4*i, 1);
        TfTdid = P_prop * z_do(3:4, 1).^2;
        Tf_d = TfTdid(1);
        eta_xd = z_do(1);
        eta_yd = z_do(2);
        % [eta_xd eta_yd Tf_d]
        u_d = get_f(eta_xd, eta_yd, Tf_d);
        % [eta_xd eta_yd Tf_d]
        f0 = raw(1, 3*i-2 : 3*i);
        f0_feasibility = [(lower_x <= eta_x0(i)) & (eta_x0(i) <= upper_x) (lower_y <= eta_y0(i)) & (eta_y0(i) <= upper_y) (0 <= Tf_0(i)) & (Tf_0(i) <= f_max)];
        fd_feasibility = [(lower_x <= eta_xd) & (eta_xd <= upper_x) (lower_y <= eta_yd) & (eta_yd <= upper_y) (0 <= Tf_d) & (Tf_d <= f_max)];
        fprintf("Agent %d\n", i)
        fprintf("f0: %d%d%d | fd: %d%d%d | raw f0: %3f, %3f, %3f \n", f0_feasibility, fd_feasibility, f0);

        quiver3(0, 0, 0, u(1), u(2), u(3), 'Color', '#000000', 'LineWidth', 2, 'AutoScale', 'off'); hold on
        quiver3(0, 0, 0, u_d(1), u_d(2), u_d(3), 'Color', '#0000AA', 'LineWidth', 2, 'AutoScale', 'off'); hold on
        for j=2:n
            if sum(raw(j, :)) == 0
                break;
            end
            f0 = raw(j-1, 3*i-2 : 3*i)';
            f = raw(j, 3*i-2 : 3*i)';
            ff = squeeze(intersections(j, i, 1, :));
            fn = squeeze(intersections(j, i, 2, :));
            fp = squeeze(intersections(j, i, 3, :));
            df = f - f0;
            if norm(df) < 1e-10
                % scatter3(f0(1), f0(2), f0(3), 'Color', '#AA0000', 'LineWidth', 2, 'Marker', '*'); hold on
            else 
                quiver3(f0(1), f0(2), f0(3), df(1), df(2), df(3), 'Color', cmap(j, :), 'LineWidth', 1, 'AutoScale', 'off'); hold on
                % scatter3(ff(1), ff(2), ff(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', 'x'); hold on
                plot3(ff(1), ff(2), ff(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', 'o'); hold on
                plot3([f(1) ff(1)], [f(2) ff(2)], [f(3) ff(3)], 'Color', cmap(j, :), 'LineWidth', 1, 'LineStyle', '--'); hold on

                plot3(fn(1), fn(2), fn(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', 'x'); hold on
                plot3([f(1) fn(1)], [f(2) fn(2)], [f(3) fn(3)], 'Color', cmap(j, :), 'LineWidth', 1, 'LineStyle', ':'); hold on
                plot3(fp(1), fp(2), fp(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', '+'); hold on
                plot3([f(1) fp(1)], [f(2) fp(2)], [f(3) fp(3)], 'Color', cmap(j, :), 'LineWidth', 1, 'LineStyle', ':'); hold on
                
                % for k=1:3
                %     ff = f0 + tw(j, i, k) .* (f-f0);
                %     scatter3(ff(1), ff(2), ff(3), 'Color', cmap(j, :));
                % end
            end
            [Tf_s, eta_xs, eta_ys] = inverse_input(n, raw(j, :)');
            fs_feasibility = [(lower_x <= eta_xs(i)) & (eta_xs(i) <= upper_x) (lower_y <= eta_ys(i)) & (eta_ys(i) <= upper_y) (0 <= Tf_s(i)) & (Tf_s(i) <= f_max)];
            fprintf("%d%d%d|", fs_feasibility);
            % pause
        end
        fprintf("\nTw:\n");

        for j=1:n
            fprintf("<%d> \t%d \t%d \t%d\n", j, squeeze(tw(j, i, :)))
        end
    end
    % end region [boundary visualization]

    sat_order
end
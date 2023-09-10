function [Tf_d, eta_xd, eta_yd, packed_meta] = allocator_interior_point(env_params, drone_params, u_d, z, dt, t)
    % Configurations
    P = drone_params.pos;
    psi = drone_params.psi;
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    f_max = drone_params.f_max;

    n = length(psi);
    W = eye(3*n);
    M = get_M(n, psi, P);

    N_esp = ones([n 1]);
    if drone_params.agent_disable && ~isempty(t)
        if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
            for m=1:length(drone_params.agent_disable_id)
                N_esp(drone_params.agent_disable_id(m)) = 0;
            end
        end
    end

    [Tf0, eta_x0, eta_y0] = z2raw(n, z, env_params);
    Tf0(Tf0==0) = 0.1;
    x0 = get_f(eta_x0, eta_y0, Tf0);
    % fprintf("Iter \n");
    % disp(x0)
    [lower_x, upper_x, lower_y, upper_y] = solve_upper_lower_bounds(drone_params, dt, Tf0, eta_x0, eta_y0);
    packed_meta = [lower_x upper_x lower_y upper_y];

    % x = fmincon(fun,    x0,  A,  b,Aeq,beq, lb, ub, nonlcon)
    % Aeq = diag([ones([n 1]); zeros([2*n 1])])
    options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'Display', 'off');
    Fi = fmincon(@(x)costfun(n, x, M, u_d), x0, [], [], [], [], [], [], @(x)mycon(n, N_esp, x, f_max, lower_x, upper_x, lower_y, upper_y), options);
    [Tf_d, eta_xd, eta_yd] = inverse_input(n, Fi);

    if drone_params.agent_disable && ~isempty(t)
        if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
            for m=1:length(drone_params.agent_disable_id)
                k = drone_params.agent_disable_id(m);
                Tf_d(k) = 0;
                eta_xd(k) = 0;
                eta_yd(k) = 0;
            end
        end
    end

    % constrain on angles
    function [c, ceq] = mycon(n, N_esp, xx, f_max, lower_x, upper_x, lower_y, upper_y)
        [Tf, eta_x, eta_y] = inverse_input(n, xx);
        % Tf = Tf .* N_esp;
        c = [-Tf;
            Tf - f_max;
            lower_x - eta_x;
            eta_x - upper_x;
            lower_y - eta_y;
            eta_y - upper_y];
        ceq = Tf .* (1-N_esp);
        % fprintf("con \n");
        % disp(c)
    end
    
    function y = costfun(n, xx, M, u_d)
        % Fi = reshape(xx, [3, n]);
        % Tf = sqrt(sum(Fi.^2, 1))';
        % y_f = Tf' * Tf;
    
        % y = sum((M*xx - u_d).^2) + y_f;
        y = sum((M*xx - u_d).^2);
        % fprintf("obj \n");
        % disp(y)
        % y = F' * W * F + 0.01 * (x - u0)' * eye(30) * (x - u0);
    end
end
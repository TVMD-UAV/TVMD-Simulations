function [Tf_d, eta_xd, eta_yd] = allocator_interior_point(drone_params, u_d, z)
    % Configurations
    P = drone_params.pos;
    psi = drone_params.psi;
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    f_max = drone_params.f_max;

    n = length(psi);
    W = eye(3*n);
    M = get_M(n, psi, P);

    x0 = zeros(3*n, 1);
    %Tf_d = zeros(n, 1);
    %u0 = get_f(a0, b0, f0);
%     u0 = [];
% 
    for i = 1:n
        x0(i * 3) = 10;
    end

    % x = fmincon(fun,    x0,  A,  b,Aeq,beq, lb, ub, nonlcon)
    options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'Display', 'off');
    Fi = fmincon(@(x)costfun(n, x, M, u_d), x0, [], [], [], [], [], [], @(x)mycon(n, x, f_max, sigma_x, sigma_y), options);
    [Tf_d, eta_xd, eta_yd] = inverse_input(n, Fi);


    % constrain on angles
    function [c, ceq] = mycon(n, xx, f_max, sigma_x, sigma_y)
        [Fc, ac, bc] = inverse_input(n, xx);
        c = [-Fc;
            Fc - f_max;
            -sigma_x - ac;
            ac - sigma_x;
            -sigma_y - bc;
            bc - sigma_y];
        ceq = [];
    end
    
    function y = costfun(n, xx, M, u_d)
        Fi_f = reshape(xx, [3, n]);
        F_f = sqrt(sum(Fi_f.^2, 1))';
        y_f = F_f' * F_f;
    
        y = sum((M*xx - u_d).^2) + y_f;
        
        % y = F' * W * F + 0.01 * (x - u0)' * eye(30) * (x - u0);
    end
end
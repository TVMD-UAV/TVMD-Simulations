function [c_star, i_star, N_esp, ti, tw, txn, txp] = calc_nearest_c2(N_esp, u0, ud, lower_x, upper_x, lower_y, upper_y, f_max)
    % u0: allocation result in previous step
    % ud: temporary target

    % u is 3n x 1 column vector
    % u_xyz is 3 x n matrix
    % v is alias for u / tan
    n = length(N_esp);
    u_xyz = reshape(u0, [3 n]);
    u_delta = ud - u0;
    u_delta_xyz = reshape(u_delta, [3, n]);
    % u_delta_xyz(:, ~N_esp) = 0; % disabled actuators

    % Determine direction
    [f0, a0, b0] = inverse_input(n, u0);
    [f1, a1, b1] = inverse_input(n, ud);
    % norm(get_f(a1, b1, f1) - ud)
    a_s = lower_x;
    b_s = lower_y;
    a_s(a1 - a0 > 0) = upper_x(a1 - a0 > 0);
    b_s(b1 - b0 > 0) = upper_y(b1 - b0 > 0);
%     [df0, da0, db0] = inverse_input(n, ud-u0);
%     % norm(get_f(a1, b1, f1) - ud)
%     a_s = lower_x;
%     b_s = lower_y;
%     a_s(da0 > 0) = upper_x(da0 > 0);
%     b_s(db0 > 0) = upper_y(db0 > 0);

    % Finding nearest
    [tw, txn, txp] = solve_intersections(u_xyz, u_delta_xyz, a_s, b_s, f_max);

    % Checking direction of delta
    is_at_origin = norm(u_xyz(3, :)) == 0;
    [df1, da1, db1] = inverse_input(n, u_delta);
    mask_da = (lower_x <= da1) & (da1 <= upper_x); % in range
    mask_db = (lower_y <= db1) & (db1 <= upper_y); 
    t_dir = is_at_origin' & mask_da & mask_db;

    % Checking direction of delta
    tw(tw < 0) = inf;        % Not intersecting in forward direction
    ti = min(tw, [], 2);     % The nearest collision in the 3 DOFs
    ti(N_esp <= 0) = inf;    % 
    ti(t_dir > 0) = inf;

    [c_star, i_star] = min(ti); % deal with the case of multiple minimums
    i_star = find(ti == c_star);
end
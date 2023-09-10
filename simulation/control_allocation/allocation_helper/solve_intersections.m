function [tw, txn, txp] = solve_intersections(u_xyz, u_delta_xyz, a_s, b_s, f_max)
    % a_s: bound of a (pos / neg sigmal_a, according to the direction of delta)
    % b_s: bound of a (pos / neg sigmal_b, according to the direction of delta)
    v_delta_y = u_delta_xyz(2, :) ./ (tan(a_s)');
    v_xyz_y = u_xyz(2, :) ./ (tan(a_s)');
    v_norm = vecnorm(u_delta_xyz, 2, 1)';
    tf0 = vecnorm(u_xyz, 2, 1)';

    % y-axis
    txa = u_delta_xyz(1, :)'.^2 + u_delta_xyz(3, :)'.^2 - v_delta_y'.^2;
    txb = u_xyz(1, :)' .* u_delta_xyz(1, :)' + u_xyz(3, :)' .* u_delta_xyz(3, :)' - v_xyz_y' .* v_delta_y';
    txc = u_xyz(1, :)'.^2 + u_xyz(3, :)'.^2 - v_xyz_y'.^2;
    tx1 = (-txb - sqrt(abs(txb.^2 - txa .* txc))) ./ txa;
    tx2 = (-txb + sqrt(abs(txb.^2 - txa .* txc))) ./ txa;

    txn = tx1; % Make the txn really the smaller one
    txn(tx2 < tx1) = tx2(tx2 < tx1);
    txp = tx2; % Make the txp really the larger one
    txp(tx2 < tx1) = tx1(tx2 < tx1);
    tx = txn; % Choose the non-negative one
    tx(tx < 0) = txp(tx < 0);

%     tx = (-txb - sqrt(abs(txb.^2 - txa .* txc))) ./ txa;
%     txp = (-txb + sqrt(abs(txb.^2 - txa .* txc))) ./ txa;
%     txn = tx;
%     tx(tx < 0) = txp(tx < 0);
    
    mask_no_sol = txb.^2 - txa .* txc < 0;
    tx(mask_no_sol) = inf;

    txl = -txc ./ (2 * txb);
    mask_a_small = abs(txa) < 1e-20;
    tx(mask_a_small) = txl(mask_a_small);

    % x-axis
    if tan(b_s) > 1e10
        % sigma_y = pi/2, tan -> inf
        ty = - u_xyz(3, :)' ./ u_delta_xyz(3, :)';
    else
        ty = -(u_xyz(1, :)' - u_xyz(3, :)' .* tan(b_s)) ./ (u_delta_xyz(1, :)' - u_delta_xyz(3, :)' .* tan(b_s));
        mask_no_sol = (u_delta_xyz(1, :)' == (u_delta_xyz(3, :)' .* tan(b_s)));
        ty(mask_no_sol) = inf;
    end

    % z-axis
%     kkb = dot(u_xyz, u_delta_xyz, 1)' ./ v_norm.^2;
%     kkc = (tf0 .* tf0 - f_max^2) ./ v_norm.^2;
%     tz = -kkb + sqrt(abs(kkb .* kkb - kkc));
    kkb = dot(u_xyz, u_delta_xyz, 1)';
    kkc = (tf0 .^ 2 - f_max^2);
    tz = (-kkb + sqrt(abs(kkb .* kkb - v_norm.^2 .* kkc))) ./ v_norm.^2;

    mask_a_small = abs(v_norm.^2) < 1e-20;
    tzl = -kkc ./ (2 * kkb);    
    tz(mask_a_small) = tzl(mask_a_small);

    mask_no_sol = kkb .^ 2 - v_norm.^2 .* kkc < 0;
    tz(mask_no_sol) = inf;
    %tz(tz<0) = 0;

%     tz = -kkb + sqrt(abs(kkb .* kkb - kkc));  
%     % the case of moving in an opposite direction
%     tzp = -kkb - sqrt(abs(kkb .* kkb - kkc));
%     tz(tz < 0) = tzp(tz < 0);
%     mask_no_sol = kkb .* kkb - kkc < 0;
%     tz(mask_no_sol) = inf;

    % disp(size(ty))
    % disp(size(tx))
    % disp(size(tz))

    tw = [ty tx tz];
end
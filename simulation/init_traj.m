function traj = init_traj(t)
    persistent traj_func
    if isempty(traj_func)
        syms ts
        %zeta = [3 * sin(1 * ts); 2 * ts; 3 * sin(1 * ts); 0 * ts-pi/3; 0 * ts; 0 * ts-pi/2];
        %zeta = [3*sin(1*ts); 2*ts; 1*sin(1*ts); 0 * ts-pi/4; 0 * ts; 0 * ts-pi/2];
        zeta = [3*sin(1*ts); 2*ts; 1*sin(1*ts); 0 * ts-pi/4; 0 * ts; 0 * ts-pi/2];
        d_zeta = diff(zeta);
        dd_zeta = diff(d_zeta);
        ddd_zeta = diff(dd_zeta);
        %desire = [zeta d_zeta dd_zeta ddd_zeta];
        desire = [zeta d_zeta dd_zeta];
        traj_func = matlabFunction(desire);
    end
    traj = traj_func(t);
    %traj = zeros([6 3]);
end

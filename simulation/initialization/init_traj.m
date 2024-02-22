function traj = init_traj(t, traj_type, drone_type, traj_params)
    persistent traj_func

    if traj_type == "sine"
        if isempty(traj_func)
            traj_func = traj_sine(drone_type, traj_params);
        end
        traj = traj_func(t);
    elseif traj_type == "regulation"
        if drone_type == "single"
            traj = zeros([4 3]);
        else
            traj = zeros([6 3]);    
        end
    elseif traj_type == "hover_tilt"
        traj = traj_hover_tilt(t, drone_type, traj_params);
    end
    
    if traj_params.sudden_change && (t > traj_params.sudden_time_start) && (t < traj_params.sudden_time_end)
        traj(1:3, 1) = traj(1:3, 1) + traj_params.sudden_pos_offset;
    end
end

function traj = traj_hover_tilt(t, drone_type, traj_params)
    up_time = 2;
    hover_time = 10;
    land_time = 2;
    shutdown_time = 2;

    h = 1.5;
    T = 2;
    h_tilt = pi / 16;
    T_tilt = 6;

    t1 = up_time;
    t2 = t1 + hover_time;
    t3 = t2 + land_time;
    t4 = t3 + shutdown_time;

    if drone_type == "single"
        % Only team system is supported currently
        zeta = [0; 0; 0; 0];
    else
        x_zr = 0;
        v_zr = 0;
        phi_r = 0;
        if t < t1  % Ascending
            x_zr = (0.5 * h * (1 - cos(pi * (t - 0) / T)));
            v_zr = 0.5 * h* pi / T * sin(pi * (t - 0) / T);
        elseif t >= t1 && t < t2 % Hover
            x_zr = h;

            t_m = (t1 + t2) / 2;
            if t > t_m - T_tilt / 2 && t < t_m + T_tilt / 2
                % Scale for manual control: 1000
                phi_r = h_tilt * (1 + cos(2 * pi / T_tilt * (t - t_m)));
            end
        elseif t >= t2 && t < t3 % Descending
            x_zr = h - (0.5 * h * (1 - cos(pi * (t - t2) / T)));
            v_zr = -0.5 * h* pi / T * sin(pi * (t - t2) / T);
        elseif t >= t3 && t < t4
            x_zr = (t - t3);
        elseif t >= t4
            x_zr = -10;
        end
        zeta = [0; 0; x_zr; 0; phi_r; 0];   % x, y, z, z(psi), x(phi), y(pitch)
        d_zeta = [0; 0; v_zr; 0; 0; 0];
    end
    dd_zeta = d_zeta * 0;
    %desire = [zeta d_zeta dd_zeta ddd_zeta];
    traj = [zeta d_zeta dd_zeta];
end

function traj_func = traj_sine(drone_type, traj_params)
    syms ts
    if drone_type == "single"
        zeta = [traj_params.amps(1) * sin(traj_params.freq(1)*ts + traj_params.phase(1)) + traj_params.offset(1,1)*ts + traj_params.offset(1,2); 
                traj_params.amps(2) * sin(traj_params.freq(2)*ts + traj_params.phase(2)) + traj_params.offset(2,1)*ts + traj_params.offset(2,2); 
                traj_params.amps(3) * sin(traj_params.freq(3)*ts + traj_params.phase(3)) + traj_params.offset(3,1)*ts + traj_params.offset(3,2); 
                traj_params.amps(4) * sin(traj_params.freq(4)*ts + traj_params.phase(4)) + traj_params.offset(4,1)*ts + traj_params.offset(4,2)];
    else
        zeta = [traj_params.amps(1) * sin(traj_params.freq(1)*ts + traj_params.phase(1)) + traj_params.offset(1,1)*ts + traj_params.offset(1,2); 
                traj_params.amps(2) * sin(traj_params.freq(2)*ts + traj_params.phase(2)) + traj_params.offset(2,1)*ts + traj_params.offset(2,2); 
                traj_params.amps(3) * sin(traj_params.freq(3)*ts + traj_params.phase(3)) + traj_params.offset(3,1)*ts + traj_params.offset(3,2); 
                traj_params.amps(4) * sin(traj_params.freq(4)*ts + traj_params.phase(4)) + traj_params.offset(4,1)*ts + traj_params.offset(4,2);
                traj_params.amps(5) * sin(traj_params.freq(5)*ts + traj_params.phase(5)) + traj_params.offset(5,1)*ts + traj_params.offset(5,2);
                traj_params.amps(6) * sin(traj_params.freq(6)*ts + traj_params.phase(6)) + traj_params.offset(6,1)*ts + traj_params.offset(6,2)];
    end
    d_zeta = diff(zeta);
    dd_zeta = diff(d_zeta);
    ddd_zeta = diff(dd_zeta);
    %desire = [zeta d_zeta dd_zeta ddd_zeta];
    desire = [zeta d_zeta dd_zeta];
    traj_func = matlabFunction(desire);
end

function traj_func = traj_regulation(drone_type)
    syms ts
    %zeta = [3 * sin(1 * ts); 2 * ts; 3 * sin(1 * ts); 0 * ts-pi/3; 0 * ts; 0 * ts-pi/2];
    %zeta = [3*sin(1*ts); 2*ts; 1*sin(1*ts); 0 * ts-pi/4; 0 * ts; 0 * ts-pi/2];
    if drone_type == "single"
        zeta = [0*ts; 0*ts; 0*ts; 0*ts];
    else
        zeta = [0; 0; 0; 0; 0; 0] * ts;
    end
    
    d_zeta = diff(zeta);
    dd_zeta = diff(d_zeta);
    %desire = [zeta d_zeta dd_zeta ddd_zeta];
    desire = [zeta d_zeta dd_zeta];
    traj_func = matlabFunction(desire);
end
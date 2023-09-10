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
    end
    
    if traj_params.sudden_change && (t > traj_params.sudden_time_start) && (t < traj_params.sudden_time_end)
        traj(1:3, 1) = traj(1:3, 1) + traj_params.sudden_pos_offset;
    end
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
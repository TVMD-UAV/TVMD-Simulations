function offset = euler_crossover(angle, angle0)
    if angle - angle0 > pi
        offset = -2*pi;
    elseif angle - angle0 < -pi
        offset = 2*pi;
    else
        offset = 0;
    end
end
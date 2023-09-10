function [avr, vmin, vmax, integ] = cal_statistic(t, d)
    tPulse = t(1):0.1:t(end);
    avr = mean(tsa(d, t, tPulse));
    vmin = min(d);
    vmax = max(d);
    
    integ = 1 *  dot(d(2:end), (t(2:end) - t(1:end-1)));
end

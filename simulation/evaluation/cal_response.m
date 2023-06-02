function [yfinal, peak, settling] = cal_response(t, d)
    % S = stepinfo(d,t, 0);
    yfinal = mean(d(length(d)-10:end));
    % peak = S.Peak;
    peak = max(d);
    % settling = S.SettlingTime;
    settle_range = 0.03 * peak; 
    mask = (d > settle_range);
    [m, v] = min(abs(1-t));
    mask(1:v) = 1;
    t(mask) = inf;
    [~, settling_i] = min(t);
    settling = t(settling_i);
    % min_v = min(d);
    % average_v = mean(d);
    % max_v = max(d);
    % steady_avr = mean(floor(length(d) * 0.9):end, d);
end

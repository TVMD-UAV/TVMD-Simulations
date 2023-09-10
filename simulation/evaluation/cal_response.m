function [yfinal, peak, settling] = cal_response(t, d)
    % S = stepinfo(d,t, 0);
    peak = max(d);
    stable_tolerance = 1e-3 * peak;
    last_d = d(length(d)-10:end);
    yfinal = mean(last_d);
    if (max(last_d) - min(last_d)) > stable_tolerance
        % Not converging
        settling = inf; 
        return; 
    end
    % peak = S.Peak;
    % settling = S.SettlingTime;
    % settle_range = 0.03 * d(1); 
    settle_range = 0.02 * peak; 
    % settle_range = 0.1; 
    % settle_range = 0.1 * peak; 
    mask = (d - yfinal > settle_range)';
    [settling, v] = max(t .* mask);
    % [d' t .* mask]
    
    % [m, v] = min(abs(0.1-t));
    % mask(1:v) = 1;
    % t(mask) = inf;
    % [~, settling_i] = min(t);
    % settling = t(settling_i);
    % min_v = min(d);
    % average_v = mean(d);
    % max_v = max(d);
    % steady_avr = mean(floor(length(d) * 0.9):end, d);
end

function y = f_phi_h(u, v)
    y = 3 * (skew(u)^2 - eye(3)) * v * u' / sqrt(1 + u'*u)^5 + ...
        (1 + u'*u) * (2*skew(u)*skew(v)-skew(v)*skew(u));
end
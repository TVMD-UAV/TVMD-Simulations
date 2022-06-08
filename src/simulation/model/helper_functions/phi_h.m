function y = phi_h(x)
    y = (eye(3) - skew(x)^2) / sqrt(1 + x'*x)^3;
end
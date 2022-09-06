function Q = theta_vector2Q(theta, u)
    Q = [cos(theta/2); sin(theta/2)*u/norm(u) ];
end
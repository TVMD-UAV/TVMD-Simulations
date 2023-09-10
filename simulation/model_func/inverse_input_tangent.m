function [tf1, x1, y1] = inverse_input_tangent(n, f0, f1)
    [tf, x, y] = inverse_input(n, f0);
    if (uint8(n) == 1)
        f_delta = f1 - f0;
        ff = 0.1 * f_delta / norm(f_delta);
        % ff = f0 + 0.1 * f_delta / norm(f_delta);
        jac_f = [-tf*sin(x)*sin(y)   tf*cos(x)*cos(y)  cos(x)*sin(y); ...
                        -tf*cos(x)                  0        -sin(x); ...
                 -tf*cos(y)*sin(x)  -tf*cos(x)*sin(y)  cos(x)*cos(y)];
        out = jac_f \ ff;
        x1 = out(1); y1 = out(2); tf1 = out(3);
    else
        out = zeros([3 n]);
        for i=1:n
            f_delta = f1(3*i-2 : 3*i) - f0(3*i-2 : 3*i);
            ff =  0.1 * f_delta / norm(f_delta);
            % ff = f0(3*i-2 : 3*i) + 0.1 * f_delta / norm(f_delta);
            jac_f = [-tf(i)*sin(x(i))*sin(y(i))   tf(i)*cos(x(i))*cos(y(i))  cos(x(i))*sin(y(i)); ...
                        -tf(i)*cos(x(i))                                  0        -sin(x(i)); ...
                 -tf(i)*cos(y(i))*sin(x(i))  -tf(i)*cos(x(i))*sin(y(i))  cos(x(i))*cos(y(i))];
            out(:, i) = jac_f \ ff;
        end
        x1 = out(1, :)'; y1 = out(2, :)'; tf1 = out(3, :)';
    end
end
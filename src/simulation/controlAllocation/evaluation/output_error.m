function [ef, em] = output_error(u_des, u)
    ef = 100 * norm(u_des(1:3) - u(1:3)) / norm(u_des(1:3));
    em = 100 * norm(u_des(4:6) - u(4:6)) / norm(u_des(4:6));
end

function [ef, em, df, dm] = output_error(u_des, u)
    esp = 0.01;
    % ef = 100 * norm(u_des(1:3) - u(1:3)) / norm(u_des(1:3));
    ef = norm(u_des(1:3) - u(1:3));
    em = norm(u_des(4:6) - u(4:6));
    % em = 100 * norm(u_des(4:6) - u(4:6)) / (norm(u_des(4:6)) + esp);
    df = (u_des(1:3)' * u(1:3)) / (norm(u_des(1:3)) * norm(u(1:3)));
    dm = (u_des(4:6)' * u(4:6)) / (norm(u(4:6)) + esp);
    % dm = (u_des(4:6)' * u(4:6)) / (norm(u_des(4:6)) * norm(u(4:6)) + esp);
end

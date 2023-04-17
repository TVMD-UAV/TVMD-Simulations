function [pos, psi] = model_A10_con(GRID_SIZE)
    pos = [ 4  3 3  2 2 2  1  1 1 1;
            0 -1 1 -2 0 2 -3 -1 1 3;
            0  0 0  0 0 0  0  0 0 0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [0 0 0 0 0 0 0 0 0 0];
end
function [pos, psi] = model_A5_con(GRID_SIZE)
    pos = [ 1  1 0 -1 -1;
            1 -1 0  1 -1;
            0  0 0  0 0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [0 0 0 0 0];
end
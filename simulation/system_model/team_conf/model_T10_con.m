function [pos, psi] = model_T10_con(GRID_SIZE)
    pos = [-1 1 0 -1 1 -1 1 -2 0 2;
            3 3 2 1 1 -1 -1 -2 -2 -2;
            0 0 0 0 0 0 0 0 1 0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [0 0 0 0 0 0 0 0 0 0];
end
function [pos, psi] = model_A5_inc(GRID_SIZE)
    pos = [ 1  1 0 -1 -1;
            1 -1 0  1 -1;
            0  0 0  0 0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [pi/2 pi/2 pi/2 0 0];
end
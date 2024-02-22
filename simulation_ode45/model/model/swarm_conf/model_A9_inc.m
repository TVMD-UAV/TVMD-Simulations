function [pos, psi] = model_A9_inc(GRID_SIZE)
    pos = [ 2  1 1  0 0 0 -1 -1 -2;
            0 -1 1 -2 0 2 -1  1  0;
            0  0 0  0 0 0  0  0  0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [0 pi/2 pi/2 0 pi/2 0 pi/2 pi/2 0];
end

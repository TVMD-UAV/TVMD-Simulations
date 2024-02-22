function [pos, psi] = model_A3_inc(GRID_SIZE)
    pos = [ 4  3 3;
            0 -1 1;
            0  0 0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [pi/2 0 0];
end
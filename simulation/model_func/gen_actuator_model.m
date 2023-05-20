function [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp)
    A_z = [   0    1    0    0    0    0;
           -mKp -mKd    0    0    0    0;
              0    0    0    1    0    0;
              0    0 -mKp -mKd    0    0;
              0    0    0    0 -pKp    0;
              0    0    0    0    0 -pKp];
    B_z = [   0    0    0    0;
            mKp    0    0    0;
              0    0    0    0;
              0  mKp    0    0;
              0    0  pKp    0;
              0    0    0  pKp];
    C_z = [1 0 0 0 0 0;
           0 0 1 0 0 0;
           0 0 0 0 1 0;
           0 0 0 0 0 1];
    end
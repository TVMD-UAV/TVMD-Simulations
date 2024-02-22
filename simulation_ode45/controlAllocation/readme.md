# Control Allocation

## Quick start
- Run `control_allocation_test.m` to test the algorihtm with sequential data.
- Run `test_allocator_*.m` to test separate allocation algorithm separately, where `*` is the name of the algorithm.

## Algorithms

### fmincon() interior point method
This method directly applies [fmincon](https://www.mathworks.com/help/optim/ug/fmincon.html) to solve the original optimization problem with the default `interior-point` method. This method is regarded as the baseline.

### Nullspace-based control allocation
This is the implementation of [1: Su et al. 2021], which minimize the difference to the previous state and the vector in nullspace. The QP problem is solved with [quadprog](https://www.mathworks.com/help/optim/ug/quadprog.html) where the default `interior-point-convex` algorithm is used.

### Redistributed control allocation
This is a modification from [2: Stephan & Fichter 2019] to apply to a nonlinear system.

## References
- [1: Su et al. 2021]
    Yao Su, Pengkang Yu, Matthew J. Gerber, Lecheng Ruan, and Tsu-Chin Tsao, “Nullspace-Based Control Allocation of Overactuated UAV Platforms,” IEEE Robotics and Automation Letters, Vol. 6, No. 4, pp. 8094–8101, Oct. 2021.
- [2: Stephan & Fichter 2019]
    Johannes Stephan and Walter Fichter, “Fast Exact Redistributed Pseudoinverse Method for Linear Actuation Systems,” IEEE Transactions on Control Systems Technology, Vol. 27, No. 1, pp. 451–458, Jan. 2019.

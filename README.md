# RPS-Solver
A solver for 3 degree of freedom revolute-prismatic-spherical (RPS) stabilization platform

![DevCar](https://github.com/RPS-Solver/blob/main/resources/rps_3dof.png)

### Approach 1
1. Assume the centre of the top base is of fixed height and position
2. A change in length of each actuator creates a rotation in the top base about a fixed axis
3. The length of the other 2 actuators are then adjusted to accomodate this rotation in the top base

# Kinematics / Dynamics
 
Code developed while doing course assignments to compute the kinematics and dynamics of different robotic systems. 

Contents:
1. FK/IK for a 5-DOF [Lynxmotion](http://www.lynxmotion.com/c-130-al5d.aspx) manipulator
   - For forward kinematics, all the joint positions and transformation matrices are computed using DH method. 
   - For inverse kinematics, the code uses kinematic decoupling and havs checked joint limits, feasibility, and considered other edge cases. 
2. 2D Pendulum Dynamics
   - The `dynamics.m` uses the Euler-Lagrange method to derive the closed-form equation of motion for the pendulum model. The symbolic expression has been copied to the `solution.m` script, so run `solution.m` directly to solve the ODE and plot for joint variable values w.r.t. time (assume torque = 0).
   - The other type of dynamics problem it solves is that if we know the functions of joint variables w.r.t. time, run `torque.m` to get the desired torques to generate this trajectory.
   - The code solves the dynamics problem of system DOF=2, but it is easy to be revised for solving higher-order pendulum dynamics models. However, some matrix manipulation may be needed beforehand to simplify the computation since the differentiation in MATLAB takes a lot more time as the number of symbolic variables grow.

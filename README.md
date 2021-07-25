# Kinematics / Dynamics
 
Code developed while doing course assignments for the kinematics and dynamics of different robotic systems. 

Contents:
1. FK/IK for a 5-DOF [manipulator](http://www.lynxmotion.com/c-130-al5d.aspx)
   - For forward kinematics, all the joint positions and transformation matrices are computed using DH method. 
   - For inverse kinematics, the code uses kinematic decoupling and has checked joint limits, feasibility, and considered other edge cases.
   - Call functions `calculateFK` and `calculateIK` to compute the results.
2. 2D pendulum dynamics
   - The `dynamics.m` uses the Euler-Lagrange method to derive the closed-form equation of motion for the pendulum model. The symbolic expression has been copied to the `solution.m` script. Run `solution.m` directly to solve the ODE and plot for joint variable values w.r.t. time (assume torque = 0).
   - The other type of dynamics problem it solves is that when the functions of joint variables w.r.t. time is known. In this case, run `torque.m` to compute the desired torques to along this trajectory.
   - The code solves the dynamics problem of system DOF = 2, but it is easy to be revised for solving higher-order pendulum dynamics models. However, some matrix manipulation may be needed beforehand to simplify the computation since the differentiation in MATLAB takes a lot more time as the number of symbolic variables grow.
3. Velocity kinematics for a mobile manipulator
   - The mobile manipulator is a 3-joint manipulator with a mobile base (differential drive). The `jacobian.m` code stores the Jacobian matrix of the system, which is used to transform the end-effector velocity to each joint's velocity. The `main.m` simulates the motion and `animate.m` creates the animation.
   - There are 4 types of tests you can run in `main.m`. Two of them are shown in the animation below. In these two tests, the end-effector draws a square shape. The code computes and executes the velocity of each joint using the functions mentioned above to achieve the desired motion.
   - Test 1 (mobile base unlocked)

   ![image](https://github.com/tiansy97/Kinematics-Dynamics/blob/master/Mobile%20manipulator/test1.gif)
   
   - Test 2 (mobile base locked)

   ![image](https://github.com/tiansy97/Kinematics-Dynamics/blob/master/Mobile%20manipulator/test2.gif)
   

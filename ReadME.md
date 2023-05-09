## StartUp ##
Firstly, run **"startup.m"** to add the file path to matlab workspace.
## Visualization ##
Using **showmotion(model, t_data, q_data)** function for animation, exactly using **“showmotion(quadruped_robot, tout, fbanim(Xout, Qout))”** for the quadruped simulation. The function **Q=fbanim(X)** and **Q=fbanim(X,Qr)** are designed to calculate the joint angle matrix including the 3 revolute angles and 3 prismatic angles of floating base, where **"X"** contains 13 singularity-free elements including a unit **quaternion** specifying the orientation of the floating-base frame relative to the fixed base, a 3D vector giving the **position** of the origin of the floating-base frame, and the **spatial velocity** of the floating base expressed in fixed-base coordinates. **"Qr"** is joint angles. For details, check the denefination of each function.

## Simulink ##
The whole frame is constructed referring to "MIT Cheetah Mini". The forward dynamics via **"Articulated-Body Algorithm"** is the same as CheetahMini's. The contact and friction forces are calculated using a nonlinear spring-damper model which is packaged as the simulink module("3D Ground Contact") in spatial-V2

### Initialization ###
Run **freego_parameter.m** in "/models" folder to load the kinematics and dynamics parameters of the robot.

### Leg Controller ###
It's realized by simulink subsystems which is nearly the same as "LegController.cpp" in Cheetah-Software.

### State Machine ###
This module is designed to plan jumping trajectory in time schedule manually, which is very stupid.


# ece470
# update 1
We used CoppeliaSim V4.0.0 EDU as our simulator.
We imported built-in robot model(ant hexapod) and sensor(passage counter).
You can just copy our "Current_code for simple movement" and paste it on the script of ant robot.
Ant robot will go straight without ant other movement after clicking "Start simulation" button.

# update 2
In this update, we worked more on using our knowledge from ECE470 labs on our own UR3 robot. The objective is to write a forward kinematics function that calculates the endpoint position using 6 arbitrary joint angles. Because V-rep already has built-in functionality that moves the robot to desired angular positions, we will use it to check if our forward kinematics could calculate the correct position of the robot in XYZ coordinates. 

# update 3
We used python to remote control the robot simulation using remote api. Download and open and run the scene, and run the python code to see the robot's animation run.

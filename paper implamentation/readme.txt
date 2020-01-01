
Tools needed:
-------------
1. simulation software used for autonomus robot simulation: "Gazebo 7.1" http://gazebosim.org/


2. Programing language: python 3.6 / 2.7


3. Os: ubuntu 16.0.4 (important Gazebo 7.1 only works in ubuntu 16.0.4)



Instruction(setup):
-------------------

1. first need to have ubuntu 16.0.4 installed pc (can be installed through vmware virtual machine )

2. Need to install install ros(http://wiki.ros.org/kinetic/Installation/Ubuntu), Gazebo 7.1(http://gazebosim.org/) with 

  
   simply way:
   runing command  "bash ros_with_gazebo_7.sh"



Instruction (simulation):
------------------------

1. opening terminal in the code directory and 
   running command 
   "ROBOT_INITIAL_POSE="-x -8.0 -y -8.0 -z 0.0" roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/work56/Pictures/code/world1.world"

2.Again opening another terminal in the code directory and 
   running command
   "python start_simulation.py


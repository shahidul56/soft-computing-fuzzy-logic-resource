
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
   




![Screenshot 2019-12-29 20^%35^%37](https://user-images.githubusercontent.com/11449967/71638508-050cf400-2c8c-11ea-82f3-a00d04623fa1.png)
![Screenshot 2019-12-29 20^%35^%47](https://user-images.githubusercontent.com/11449967/71638509-050cf400-2c8c-11ea-9c69-8d6479a095c2.png)
![Screenshot 2019-12-29 20^%36^%00](https://user-images.githubusercontent.com/11449967/71638506-04745d80-2c8c-11ea-9667-a2b4f65d0c46.png)
![Screenshot 2019-12-29 20^%36^%10](https://user-images.githubusercontent.com/11449967/71638507-04745d80-2c8c-11ea-96ff-532afb049cb0.png)


# Part2
This project implements A star algorithm on 2D map.  
Based on the code in 2D implement, use turtlebot3 to simulate the path in Gazebo.  
## Dependencies
Ubuntu-20.04 neotic  
Gazebo  
Python3.8  
ros-neotic-turtlebot3  
## Build
To build the package, create a workspace/src and open a terminal under the workspace  
Type  
```bash
catkin_make
```
Don't foget to source the workspace  
Open a terminal under the workspace and type   
```bash
source devel/setup.bash
```
## Run
To run the code, there are some parameters could adjust while typing  
```bash
rosrun turtlebot3_plan plan.py -s -4 -4 0 -g 4 4 -r 10 7 -c 0.1
#this command wouldn't show the simulation but only figures
#-s means start position and theta
#-g means goal position
#-r means RPM of two wheels
#-c means clearance 
```
To simulate the robot in Gazebo, type  
```bash
roslaunch turtlebot3_plan plan.launch
#this will run default parameter
#-s -4 -4 0 -g 4 4 -r 10 7 -c 0.1
```
To change the parameters, call  
```bash
roslaunch turtlebot3_plan plan.launch --ros-args
```
GitHub link: https://github.com/liudiepie/A_star_in_Gazebo/tree/main/turtlebot3_plan
Youtube link: https://youtu.be/Dgqk7kelJsQ  
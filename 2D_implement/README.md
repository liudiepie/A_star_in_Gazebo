# Part1
This project implements A star algorithm on 2D map.  
Based on the code in 2D implement, use turtlebot3 to simulate the path in Gazebo.  
## Dependencies
Python3.8, matplotlib, math, heapq, cv2  
## Run 
To run the code, open the terminal under the file and type  
```bash
python3 proj3_2.py
```
There will show the following inputs in terminal.  
User can choose the valid values they want.  
For example,  
```bash
Enter the x and y coordiante of the start point: 
#type 1 1
```
The start point will be at (1,1)  
```bash
Enter the start theta: 
#type 30
```
The start angle will be 30  
```bash
Enter the x and y coordiante of the goal point:  
#type 9 9
```
The goal point will be at (9,9)  
```bash
Enter RPM1 and RPM2, default values are 10, 7:  
#type 10 7 or enter
```
The RPM of two wheels will be 10 and 7  
```bash
Enter the clearance: 
#type 0.1
```
The clearance will be 0.1 meter  
## Result
Based on the parameters above, it will produce one video and one image.  
![](<proj3_2.png>)
GitHub link: https://github.com/liudiepie/A_star_in_Gazebo/tree/main/2D_implement
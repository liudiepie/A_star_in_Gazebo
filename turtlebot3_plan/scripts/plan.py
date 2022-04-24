#!/usr/bin/env python3
# -*-coding:utf-8 -*-
'''
@File    :   plan.py
@Time    :   2022/04/23 15:35:17
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   None
'''

# for testing: start: -4 -4 theta: 30 goat: 4 4 clearance: 0.1
import matplotlib.pyplot as plt
import numpy as np
import math
import heapq
import sys
import argparse
import rospy
from geometry_msgs.msg import Twist


# default input args
DEFAULT_START = [-4, -4, 0]
DEFAULT_GOAL = [4, 4]
DEFAULT_CLEARANCE = 0.1
DEFAULT_RPM = [10.0, 7.0]
radius = 0.105
L = 0.16
path =[]

class Node:
    """
    Node class : This class is built to store the node information.
    For each node, its neighbours, parents & distance to reach that node is stored.
    """

    def __init__(self, i, j, endI, endJ, theta):
        self.i = i
        self.j = j
        self.theta = theta
        self.costToCome = 0.0
        self.costToGo = 2.5*(math.sqrt((i - endI) ** 2 + (j - endJ) ** 2))
        self.cost = None
        self.neighbours = {}
        self.valid_actions = {}
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class graph:
    """
    graph class : This class defines all methods to generate a graph and perform AStar Algorithm.
    """

    def __init__(self, start, end, RPM1, RPM2, CLEARANCE):
        self.visited = {}
        self.startI = start.i
        self.startJ = start.j
        self.endI = end.i
        self.endJ = end.j
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.CLR = CLEARANCE + radius

    def getNeighbours(self, currentNode):
        """
        Description: Returns neighbours for the currentNode.
        """
        i, j, theta = currentNode.i, currentNode.j, currentNode.theta
        neighbours = {}
        valid_actions = {}
        actions = [[0, self.RPM1], [self.RPM1, 0], [self.RPM1, self.RPM1], [0, self.RPM2], [self.RPM2, 0], [self.RPM2, self.RPM2], [self.RPM1, self.RPM2], [self.RPM2, self.RPM1]]
        for UL, UR in actions:
            x, y, newTheta, distance = self.cost(i,j,theta,UL,UR)
            if (not self.isOutsideArena(x, y)) and (not self.isAnObstacle(x, y)):
                newNode = Node(x, y, self.endI, self.endJ, newTheta)
                neighbours[newNode] = distance
                valid_actions[newNode] = [UL, UR]
        return neighbours, valid_actions

    def inside_rect1(self,x,y):
        return (y-0.75-self.CLR<=0 and x+3.25-self.CLR<=0 and y+0.75+self.CLR>=0 and x+4.75+self.CLR>=0)

    def inside_rect2(self,x,y):
        return (y-0.75-self.CLR<=0 and x-1.25-self.CLR<=0 and y+0.75+self.CLR>=0 and x+1.25+self.CLR>=0)
                
    def inside_rect3(self,x,y):
        return (y+1-self.CLR<=0 and x-3.75-self.CLR<=0 and y+3+self.CLR>=0 and x-2.25-self.CLR>=0)

    def inside_circle1(self,x,y):
        return ((x+3)**2+(y-3)**2-(1+self.CLR)**2<=0)

    def inside_circle2(self,x,y):
        return ((x+3)**2+(y+3)**2-(1+self.CLR)**2<=0)

    def generate_map(self):
        fig, ax = plt.subplots(figsize=(11,11))
        rect = plt.Rectangle((-5,-5), 10, 10, color='black', fill=False)
        circle1 = plt.Circle((-3,-3), 1, color='blue')
        circle2 = plt.Circle((-3,3), 1, color='blue')
        rect1 = plt.Rectangle((-4.75,-0.75), 1.5, 1.5, color='blue')
        rect2 = plt.Rectangle((-1.25,-0.75), 2.5, 1.5, color='blue')
        rect3 = plt.Rectangle((2.25,-3), 1.5, 2, color='blue')
        start = plt.Circle((self.startI,self.startJ), 0.105, color='green')
        end = plt.Circle((self.endI, self.endJ), 0.105, color='red')
        ax.add_patch(circle1)
        ax.add_patch(circle2)
        ax.add_patch(rect1)
        ax.add_patch(rect2)
        ax.add_patch(rect3)
        ax.add_patch(rect)
        ax.add_patch(start)
        ax.add_patch(end)
        plt.xlim([-5.5, 5.5])
        plt.ylim([-5.5, 5.5])
        return fig
        
    def getRoundedNumber(self, i):
        i = 50*i
        i = int(i)
        i = i/50
        return i

    def cost(self, Xi,Yi,Thetai,UL,UR):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180
        # Xi, Yi,Thetai: Input point's coordinates
        # Xs, Ys: Start point coordinates for plot function
        # Xn, Yn, Thetan: End point coordintes
        D=0
        while t<1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        Thetan = 180 * (Thetan) / 3.14
        Xn = self.getRoundedNumber(Xn)
        Yn = self.getRoundedNumber(Yn)
        return Xn, Yn, Thetan, D

    def plot_curve(self, node, UL, UR, color):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.1
        Xn=node.i
        Yn=node.j
        Thetan = 3.14 * node.theta / 180
        # Xi, Yi,Thetai: Input point's coordinates
        # Xs, Ys: Start point coordinates for plot function
        # Xn, Yn, Thetan: End point coordintes
        ax = plt.gca()
        while t<1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            plt.plot([Xs, Xn], [Ys, Yn], color=color)
        Thetan = 180 * (Thetan) / 3.14
        return Xn, Yn, Thetan

    def isInTargetArea(self, i, j):
        """
        Description: Checks if the currentnode is in target area to terminal the program
        Input: Current Node co-ordinates
        Output: Boolean
        """
        if (i - self.endI) ** 2 + (j - self.endJ) ** 2 - 0.01 <= 0:
            return True
        else:
            return False

    def backTrack(self, child):
        """
        Description: Backtracking from the finishing node to the start node.
        Input: Ending Node
        Output: A animation of the path generated.
        """
        while child != None:
            path.append(child)
            print(child.i, child.j, "Path")
            child = child.parent
        return True

    def performAStar(self, start):
        """
        Description: Defining initial constants - Visited array, Rows, Cols, Target String.
        Input: Starting and ending node for the robot to browse.
        Output: Returns True or False to define if an optimal path can be found or not.
        """
        print("Finding path...")
        priorityQueue = []
        visited_list = {}
        heapq.heappush(priorityQueue, (start.cost, start))
        while len(priorityQueue):
            currentNode = heapq.heappop(priorityQueue)
            currentNode = currentNode[1]
            if self.isInTargetArea(currentNode.i, currentNode.j):
                print("Found a path!")
                return True

            if tuple([currentNode.i, currentNode.j]) in visited_list:
                continue
            visited_list[tuple([currentNode.i, currentNode.j])] = True

            currentDistance = currentNode.costToCome
            neighbours, valid_actions = self.getNeighbours(currentNode)
            currentNode.neighbours = neighbours
            currentNode.valid_actions = valid_actions
            for neighbourNode, newDistance in neighbours.items():
                neighbourNode.costToCome = currentDistance + newDistance
                neighbourNode.cost = neighbourNode.costToCome + neighbourNode.costToGo
                neighbourNode.parent = currentNode
                heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNode))
                #print((neighbourNode.i, neighbourNode.j))
        print("Cannot find a path :(")
        return False

    def visualizeAStar(self, start):
        """
        Description: Visualization of the algorithm.
        Input: Starting and ending node for the robot to browse.
        Output: A animation of nodes which are browsed and the path generated.
        """
        fig = self.generate_map()
        visited_list = {}
        priorityQueue = []
        heapq.heappush(priorityQueue, (start.cost, start))
        while len(priorityQueue):

            currentNode = heapq.heappop(priorityQueue)
            currentNode = currentNode[1]

            if self.isInTargetArea(currentNode.i, currentNode.j):
                self.backTrack(currentNode)
                print("Distance Required to reach from start to end is:", currentNode.costToCome)
                path.reverse()
                for index in range(len(path)-1):
                    node = path[index]
                    action = node.valid_actions[path[index+1]]
                    self.plot_curve(node, action[0], action[1], "green")
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    fig.savefig("test.png")
                    plt.pause(0.0001)
                    plt.show(block=False)
                return path

            if tuple([currentNode.i, currentNode.j]) in visited_list:
                continue
            visited_list[tuple([currentNode.i, currentNode.j])] = True

            for neighbourNode, action in currentNode.valid_actions.items():
                self.plot_curve(currentNode,action[0],action[1],"purple")

            for neighbourNode, newDistance in currentNode.neighbours.items():
                heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNode))
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.0001)
            plt.show(block=False)
        return


    def isAnObstacle(self, x, y):
        """
        Description: Checks if the point (x,y) is inside an obstacle or not.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        return self.inside_rect1(x,y) or self.inside_rect2(x,y) or self.inside_rect3(x,y) or self.inside_circle1(x,y) or self.inside_circle2(x,y)

    def isOutsideArena(self, x, y):
        """
        Description: Checks if the point (x,y) is outside the areana or not.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        return  x < -5 + self.CLR or y < -5 + self.CLR or x > 5 - self.CLR or y > 5 - self.CLR

    def check_valid(self):
        if self.isAnObstacle(self.startI, self.startJ) or self.isOutsideArena(self.startI, self.startJ):
            print("start position is not valid")
            return True
        elif self.isAnObstacle(self.endI, self.endJ) or self.isOutsideArena(self.endI, self.endJ):
            print("goal position is not valid")
            return True
        elif self.CLR < 0.105: 
            print("clearence is not valid")
            return True
        else:
            return False

class Options:
    # required arguments
    start               = None      # [meters, meters, degrees]
    goal                = None      # [meters, meters]
    rpm                 = None      # [rot/min, rot/min]
    clearance           = None      # meters
    # fixed constants (taken from turtlebot3 URDF)
    radius              = 0.105   # meters
    wheel_radius        = 0.033     # meters
    wheel_separation    = 0.16      # meters (estimate)
    timestep            = 3.0/60    # minutes
    def __init__(self, start, goal, rpm, clearance):
        if not len(start) == 3:
            raise RuntimeError("Invalid start node: {}".format(start))
        if not len(goal) == 2:
            raise RuntimeError("Invalid start node: {}".format(start))
        if not len(rpm) == 2:
            raise RuntimeError("Invalid start node: {}".format(start))
        self.start = start
        self.goal = goal
        self.rpm = rpm
        self.clearance = clearance

def parse_args():
    parser = argparse.ArgumentParser(description="Solve for an optimal path via A*.") 
    parser.add_argument("-s", "--start", 
        default=DEFAULT_START, nargs='+', type=float, help="Starting node indices.")
    parser.add_argument("-g", "--goal", 
        default=DEFAULT_GOAL, nargs='+', type=float, help="Goal node indices.")
    parser.add_argument("-r", "--rpm", 
        default=DEFAULT_RPM, nargs='+', type=float, help="Input RPM")
    parser.add_argument("-c", "--clearance", 
        default=DEFAULT_CLEARANCE, type=float, help="Obstacle avoidance clearance.")

    args,_ = parser.parse_known_args()

    # convert input Yaw to degrees
    args.start[2]*=180/3.14159

    options = Options(args.start, args.goal, args.rpm, args.clearance)
    return options

def generate_trajectory(path, r, L):
    """Convert a given path to a fixed-timestep trajectory.
    """
    trajectory = []
    for index in range(len(path)-1):
        node = path[index]
        action = node.valid_actions[path[index+1]]
        # skip nodes without parent action (e.g. first)
        if action is None:
            continue 
        msg = Twist()
        # compute twist message from given actions (convert to m/s, rad/s)
        msg.linear.x = r*(action[0]+action[1])/120.0 
        msg.angular.z = r*(action[1]-action[0])/(60.0*L)
        trajectory.append(msg)
    return trajectory

def main():
    # get input arguments
    options = parse_args()

    # initialize as ROS node
    rospy.init_node('planner', anonymous=True)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sleep_time = options.timestep*60.0

    start = Node(options.start[0], options.start[1], options.goal[0], options.goal[1], options.start[2])
    end = Node(options.goal[0], options.goal[1], options.goal[0], options.goal[1], 0)
    map = graph(start, end, options.rpm[0], options.rpm[1], options.clearance)
    if map.check_valid():
        raise RuntimeError("Invalid input")
    success = map.performAStar(start)
    if success:
        path = map.visualizeAStar(start)
        traj = generate_trajectory(path, radius, L)
        print("Publishing desired velocity commands to /cmd_vel")
        for msg in traj:
            publisher.publish(msg)
            rospy.sleep(sleep_time)
        # publish stop message at the end
        publisher.publish(Twist())
        print("Finished commanded velocities.")
        rospy.spin()
    else:
        raise RuntimeError("Error")


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# -*-coding:utf-8 -*-

import sys
import argparse
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_plan.map import FinalMap
from turtlebot3_plan.node import Node, ActionSet
from turtlebot3_plan.graph import Graph
from turtlebot3_plan.search import AStar
from turtlebot3_plan.options import Options
from turtlebot3_plan.visualize import ExplorationVisualizer, plot_path

# default input args
DEFAULT_START = [-4, -4, 0]
DEFAULT_GOAL = [4, 4]
DEFAULT_RPM = [100, 200]
DEFAULT_CLEARANCE = 0.01

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
    parser.add_argument("-v", "--visualize", action="store_true",
        help="Generate a video of the path planning process.")

    args,_ = parser.parse_known_args()

    # convert input Yaw to degrees
    args.start[2]*=180/3.14159

    options = Options(args.start, args.goal, args.rpm, args.clearance, args.visualize)
    return options

def generate_trajectory(path, r, L):
    """Convert a given path to a fixed-timestep trajectory.
    """
    trajectory = []
    for node in path:
        action = node.parent_action

        # skip nodes without parent action (e.g. first)
        if action is None:
            continue
        
        msg = Twist()

        # compute twist message from given actions (convert to m/s, rad/s)
        msg.linear.x = r*(action[0]+action[1])/120.0 
        msg.angular.z = r*(action[1]-action[0])/(60.0*L)
        trajectory.append(msg)
    return trajectory

if __name__ == "__main__":
    # get input arguments
    options = parse_args()

    # initialize as ROS node
    rospy.init_node('planner', anonymous=True)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sleep_time = options.timestep*60.0

    # generate obstacle map
    obstacle_map = FinalMap()

    # Set node class variables based on inputs
    action_set = ActionSet(
            options.rpm,
            options.wheel_radius,
            options.wheel_separation,
            options.timestep)
    resolution = action_set.calc_resolution(options.start[2])
    print("Using graph resolution {}".format(resolution))
    Node.set_actionset(action_set)
    Node.set_resolution(resolution)
    Node.set_hash_offset(obstacle_map.size()+[0])

    # create start and goal nodes
    buffer_ = options.radius + options.clearance
    if not obstacle_map.is_valid(options.start, buffer_):
        raise RuntimeError("Invalid start node: {}".format(options.start))
    if not obstacle_map.is_valid(options.goal+[0], buffer_):
        raise RuntimeError("Invalid goal node: {}".format(options.goal))
    start_node = Node(options.start)
    goal_node = Node(options.goal+[0])

    print("Start node: {}".format(start_node))
    print("Goal  node: {}".format(goal_node))

    # generate graph
    print("Generating graph...")
    graph = Graph(obstacle_map, start_node, buffer_=buffer_)

    # perform search    
    print("Performing A* search...")
    d = AStar(graph, start_node)
    if not d.solve(goal_node, goal_tolerance=10*resolution[0]):
        print("Failed to find a path to the goal node.")
        sys.exit(1)

    # get path to goal node
    optimal_path,_ = d.get_path()

    # visualize optimal path (and make video of exploration)
    if options.visualize:
        visualizer = ExplorationVisualizer(
                obstacle_map, 
                optimal_path,
                *d.get_exploration(True, goal_node)
        ) 
        visualizer.plot(True)
    else:
        plot_path(obstacle_map, optimal_path, True)

    # control to path
    traj = generate_trajectory(optimal_path, options.wheel_radius, options.wheel_separation)

    # control to plan (open loop)
    print("Publishing desired velocity commands to /cmd_vel")
    for msg in traj:
        publisher.publish(msg)
        rospy.sleep(sleep_time)
    
    # publish stop message at the end
    publisher.publish(Twist())
    print("Finished commanded velocities.")

    rospy.spin()



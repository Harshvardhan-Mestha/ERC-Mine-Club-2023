#!/usr/bin/env python3

#using isValidPoint() from obstacle_detection

## Can be heavily optimize by simply changing generate_random_point function
# simply making it such that it takes the two points of goals, and just makes random points
# within those will make it so much faster
#
# also optimizing the point distance and using same path of entering for exit things like that could make it much faster
#
# also as it decides some points, we can start publishing those to /planned_path instead of waiting for entire thing as doing currently
#

import obstacle_detection as obsdet
from shapely.geometry import LineString, Point, Polygon

import matplotlib.pyplot as plt

import rospy
import random
import math

import std_msgs
from geometry_msgs.msg import Point

from robotics_hackathon_automation.msg import PointArray 

from std_msgs.msg import String

# from std_msgs import String

# import robotics_hackathon_automation
# from ERC_hackathon_2023.robotics_hackathon_automation.msg import PointArray


class Wall:
    def __init__(self, centerX, centerY, length, width):
        self.centerX = centerX
        self.centerY = centerY
        self.width = width + 0.2
        self.length = length+0.2
        Ax = (self.centerX+(self.length/2))
        Ay = (self.centerY+(self.width/2))
        Bx = (self.centerX+(self.length/2))
        By = (self.centerY-(self.width/2))
        Cx = (self.centerX-(self.length/2))
        Cy = (self.centerY-(self.width/2))
        Dx = (self.centerX-(self.length/2))
        Dy = (self.centerY+(self.width/2))
        self.polygon = Polygon(
            [(Ax, Ay), (Bx, By), (Cx, Cy), (Dx, Dy), (Ax, Ay)])
        self.corners = [(Ax, Ay), (Bx, By), (Cx, Cy), (Dx, Dy)]


maze = [Wall(-5.191, 0.9886, 1, 0.15), Wall(-5.639, -0.8309, 0.15, 3.769200), Wall(-5.672, 1.785, 0.15, 1.597130), Wall(-4.957, 2.543, 1.597130, 0.15), Wall(-4.277, 2.007956, 0.15, 1.169920), Wall(-0.0037, 2.51, 8.729630, 0.15), Wall(-1.588, 1.8136, 0.15, 1.25), Wall(-1.588, 0.0886, 0.15, 2.5), Wall(-2.138, 1.26, 1.25, 0.15), Wall(-2.668, 0.7136, 0.15, 1.25), Wall(-3.488, 0.16, 1.75, 0.15), Wall(2.405, 0.656, 0.75, 0.15), Wall(2.705, 0.956, 0.15, 0.75), Wall(3.2522, 1.2566, 1.25, 0.15), Wall(3.80526, 0.2066, 0.15, 2.25), Wall(3.3802, -0.844, 1, 0.15), Wall(2.955, -0.5433, 0.15, 0.75), Wall(2.7802, -0.2433, 0.5, 0.15), Wall(2.605, -0.5433, 0.15, 0.75), Wall(4.301, 2.189, 0.15, 0.810003), Wall(4.975, 2.5196, 1.50, 0.15), Wall(5.711,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              1.998, 0.15, 1.192330), Wall(5.306, 1.463, 0.919672, 0.15), Wall(5.698, 0.301, 0.15, 2.276490), Wall(5.185, -0.885, 1.119670, 0.15), Wall(4.7, -1.296, 0.15, 0.982963), Wall(5.67, -1.7033, 0.15, 1.75), Wall(5.154, -2.521, 1.185380, 0.15), Wall(0.673, -2.534, 7.883080, 0.15), Wall(1.906, -1.93, 0.15, 1.206910), Wall(0.877, -1.7, 0.15, 1.719980), Wall(0.2502, -0.917, 1.50, 0.15), Wall(-0.433, -1.389, 0.15, 1.072), Wall(-0.4292, -0.4799, 0.15, 0.927565), Wall(0.9177, 0.2156, 0.15, 2.416050), Wall(0.23527, 1.3486, 1.5, 0.15), Wall(-0.439, 1.048, 0.15, 0.75), Wall(-3.2627, -1.72, 0.15, 1.75), Wall(-3.883, -0.9203, 1.414750, 0.15), Wall(-3.9377, -2.52, 1.5, 0.15), Wall(-4.615, -2.157, 0.15, 0.870384), Wall(2.105, 1.58, 0.15, 2.15893)]


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def visualize_all_paths(paths, point_list):
    plt.figure()
    
    for i in range(len(paths)):
        path = paths[i]
        start_x, start_y = point_list[i]
        goal_x, goal_y = point_list[i + 1]
        
        if path:
            for j in range(1, len(path)):
                plt.plot([path[j-1][0], path[j][0]], [path[j-1][1], path[j][1]], 'b-')
            plt.plot(start_x, start_y, 'go', label=f'Start {i + 1}')
            plt.plot(goal_x, goal_y, 'ro', label=f'Goal {i + 2}')
            plt.plot(*zip(*path), 'g-', linewidth=2, label=f'Path {i + 1}-{i + 2}')
        else:
            plt.plot(start_x, start_y, 'go')
            plt.plot(goal_x, goal_y, 'ro')
    
    plt.legend()

    plt.xlim(-6, 6)  # Adjust as needed
    plt.ylim(-4, 4)
    plt.xlabel('X')
    plt.ylabel('Y')
    display_maze = [Wall(-5.191, 0.9886, 1, 0.15), Wall(-5.639, -0.8309, 0.15, 3.769200), Wall(-5.672, 1.785, 0.15, 1.597130), Wall(-4.957, 2.543, 1.597130, 0.15), Wall(-4.277, 2.007956, 0.15, 1.169920), Wall(-0.0037, 2.51, 8.729630, 0.15), Wall(-1.588, 1.8136, 0.15, 1.25), Wall(-1.588, 0.0886, 0.15, 2.5), Wall(-2.138, 1.26, 1.25, 0.15), Wall(-2.668, 0.7136, 0.15, 1.25), Wall(-3.488, 0.16, 1.75, 0.15), Wall(2.405, 0.656, 0.75, 0.15), Wall(2.705, 0.956, 0.15, 0.75), Wall(3.2522, 1.2566, 1.25, 0.15), Wall(3.80526, 0.2066, 0.15, 2.25), Wall(3.3802, -0.844, 1, 0.15), Wall(2.955, -0.5433, 0.15, 0.75), Wall(2.7802, -0.2433, 0.5, 0.15), Wall(2.605, -0.5433, 0.15, 0.75), Wall(4.301, 2.189, 0.15, 0.810003), Wall(4.975, 2.5196, 1.50, 0.15), Wall(5.711, 1.998, 0.15, 1.192330), Wall(5.306, 1.463, 0.919672, 0.15), Wall(5.698, 0.301, 0.15, 2.276490), Wall(5.185, -0.885, 1.119670, 0.15), Wall(4.7, -1.296, 0.15, 0.982963), Wall(5.67, -1.7033, 0.15, 1.75), Wall(5.154, -2.521, 1.185380, 0.15), Wall(0.673, -2.534, 7.883080, 0.15), Wall(1.906, -1.93, 0.15, 1.206910), Wall(0.877, -1.7, 0.15, 1.719980), Wall(0.2502, -0.917, 1.50, 0.15), Wall(-0.433, -1.389, 0.15, 1.072), Wall(-0.4292, -0.4799, 0.15, 0.927565), Wall(0.9177, 0.2156, 0.15, 2.416050), Wall(0.23527, 1.3486, 1.5, 0.15), Wall(-0.439, 1.048, 0.15, 0.75), Wall(-3.2627, -1.72, 0.15, 1.75), Wall(-3.883, -0.9203, 1.414750, 0.15), Wall(-3.9377, -2.52, 1.5, 0.15), Wall(-4.615, -2.157, 0.15, 0.870384), Wall(2.105, 1.58, 0.15, 2.15893)]
    for wall in display_maze:
        x, y = wall.polygon.exterior.xy
        plt.plot(x, y)


    plt.title('RRT Path Planning')
    plt.show()

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def generate_random_point():
    return random.uniform(-5.4,6), random.uniform(-2.7, 3)  # Adjust range as needed

def find_nearest_node(nodes, x, y):
    nearest_node = None
    min_distance = float('inf')
    for node in nodes:
        distance = euclidean_distance(node.x, node.y, x, y)
        if distance < min_distance:
            nearest_node = node
            min_distance = distance
    return nearest_node


def build_rrt(start_x, start_y, goal_x, goal_y, iterations):
    start_node = Node(start_x, start_y)
    nodes = [start_node]

    for _ in range(iterations):
        random_x, random_y = generate_random_point()
        nearest_node = find_nearest_node(nodes, random_x, random_y)
        
        new_x = nearest_node.x + (random_x - nearest_node.x) * 0.1  # Adjust step size
        new_y = nearest_node.y + (random_y - nearest_node.y) * 0.1

        if obsdet.isValidPoint(nearest_node.x, nearest_node.y, new_x, new_y):
            new_node = Node(new_x, new_y)
            new_node.parent = nearest_node
            nodes.append(new_node)
            
            if euclidean_distance(new_x, new_y, goal_x, goal_y) > 0.2 and euclidean_distance(new_x, new_y, goal_x, goal_y) < 0.5:  # Adjust goal threshold
                goal_node = Node(goal_x, goal_y)
                goal_node.parent = new_node
                nodes.append(goal_node)
                return nodes
    
    return None  # No path found

def extract_path(goal_node):
    path = []
    current = goal_node
    while current is not None:
        path.insert(0, (current.x, current.y))
        current = current.parent
    return path

def plan_path_for_points(point_list, iterations_per_point):
    paths = []

    for i in range(len(point_list) - 1):
        start_x, start_y = point_list[i]
        goal_x, goal_y = point_list[i + 1]

        nodes = build_rrt(start_x, start_y, goal_x, goal_y, iterations_per_point)
        if nodes:
            path = extract_path(nodes[-1])
            paths.append(path)
        else:
            paths.append(None)
    
    return paths

# List of points 
point_list = [
    [-5.06, -3.12],
    [-3.61, -2.20],
    # [-4.61,-1.29], #d2
    # [-5.24,-0.26], # dummy point
    [-2.28, 1.86],
    [0.57, 0.33],
    [1.58, -2.26],    
    [5.18, -2.19]

]

iterations_per_point = 100000
planned_paths = plan_path_for_points(point_list, iterations_per_point)


rospy.init_node('path_planner', anonymous=True)
# path_pub = rospy.Publisher('/planned_path', PointArray, queue_size=10)  # Replace 'point_array_topic' with your desired topic name

array_pub = rospy.Publisher('/planned_path', String, queue_size=10)  # Replace 'point_array_topic' with your desired topic name


# for i, path in enumerate(planned_paths):
#     if path:
#         print(f"Path from point {i + 1} to point {i + 2}: {path}")
# # Inside the loop
#         # Inside the loop
#         visualize_path(build_rrt(point_list[i][0], point_list[i][1], point_list[i + 1][0], point_list[i + 1][1], iterations_per_point), path, point_list[i][0], point_list[i][1], point_list[i + 1][0], point_list[i + 1][1])

#     else:
#         print(f"No path found from point {i + 1} to point {i + 2}")

def convert_to_custom_schema(points_list):
    schema_list = [{'x': x, 'y': y} for x, y in points_list]
    return schema_list

# Create a list to store sequences of points
all_paths = []

for i in range(len(point_list) - 1):
    start_x, start_y = point_list[i]
    goal_x, goal_y = point_list[i + 1]

    nodes = build_rrt(start_x, start_y, goal_x, goal_y, iterations_per_point)
    path = extract_path(nodes[-1]) if nodes else None

    if path:
        print(f"Path from point {i + 1} to point {i + 2}: {path}")
        custom_schema_path = [Point(x=x, y=y) for x, y in path]
        all_paths.append(custom_schema_path)  # Add the sequence of points

    else:
        print(f"No path found from point {i + 1} to point {i + 2}")

# Convert the array of arrays to a single string
array_string = str(all_paths)

# Save the string to a text file
with open("paths_data_temp.txt", "w") as f:
    f.write(array_string)


with open("paths_data_temp.txt", "r") as f:
    array_string = f.read()


# Publish the array string as a ROS message
from time import sleep

sleep(5) ## have to do this as workaround, to give enough time to publish, although showing map should work

array_pub.publish(array_string)

planned_paths = plan_path_for_points(point_list, iterations_per_point)
visualize_all_paths(planned_paths, point_list)
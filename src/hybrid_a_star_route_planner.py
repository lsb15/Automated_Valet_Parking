import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import math
from pose import Pose
from free_space_map import FreeSpaceMap

# Define a class for representing a node in the search tree
class Node:
    def __init__(self, pose, cost, steering, parent_node_index):
        self.pose = pose  # Pose of the node (position and orientation)
        self.discrete_x = round(pose.x) # Discrete x-coordinate
        self.discrete_y = round(pose.y) # Discrete y-coordinate
        self.cost = cost # Cost to reach this node
        self.steering = steering # Steering angle
        self.parent_node_index = parent_node_index # Index of the parent node

# Define a class for the hybrid A* route planner
class HybridAStarRoutePlanner:
    def __init__(self):
        self.free_space_map: FreeSpaceMap = None # Instance of FreeSpaceMap
        self.wheelbase = 2.7  # Wheelbase of the vehicle
        steering_degree_inputs = [-40, -20, -10, 0, 10, 20, 40] # Steering angles in degrees
        self.steering_inputs = [math.radians(x) for x in steering_degree_inputs] # Convert steering angles to radians
        self.chord_lengths = [1, 2] # Chord lengths for different velocities
        self.goal_node = None # Goal node for route planning

    # Method to search for a route using hybrid A*
    def search_route(self, free_space_map: FreeSpaceMap, show_process=False):
        self.free_space_map = free_space_map
        start_pose = self.free_space_map.get_drop_off_spot() # Get start pose from the map
        goal_pose = self.free_space_map.get_goal_state() # Get goal pose from the map
        print(f"Start Hybrid A Star Route Planner (start {start_pose.x, start_pose.y}, end {goal_pose.x, goal_pose.y})")
        start_node = Node(start_pose, 0, 0, -1)  # Create start node
        self.goal_node = Node(goal_pose, 0, 0, -1)  # Create goal node
        open_set = {self.free_space_map.get_grid_index(start_node.discrete_x, start_node.discrete_y): start_node} # Initialize open set
        closed_set = {}  # Initialize closed set

        while open_set:
            current_node_index = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calculate_heuristic_cost(open_set[o]),
            )
            current_node = open_set[current_node_index]

            if show_process:
                self.free_space_map.plot_map()
                self.plot_process(open_set, closed_set, current_node)

            if self.calculate_distance_to_end(current_node.pose) < 0.5:
                print("Find Goal")
                self.goal_node = current_node
                rx, ry = self.process_route(closed_set)
                self.free_space_map.plot_route(rx, ry)
                return rx, ry

            del open_set[current_node_index]
            closed_set[current_node_index] = current_node

            next_nodes = [
                self.calculate_next_node(
                    current_node, current_node_index, velocity, steering
                )
                for steering in self.steering_inputs
                for velocity in self.chord_lengths
            ]
            for next_node in next_nodes:
                if self.free_space_map.is_not_on_obstacle((next_node.discrete_x, next_node.discrete_y)):
                    next_node_index = self.free_space_map.get_grid_index(next_node.discrete_x, next_node.discrete_y)
                    if next_node_index in closed_set:
                        continue
                    if next_node_index not in open_set:
                        open_set[next_node_index] = next_node
                    else:
                        if open_set[next_node_index].cost > next_node.cost:
                            open_set[next_node_index] = next_node
        print("Cannot find Route")
        return [], []

     # Method to process the route
    def process_route(self, closed_set):
        rx = [self.goal_node.pose.x]
        ry = [self.goal_node.pose.y]
        parent_node = self.goal_node.parent_node_index
        while parent_node != -1:
            n = closed_set[parent_node]
            rx.append(n.pose.x)
            ry.append(n.pose.y)
            parent_node = n.parent_node_index
        return rx, ry

    # Method to calculate the next node in the search tree
    def calculate_next_node(self, current, current_node_index, chord_length, steering):
        theta = self.change_radians_range(
            current.pose.theta + chord_length * math.tan(steering) / float(self.wheelbase)
        )
        x = current.pose.x + chord_length * math.cos(theta)
        y = current.pose.y + chord_length * math.sin(theta)

        return Node(
            Pose(x, y, theta),
            current.cost + chord_length,
            steering,
            current_node_index,
        )

    # Method to calculate the heuristic cost
    def calculate_heuristic_cost(self, node):
        distance_cost = self.calculate_distance_to_end(node.pose)
        angle_cost = abs(self.change_radians_range(node.pose.theta - self.goal_node.pose.theta)) * 0.1
        steering_cost = abs(node.steering) * 10

        cost = distance_cost + angle_cost + steering_cost
        return float(cost)

    # Method to calculate the distance to the goal
    def calculate_distance_to_end(self, pose):
        distance = math.sqrt(
            (pose.x - self.goal_node.pose.x) ** 2 + (pose.y - self.goal_node.pose.y) ** 2
        )
        return distance

    # Method to handle the range of radians (-pi to pi)
    @staticmethod
    def change_radians_range(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # Method to plot the process of searching for the route
    def plot_process(self, open_set, closed_set, current_node):
        self.free_space_map.plot_map()

        for node_index in closed_set:
            node = closed_set[node_index]
            plt.plot(self.free_space_map._m_to_px(node.pose.x), self.free_space_map._m_to_px(node.pose.y), "or", markersize=4)

        for node_index in open_set:
            node = open_set[node_index]
            plt.plot(self.free_space_map._m_to_px(node.pose.x), self.free_space_map._m_to_px(node.pose.y), "og", markersize=4)

        plt.plot(self.free_space_map._m_to_px(current_node.pose.x), self.free_space_map._m_to_px(current_node.pose.y), "xc", markersize=6)

        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )
        plt.pause(0.001)

def main():
    free_space_map = FreeSpaceMap("modified_map.pgm")
    free_space_map.set_drop_off_spot(830, 1904, 60)
    free_space_map.set_goal_state(2342, 5956, 90)
    free_space_map.plot_map()

    hybrid_a_star_route_planner = HybridAStarRoutePlanner()
    hybrid_a_star_route_planner.search_route(free_space_map, show_process=False)

    # Annotate start and goal points
    start_pose = free_space_map.get_drop_off_spot()
    goal_pose = free_space_map.get_goal_state()
    plt.text(
        free_space_map._m_to_px(start_pose.x) + 1,
        free_space_map._m_to_px(start_pose.y) + 1,
        "Start",
        fontsize=10,
        ha="center",
        va="bottom",
        color="blue",
    )
    plt.text(
        free_space_map._m_to_px(goal_pose.x) + 1,
        free_space_map._m_to_px(goal_pose.y) + 1,
        "Goal",
        fontsize=10,
        ha="center",
        va="bottom",
        color="purple",
    )

    free_space_map.plot_show()

if __name__ == "__main__":
    main()

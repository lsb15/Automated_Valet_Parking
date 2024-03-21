import os

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np

from pose import Pose # Importing Pose class from pose module

# Define a class called FreeSpaceMap
class FreeSpaceMap:
    def __init__(self, map_file_name):
        # Initialize parameters for the map
        self.free_space = 254
        self.resolution = 0.05  # Resolution of the map in meters per pixel
        self.resolution_inverse = int(1 / self.resolution) # Inverse of resolution for conversion
        
        # Define sizes of parking space and vehicle in pixels
        self.parking_width = self._m_to_px(3)
        self.parking_length = self._m_to_px(6)
        self.vehicle_width = self._m_to_px(2)
        self.vehicle_front_length = self._m_to_px(4)
        self.vehicle_rear_length = self._m_to_px(1)
        self.vehicle_length = self.vehicle_front_length + self.vehicle_rear_length
        
        # Read the map image file
        map_file_path = os.path.join(os.path.dirname(__file__), map_file_name)
        with open(map_file_path, "rb") as map_image:
            self.map = plt.imread(map_image) # Read map image

        # Calculate dimensions of the map in meters
        self.lot_width = self._px_to_m(self.map.shape[1])
        self.lot_height = self._px_to_m(self.map.shape[0])

        # Define variables for drop off spot, goal state, and parking space
        self.drop_off_spot = ()
        self.goal_state = ()
        self.parking_space = ()

        fig = plt.figure()
        self.ax = fig.add_subplot(111)

    # Method to get grid index from given x and y coordinates
    def get_grid_index(self, x, y):
        return x + y * self.lot_width

    # Method to check if a given node is not on an obstacle
    def is_not_on_obstacle(self, current_node):
        is_in = 0 < current_node[0] < self.lot_width and 0 < current_node[1] < self.lot_height
        return is_in and not self._is_on_obstacle(current_node)

     # Method to check if a given node is on an obstacle
    def _is_on_obstacle(self, current_node):
        for x in range(self._m_to_px(current_node[0]), self._m_to_px(current_node[0] + 1)):
            for y in range(self._m_to_px(current_node[1]), self._m_to_px(current_node[1] + 1)):
                if self.map[y, x] != self.free_space:
                    return True

    # Method to set the drop off spot with x, y, and theta (orientation)
    def set_drop_off_spot(self, x, y, theta):
        self.drop_off_spot = (x, y, theta)

    # Method to get the drop off spot
    def get_drop_off_spot(self):
        return Pose(
            self._px_to_m(self.drop_off_spot[0]),
            self._px_to_m(self.drop_off_spot[1]),
            np.radians(self.drop_off_spot[2])
        )

    # Method to set the goal state with x, y, and theta (orientation)
    def set_goal_state(self, x, y, theta):
        self.goal_state = (x, y, theta)

    # Method to set the goal state with a Pose object
    def set_goal_state_pose(self, pose):
        self.goal_state = (self._m_to_px(pose.x), self._m_to_px(pose.y), np.degrees(pose.theta))

    # Method to get the goal state
    def get_goal_state(self):
        return Pose(
            self._px_to_m(self.goal_state[0]),
            self._px_to_m(self.goal_state[1]),
            np.radians(self.goal_state[2])
        )

    # Method to set the parking space with x, y, and theta (orientation)
    def set_parking_space(self, x, y, theta):
        self.parking_space = (x, y, theta)

    # Method to get the parking space
    def get_parking_space(self):
        return Pose(
            self._px_to_m(self.parking_space[0]),
            self._px_to_m(self.parking_space[1]),
            np.radians(self.parking_space[2])
        )

    # Method to plot the map
    def plot_map(self):
        plt.imshow(self.map, cmap="gray") # Display the map image

         # Draw drop off spot, goal state, and parking space if defined
        if self.drop_off_spot:
            self._draw_vehicle_pose(self.drop_off_spot, "blue")

        if self.goal_state:
            self._draw_vehicle_pose(self.goal_state, "yellow")

        if self.parking_space:
            self._draw_parking_space(self.parking_space, "red")

        # Set the plot limits based on defined spots
        if self.drop_off_spot and self.goal_state:
            self.ax.set(
                xlim=[
                    min(self.drop_off_spot[0], self.goal_state[0]) - 800,
                    max(self.drop_off_spot[0], self.goal_state[0]) + 800
                ],
                ylim=[
                    min(self.drop_off_spot[1], self.goal_state[1]) - 1000,
                    max(self.drop_off_spot[1], self.goal_state[1]) + 1000
                ]
            )
        else:
            self.ax.set(
                xlim=[0, self.map.shape[1]], ylim=[0, self.map.shape[0]]
            )

     # Method to plot the current process
    def plot_process(self, current_node):
        # Plot the current position
        plt.plot(self._m_to_px(current_node.discrete_x), self._m_to_px(current_node.discrete_y), "xc")
        # Connect to the escape key to stop the simulation
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )

     # Method to plot the route
    def plot_route(self, rx, ry):
        plt.plot([self._m_to_px(x) for x in rx], [self._m_to_px(y) for y in ry], "-r")
        plt.pause(0.001)

    # Method to show the plot
    def plot_show(self):
        if self.drop_off_spot:
            self._draw_vehicle_pose(self.drop_off_spot, "blue")

        if self.goal_state:
            self._draw_vehicle_pose(self.goal_state, "yellow")

        if self.parking_space:
            self._draw_parking_space(self.parking_space, "red")

        plt.show()

    # Method to draw the parking space
    def _draw_parking_space(self, pose, color):
        rotation = np.radians(pose[2] - 90)
        x = pose[0] - self.parking_width / 2 * np.cos(rotation) + self.vehicle_rear_length * np.sin(rotation)
        y = pose[1] - self.parking_width / 2 * np.sin(rotation) - self.vehicle_rear_length * np.cos(rotation)
        parking_space = patches.Rectangle((x, y), self.parking_width, self.parking_length, color=color, alpha=0.50)
        t = transforms.Affine2D().rotate_deg_around(
            pose[0], pose[1], (pose[2] - 90)
        ) + self.ax.transData
        parking_space.set_transform(t)
        self.ax.add_patch(parking_space)

    # Method to draw the vehicle pose
    def _draw_vehicle_pose(self, pose, color):
        rotation = np.radians(pose[2])
        x = pose[0] - self.vehicle_rear_length * np.cos(rotation) + self.vehicle_width / 2 * np.sin(rotation)
        y = pose[1] - self.vehicle_rear_length / 2 * np.sin(rotation) - self.vehicle_width / 2 * np.cos(rotation)
        parking_space = patches.Rectangle((x, y), self.vehicle_length, self.vehicle_width, color=color, alpha=0.50)
        t = transforms.Affine2D().rotate_deg_around(
            pose[0], pose[1], pose[2]
        ) + self.ax.transData
        parking_space.set_transform(t)
        self.ax.add_patch(parking_space)

    # Method to convert meters to pixels
    def _m_to_px(self, m):
        return int(m * self.resolution_inverse)

    # Method to convert pixels to meters
    def _px_to_m(self, px):
        return px * self.resolution


if __name__ == "__main__":
    free_space_map = FreeSpaceMap("modified_map.pgm")
    free_space_map.set_drop_off_spot(830, 1904, 60)
    free_space_map.set_parking_space(3114, 5790, 85)
    free_space_map.plot_map()
    free_space_map.plot_show()

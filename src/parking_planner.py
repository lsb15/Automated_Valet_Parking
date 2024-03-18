import numpy as np
import matplotlib.pyplot as plt
from pose import Pose

class ParkingPlanner:
    def __init__(self):
        self.vehicle_width = 2
        self.wheel_base = 2.7
        self.front_overhang = 1.3
        self.rear_overhang = 1.0
        self.vehicle_front_length = self.wheel_base + self.front_overhang
        self.vehicle_length = self.vehicle_front_length + self.rear_overhang
        self.parking_space_width = 3
        self.parking_space_length = 6
        self.parking_space_front_length = 5
        self.steering_angle = np.radians(40)
        self.r = self.wheel_base / np.tan(self.steering_angle)
        self.x_tan = 0
        self.y_tan = 0
        self.x_mid = 0
        self.y_mid = 0
        self.x_turn = 0
        self.y_turn = 0
        self.theta_turn = 0
        self.r1_center = []
        self.r2_center = []
        self.r1_x = []
        self.r1_y = []
        self.r2_x = []
        self.r2_y = []

    def get_goal_pose(self):
        return Pose(self.x_turn, self.y_turn, self.theta_turn)

    def plan(self, start_pose: Pose, parking_pose: Pose):
        c_margin = 0.5
        d_m = np.sqrt(
            (self.r - self.vehicle_width / 2 - c_margin) ** 2 - (self.r - self.parking_space_width / 2) ** 2
        )
        d_t = self.parking_space_front_length - d_m
        c_path = 0.5
        theta = np.arcsin(
            (c_path + self.vehicle_width / 2 + self.parking_space_front_length + self.r - d_t) / (2 * self.r)
        )
        local_x_tan = 0
        local_y_tan = d_t
        local_x_mid = local_x_tan + self.r * (1 - np.cos(theta))
        local_y_mid = local_y_tan + self.r * np.sin(theta)
        local_x_turn = local_x_mid - self.r * np.sin(theta)
        local_y_turn = local_y_mid - self.r * (1 - np.cos(theta))
        local_r1_center = [self.r + local_x_tan, local_y_tan]
        local_r2_center = [local_x_turn, local_y_turn + self.r]

        # Rotate the local coordinates to the global coordinates
        self.theta_turn = parking_pose.theta - np.radians(90)
        dx = start_pose.x - parking_pose.x
        dy = start_pose.y - parking_pose.y
        is_right = dx * np.cos(-self.theta_turn) - dy * np.sin(-self.theta_turn) > 0
        if is_right:
            local_x_tan = -local_x_tan
            local_x_mid = -local_x_mid
            local_x_turn = -local_x_turn
            local_r1_center[0] = -local_r1_center[0]
            local_r2_center[0] = -local_r2_center[0]

        self.x_tan = parking_pose.x + local_x_tan * np.cos(self.theta_turn) - local_y_tan * np.sin(self.theta_turn)
        self.y_tan = parking_pose.y + local_x_tan * np.sin(self.theta_turn) + local_y_tan * np.cos(self.theta_turn)
        self.x_mid = parking_pose.x + local_x_mid * np.cos(self.theta_turn) - local_y_mid * np.sin(self.theta_turn)
        self.y_mid = parking_pose.y + local_x_mid * np.sin(self.theta_turn) + local_y_mid * np.cos(self.theta_turn)
        self.x_turn = parking_pose.x + local_x_turn * np.cos(self.theta_turn) - local_y_turn * np.sin(self.theta_turn)
        self.y_turn = parking_pose.y + local_x_turn * np.sin(self.theta_turn) + local_y_turn * np.cos(self.theta_turn)

        self._plan_circle_paths(parking_pose, local_r1_center, local_r2_center)

    def _plan_circle_paths(self, goal_pose, local_r1_center, local_r2_center):
        self.r1_center = [
            goal_pose.x + local_r1_center[0] * np.cos(self.theta_turn) - local_r1_center[1] * np.sin(self.theta_turn),
            goal_pose.y + local_r1_center[0] * np.sin(self.theta_turn) + local_r1_center[1] * np.cos(self.theta_turn)
        ]

        self.r2_center = [
            goal_pose.x + local_r2_center[0] * np.cos(self.theta_turn) - local_r2_center[1] * np.sin(self.theta_turn),
            goal_pose.y + local_r2_center[0] * np.sin(self.theta_turn) + local_r2_center[1] * np.cos(self.theta_turn)
        ]

        self.r1_x, self.r1_y = self._plan_circle_path(
            self.r1_center,
            (self.x_tan, self.y_tan),
            (self.x_mid, self.y_mid),
        )

        self.r2_x, self.r2_y = self._plan_circle_path(
            self.r2_center,
            (self.x_mid, self.y_mid),
            (self.x_turn, self.y_turn),
        )

    def _plan_circle_path(self, center, point1, point2):
        angle1 = self._calculate_angle(center, point1)
        angle2 = self._calculate_angle(center, point2)
        if abs(angle1 - angle2) > np.pi:
            if angle1 < angle2:
                angle1 += 2 * np.pi
            else:
                angle2 += 2 * np.pi
        # Generate points for the arc
        theta = np.linspace(angle1, angle2, 100)
        x = center[0] + self.r * np.cos(theta)
        y = center[1] + self.r * np.sin(theta)

        return x, y

    @staticmethod
    def _calculate_angle(center, point):
        angle = np.arctan2(point[1] - center[1], point[0] - center[0])
        if angle < 0:
            angle += 2 * np.pi
        return angle

def main():
    parking_planner = ParkingPlanner()
    start_pose = Pose(-6, 6, np.radians(0))
    goal_pose = Pose(0, 0, np.radians(90))
    parking_planner.plan(start_pose, goal_pose)

    # Plot start and goal poses
    plt.plot(start_pose.x, start_pose.y, "ro", label="Start")
    plt.plot(goal_pose.x, goal_pose.y, "bo", label="Goal")

    # Plot key points
    plt.plot(parking_planner.x_tan, parking_planner.y_tan, "ko", label="Tan Point")
    plt.plot(parking_planner.x_mid, parking_planner.y_mid, "ko", label="Mid Point")
    plt.plot(parking_planner.x_turn, parking_planner.y_turn, "ko", label="Turn Point")
    plt.plot(parking_planner.r1_center[0], parking_planner.r1_center[1], "ko", label="R1 Center")
    plt.plot(parking_planner.r2_center[0], parking_planner.r2_center[1], "ko", label="R2 Center")

    # Plot circular paths
    plt.plot(parking_planner.r1_x, parking_planner.r1_y, "k-", label="R1 Circular Path")
    plt.plot(parking_planner.r2_x, parking_planner.r2_y, "k-", label="R2 Circular Path")

    # Annotate key points
    plt.text(start_pose.x, start_pose.y, " Start", fontsize=8, verticalalignment='bottom')
    plt.text(goal_pose.x, goal_pose.y, " Goal", fontsize=8, verticalalignment='bottom')
    plt.text(parking_planner.x_tan, parking_planner.y_tan, " Tan", fontsize=8, verticalalignment='bottom')
    plt.text(parking_planner.x_mid, parking_planner.y_mid, " Mid", fontsize=8, verticalalignment='bottom')
    plt.text(parking_planner.x_turn, parking_planner.y_turn, " Turn", fontsize=8, verticalalignment='bottom')
    plt.text(parking_planner.r1_center[0], parking_planner.r1_center[1], " R1", fontsize=8, verticalalignment='bottom')
    plt.text(parking_planner.r2_center[0], parking_planner.r2_center[1], " R2", fontsize=8, verticalalignment='bottom')

    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.title("Parking Motion Planner")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.show()

if __name__ == "__main__":
    main()

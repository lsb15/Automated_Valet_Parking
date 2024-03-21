class Pose:
    def __init__(self, x, y, theta):
        """
        Constructor for creating a Pose object.

        Args:
        - x (float): The x-coordinate of the pose.
        - y (float): The y-coordinate of the pose.
        - theta (float): The orientation (in radians) of the pose.
        """
        self.x = x # x-coordinate of the pose
        self.y = y # y-coordinate of the pose
        self.theta = theta # orientation (in radians) of the pose

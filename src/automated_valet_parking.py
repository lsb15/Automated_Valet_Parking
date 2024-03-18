from free_space_map import FreeSpaceMap
from hybrid_a_star_route_planner import HybridAStarRoutePlanner
from parking_planner import ParkingPlanner
import matplotlib.pyplot as plt

class AutomatedValetParking:
    def __init__(self):
        self.free_space_map = FreeSpaceMap("modified_map.pgm")
        self.route_planner = HybridAStarRoutePlanner()
        self.parking_planner = ParkingPlanner()

    def run(self, drop_off_spot, parking_space, show_process=False):
        self.free_space_map.set_drop_off_spot(drop_off_spot[0], drop_off_spot[1], drop_off_spot[2])
        self.free_space_map.set_parking_space(parking_space[0], parking_space[1], parking_space[2])

        self.parking_planner.plan(
            self.free_space_map.get_drop_off_spot(),
            self.free_space_map.get_parking_space()
        )
        self.free_space_map.set_goal_state_pose(self.parking_planner.get_goal_pose())

        self.free_space_map.plot_map()
        plt.plot(drop_off_spot[0], drop_off_spot[1], 'bo', label='Drop-off Spot')
        plt.plot(parking_space[0], parking_space[1], 'ro', label='Parking Space')
        self.free_space_map.plot_route(self.parking_planner.r1_x, self.parking_planner.r1_y)
        self.free_space_map.plot_route(self.parking_planner.r2_x, self.parking_planner.r2_y)

        rx, ry = self.route_planner.search_route(self.free_space_map, show_process)
        plt.title('Automated Valet Parking')
        plt.legend()
        plt.show()


def main():
    automated_valet_parking = AutomatedValetParking()
    drop_off_spots = [(5262, 5781, 0), (830, 1904, 60)]
    parking_space = (7695, 5508, 90)
    automated_valet_parking.run(drop_off_spots[0], parking_space, False)


if __name__ == "__main__":
    main()

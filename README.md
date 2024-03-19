# Automated_Valet_Parking
Automated_Valet_Parking (Individual Project)


Automated Valet Parking is a Python project aimed at implementing an automated parking system using hybrid A* path planning. The system allows a vehicle to navigate autonomously from a drop-off spot to a designated parking space.

## Features:
- ***Hybrid A* Route Planning**: The project utilizes a hybrid A* algorithm to find the shortest path from the drop-off spot to the parking space while considering the vehicle's kinematics.
- **Visualization**: The map and planned route are visualized using Matplotlib, providing a graphical representation of the parking environment and the planned path.
- **Drop-off Spot and Parking Space Configuration**: Users can specify the drop-off spot and parking space coordinates and orientations, allowing for flexibility in different parking scenarios.
- **Obstacle Avoidance**: The system detects obstacles in the parking environment and plans a collision-free route to the parking space.
- **Flexible Integration**: The project is designed to be easily integrated into larger autonomous vehicle systems, providing a modular approach to parking navigation.

## Project Structure:
- **Automated Valet Parking Class**: This class orchestrates the entire parking process, including setting up the parking environment, planning the route, and visualizing the results.
- **FreeSpaceMap Class**: Handles the representation of the car park map, obstacle detection, and visualization of the parking environment.
- **HybridAStarRoutePlanner Class**: Implements the hybrid A* algorithm for route planning, considering vehicle kinematics and obstacle avoidance.
- **ParkingPlanner Class**: Responsible for planning the parking maneuver and determining the optimal parking space.
- **Pose Class**: Represents the pose (position and orientation) of the vehicle in the parking environment.

## Usage:
1. Specify the drop-off spot and parking space coordinates and orientations.
2. Run the main script to initiate the automated valet parking process.
3. Visualize the map, planned route, and parking maneuver using Matplotlib.



Automated Valet Parking offers a robust solution for automating the parking process, enhancing convenience and safety in parking environments.

*Reference:
Siomonjjeon (Simon Jeon) / https://github.com/simonjjeon

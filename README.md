# Assignment 2

This repository contains two branches, each implementing robot navigation functionalities using different versions of ROS:

- **`ros1` Branch**: Implements an action client and service node for robot navigation in ROS 1.
- **`ros2` Branch**: Implements a node for robot navigation in ROS 2.

## Branch Details

### `ros1` Branch

In the `ros1` branch, the package `assignment2_rt` provides the following components:

1. **Action Client Node**:
   - Allows the user to set a target position `(x, y)` or cancel it.
   - Utilizes the feedback from the action server to determine when the target has been reached.
   - Publishes the robot's position and velocity as a custom message `(x, y, vel_x, vel_z)`, based on data from the `/odom` topic.

2. **Service Node**:
   - Returns the coordinates of the last target set by the user when called.

3. **Launch File**:
   - Starts the entire simulation environment, including the nodes mentioned above.

### `ros2` Branch

In the `ros2` branch, the task involves:

- Implementing a ROS 2 package with a single node capable of moving the robot around in the simulation environment.
- The node prompts the user to input linear velocity `x` and angular velocity `z`, moving the robot for 3 seconds per command, continuing until the program is terminated.

## Getting Started

To explore the implementations:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/AlessandroMangili/assignment2_rt.git
   ```
2. **Checkout the Desired Branch**:
   ```bash
   # For ROS 1 implementation
   git checkout ros1

   # For ROS 2 implementation
   git checkout ros2
   ```
3. **Follow the Instructions**:
   - Each branch contains a `README.md` file with detailed setup and usage instructions specific to that implementation.
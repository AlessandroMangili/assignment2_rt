# Assignment 2 - Action Client and Service Node for Robot Navigation

## Overview
This project involves creating a ROS package named `assignment2_rt` that implements two key components to move the robot by setting a goal to reach. These components are:

1. **Action Client Node (node1)**: an action client that allows the user to set a goal or cancel it, and publishes the custom message `Robot_info`, composed of the following fields:
    - `float64 x`
    - `float64 y`
    - `float64 vel_x`
    - `float64 vel_z`
on the `/robot_position_velocity` topic.
2. **Service Node (node2)**: a service that returns the coordinates of the last goal set by the user through the first node.

## Nodes Description

### 1. Action Client Node
Everything is managed in the same terminal tab where the launch file was started. The action client will prompt you to press one of three keys: 
- `s` to set a new goal, 
- `c` to cancel an active goal, 
- `q` to exit the program. 

If `s` is pressed, you will also need to enter the `x` and `y` coordinates of the robot's destination.

After setting the goal, once the robot reaches it, a warning will be printed containing the robot's current position and the goal status, which will be: `Goal reached!`.

Additionally, this node publishes the robot's position and velocity as a custom message containing `x, y, vel_x, vel_z`, using the `/odom` topic data.

### 2. Service Node
This node is a service that allows retrieving the last goal set to the action server. The functionality is simple, no specific request is needed, and the service response will simply be the last goal set of type `geometry_msgs/PoseStamped`.

### 3. Launch File
The launch file `simulation.launch` starts all the simulation nodes present in the `assignment1.launch` file and additionally launches the two newly created nodes.

# Install the Package

## Prerequisites
- **ROS** must be installed on your system.
- A properly configured ROS workspace.
- `Gazebo` and `RViz` installed

## 1. Download the Package
- Open a terminal.
- Navigate to the `src` folder of your ROS workspace:
  ```bash
  cd ~/catkin_ws/src
  ```
- Clone the `assignment2_rt` package repository:
  ```bash
  git clone https://github.com/AlessandroMangili/Assignment2_rt
  ```
-  Enter the repo and switch to the `ros1` branch:
    ```bash
    cd Assignment2_rt
    git switch ros1
    ```

## 2. Compile the Package
- Navigate back to the main workspace directory:
  ```bash
  cd ~/catkin_ws
  ```
- Compile the package using `catkin_make`:
  ```bash
  catkin_make
  ```
- Check that the package compiled successfully. If there are no errors, a build completion message will appear.

## 3. Source the Environment
- Update the ROS environment variables to make your package accessible by adding the following line inside your `~/.bashrc` file:
  ```bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  ```
  To apply the changes, type the command:
  ```bash
  source ~/.bashrc
  ```

## 4. Run the Package

### 4.1. Start the Simulation from launch file
- Open the terminal and run the whole simulation
  ```bash
  roslaunch assignment2_rt simulation.launch
  ```

### 4.2 (Optional) Call to the service node
- If you want to make a call to the service node to get the last goal that was sent to the action server, you need to open a new terminal tab and type the command:
  ```bash
  rosservice call /get_last_goal
  ```
  and you will receive a message of type `geometry_msgs/PoseStamped` containing the last set goal

## 5. Interacting with the Nodes

- **Action Client Node**: everything is managed in the same terminal tab where the launch file was started. The action client will prompt you to press one of three keys: 
    - `s` to set a new goal, 
    - `c` to cancel an active goal, 
    - `q` to exit the program. 
    
    **Don't worry about the messages being printed; the action client will always be listening!**
  
- **Service Node**: 
    To interact with it, just open a new terminal and use the command `rosservice call /get_last_goal`
# TurtleBot3 Cleaning System

This repository contains the full ROS2 package from **Group 1** for the MCG5138 Introduction to Autonomous Mobile Robotics course project. It contains client/server pairs for cleaning or navigating to designated rooms using a TurtleBot3 Waffle (TB3) robotic platform. It sends cleaning evaluation start and stop requests automatically to the evaluator node during cleaning. 

## 1. Installation

1. Clone this repository into your ROS 2 workspace's `src` folder:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/AidenCorke/ROS2_Humble_TB3_CleanRoom.git
   ```
2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## 2. Launching the system

Terminal 1 - Simulation Files:
Using a single launch file we launch the TB3 house environment in gazebo, the Navigation2 stack with custom parameters and a previously made map, and RViz with a custom setup. House map has been created using SLAM Toolbox with resulting map saved within the bringup package. It is automatically loaded during launch using these commands. 

```bash
ros2 launch cleaner_bringup cleaner_sim.launch.py
```

Terminal 2 - Cleaning server, room navigation server, and evaluation node:
Next we launch the server nodes for the clean_room action and the go_to_room action. We also launch the evaluator node here.

```bash
ros2 launch cleaner_bringup cleaner_eval.launch.py 
```
Note you must source the local overlay for each terminal.


## 3. Navigating to rooms

To send commands to navigate to rooms you must run the appropriate client node with an argument as follows:

```bash
ros2 run go_to_room go_to_room_client <room_name>
```
*room_name options: [bedroom, library, living_room, hallway, kitchen, pantry, dining_room]*


## 4. Cleaning rooms

To start a room cleaning operation you must launch teh appropriate client node with a room name argument as follows:

```bash
ros2 run clean_room clean_room_client <room_name>
```
*room_name options: [bedroom, library, living_room, hallway, kitchen, pantry, dining_room]*

## Notes
To change between the default and tuned Nav2 parameters the nav2_params.yaml file will need to be swapped. For simplicity a copy of each are provided in the cleaner_bringup/config/ folder.

The included MCG5138_Demo_Group1.webm file is the recorded demo video of the robot in action. It shows the launching of the files, room-to-room navigation, and cleaning. At the end a second cleaning command is given in a room where a dynamic obstacle was added, the video shows the canceling behaviour of the cleaning program in this part.
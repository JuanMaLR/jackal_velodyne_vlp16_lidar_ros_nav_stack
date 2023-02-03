## Jackal Velodyne VLP16 3D LiDAR ROS Navigation Stack

# General characteristics
- ROS Noetic

# Description
The jackal robot will use a 3D LiDAR (Velodyne VLP16) to navigate autonomously. 
In order to accomplish this, the next steps are taken:
- Read a PointCloud2 using a 3D LiDAR placed in the center of the robot. 
- Use the PCL library to eliminate all the irrelevant data for the navigation like the ground or points that are higher than the robots height. 
- Use the PCL library to downsample the PointCloud2 in order to increase performance.
- Convert the PointCloud2 into LaserScan data using the pointcloud_to_laserscan library.
- Create a map of the environment using gmapping from ROS navigation Stack. 
- Navigate autonomously using both local (for obstacle avoidance) and global (for path planning) costmaps using ROS Navigation Stack. 

The project will include a maze simulation and a real world scenario greenhouse with a maze-like structure. 

# Installation
To download and be able to execute this project: 
- Clone this project
- Run ``` catkin_make ``` to compile
- Install any missing dependencies ``` rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y ```
- Install Jackal packages ``` sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation ```

# Instructions 
In order to run the simulation, execute the following launch file: 
```
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack simulation.launch
```

The simulation includes already the map of the maze and will launch Gazebo, RViz (with a predefined configuration) and many other nodes including the ROS Navigation Stack.

If the user changes the gazebo simulated world a new map should be made. To do so:
- Read simulation.launch file and toggle comments to match the mapping feature. 
- Use the interactive markers (or any other alternative like twist teleop) to move the robot and create a map of the environment (be careful of speed, if robot goes too fast the map might get distorted). 
- Once the map is completed save the map in the corresponding folder using the following commands on a new terminal: ``` rosrun map_server map_saver -f mymap ```. Where mymap can be any name you choose for your map.
- Wait for map to be saved and terminate execution. 
- Restore navigation launch files in simulation.launch.  

Once the map is ready and the simulation.launch file has been changed accordingly, running the simulation.launch file should open Rviz where the user should first indicate the intial pose of the robot and then indicate a goal pose so the robot starts navigating autonomously. 

In order to run the real robot project, execute the following launch files: 
- In jackal on-board computer (Required SSH connection):
```
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack real_robot.launch
```
Remember to add in the robot configuration file the ROS_HOSTNAME variable

- On ground computer: 
```
export ROS_MASTER_URI=http://ip:port
export ROS_HOSTNAME=robot_name
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack real_pc.launch
```

For more instructions on how to configure this connection check: [jackal_site](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/network.html)
and [jackal_video](https://www.youtube.com/watch?v=U-YgKVRDc3w)

# Demonstration and explanation
Please refer to the following video to check a brief explanation of the project as well as a demonstration.
[Project](https://youtu.be/mxL48slc6j4)

# Special considerations

If the simulation is failing or it's too slow it might be due to Gazebo version. To fix that, run the following commands so you're sure you have the latest stable Gazebo version running:
``` sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb_release -cs\` main" > /etc/apt/sources.list.d/gazebo-stable.list' ```
``` wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - ```
``` sudo apt update ```
``` sudo apt upgrade ```
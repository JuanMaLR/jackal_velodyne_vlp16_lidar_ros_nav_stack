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
So, using Rviz the user should first indicate the intial pose of the robot and then indicate a goal pose and the robot will start navigating autonomously. 

If the user whishes to create the map by themselves, comments in the code will indicate how to do this. 

# Demonstration and explanation
Please refer to the following video to check a brief explanation of the project as well as a demonstration.
[Project](https://youtu.be/mxL48slc6j4)

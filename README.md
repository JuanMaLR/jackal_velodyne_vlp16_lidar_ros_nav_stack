## Jackal Velodyne VLP16 3D LiDAR ROS Navigation Stack

# General characteristics
- ROS Noetic

# Description
The robot will use a 3D LiDAR to generate the map of the environmet by:
- Using the pcl library to downsample and select the relevant points of the PointCloud2.
- Converting the PointCloud2 into LaserScan data using the pointcloud_to_laserscan library.
- Using the ROS Navigation Stack to perform Path Planning and object avoidance using a Jackal ClearPath Robotics robot. 

The project will include a maze simulation and the real world scenario will be a greenhouse with a maze-like structure. 

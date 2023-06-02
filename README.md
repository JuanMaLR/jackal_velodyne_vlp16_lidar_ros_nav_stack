## Jackal Velodyne VLP16 3D LiDAR vs 2D LiDAR ROS Navigation Stack

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
To download and be able to execute this project: - Update and upgrade your Operating System (OS) using: ``` sudo apt-get update && upgrade ```
- Go to the catkin_ws directory under the src folder and clone the repository found [here](https://github.com/JuanMaLR/jackal_velodyne_vlp16_lidar_ros_nav_stack). This will download the project and all its contents. 
- Without changing directories clone the RPLiDAR A3 Github found at https://github.com/Slamtec/rplidar_ros. This will enable all the files required for controlling the 2D LiDAR from ROS. 
- Install the PCL libraries and PointCloud to LaserScan conversions package in ROS using:
	- ``` sudo apt-get install ros-noetic-pcl-conversions ```
	- ``` sudo apt-get install ros-noetic-pcl-ros ```
	- ``` sudo apt install ros-noetic-pointcloud-to-laserscan ```
- Install the Velodyne VLP16 drivers. To do so, execute: ``` sudo apt-get install ros-noetic-velodyne ```. This will install the Velodyne packages for ROS. 
- If using any ROS version prior to the noetic version (i.e., melodic), install the Jackal packages using: ``` sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation ```. This will download all the files required to simulate a Jackal and important files used in the real-life implementation. If noetic is used, the above commands might not work. Try installing them using ``` apt-get ```; if unsuccessful, install the packages from source. To install from source means to enter the catkin_ws/src folder and execute ``` git clone ``` command to each package required to be installed. Then go up one directory and compile each downloaded package using ``` catkin_make ```. The GitHub URL of the packages required to be installed from source are:
	- https://github.com/jackal/jackal_desktop
	- https://github.com/jackal/jackal_simulator
	- https://github.com/jackal/jackal
- Most systems will require the installation from source of the following two packages: 
	- https://github.com/uos/sick_tim.git
	- https://github.com/ros-drivers/flir_camera_driver.git. This package does not require compiling. Compiling will generate an error saying that *Spinnaker.h: File or directory does not exist*.
- Depending on what previous projects you have worked with on ROS, you might have already installed the other required packages for this project. If you are unsure or want to double check, install from source all the following packages:
	- https://github.com/jackal/jackal_robot
	- https://github.com/ros-visualization/interactive_marker_twist_server
	- https://github.com/clearpathrobotics/lms1xx
	- https://github.com/ros-drivers/rosserial
	- https://github.com/ros-perception/openslam_gmapping
	- https://github.com/ros-perception/slam_gmapping
- Install the 3D LiDAR drivers using: ``` git clone https://github.com/ros-drivers/velodyne.git ``` 
- Go to catkin_ws folder, check for missing dependencies, and install them using: ```  rosdep install --from-paths src --ignore-src --rosdistro noetic -y ``` . Note that only dependencies declared in the *package.xml* and *CMakeLists.txt* will be installed by this command. Other packages might need to be installed additionally and will be flagged as errors during compilation. 
- Once all the packages have been cloned, they must be compiled. To do so, go one directory up using ``` cd .. ``` and compile them in either of two ways:
	- Execute ``` catkin_make ``` to compile all the contents of your catkin_ws directory. 
	- Execute ``` catkin_make --only-pkg-with-deps package_name ``` where package_name is the name of the package your downloaded using ``` git clone ```. The package's name is usually the last part of the GitHub URL; all that follows the last forward slash (/). 
- Every computer is different, and hence additional packages might be needed to be installed. If while compiling any of the packages, an error of type *missing package or package xxx not found* is found, try to install that package first using ``` sudo apt-get install ros-noetic-package-name ``` where the package name is the name of the package to be installed. If a response of type *unable to locate package <package name>* is obtained, then install that package from source as indicated by the instructions above. 

For additonal packages installation instructions, please refer to [dinvincible98 GitHub](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup)

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
- Make sure the map's name you'll use is set to "mymap" and is inside the maps folder. If you have several map files give them a temporary name. The alternative is to modify the robot's params files but that's more cumbersome. 

Once the map is ready and the simulation.launch file has been changed accordingly, running the simulation.launch file should open Rviz where the user should first indicate the intial pose of the robot and then indicate a goal pose so the robot starts navigating autonomously. 

If the user wants to run the simulation using a 2D LiDAR execute the following launch file: 
```
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack simulation2d.launch
```

In order to run the real robot project using 3D LiDAR:
- First, connect to the robot following the steps specified inside the files/Setting up Jackal.txt file
- Then, verify that the jackal has the following exports inside the .bashrc file on root:
```
export ROS_MASTER_URI=http://192.168.0.102:11311
export ROS_HOSTNAME=192.168.0.102
export ROS_IP=192.168.0.102
```

- Finally, on the jackal on-board computer execute:
```
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack real_robot.launch
```
This will start gmapping and move_base (if in mapping phase) or the amcl and move base (if in navigating phase), as well as the lidar and the static transform for the lidar.

To be able to visualize the robot, the topics, tf's, etc., use RVIZ on the ground computer. 
- First, verify that on your .bashrc you have: 
```
export ROS_MASTER_URI=http://cpr-j100-0395:11311
export ROS_HOSTNAME=192.168.0.10
export ROS_IP=192.168.0.10
```

Also, make sure that in your /etc/hosts file you add the following line:

```
192.168.0.102   cpr-j100-0395
```

This will let your computer know that the ip 192.168.0.102 has associated the hostname of the jackal robot (cpr-j100-0395)

Then, launch:
```
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack real_pc.launch
```


To launch the 2D LiDAR version do the same steps as above, just modify the names:
- In jackal on-board computer:
```
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack real_robot2d.launch
```

- On ground computer: 
```
roslaunch jackal_velodyne_vlp16_lidar_ros_nav_stack real_pc_2d.launch
```

For more instructions on how to configure this connection check: [jackal_site](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/network.html)
and [jackal_video](https://www.youtube.com/watch?v=U-YgKVRDc3w)

This and the previous section provide brief instructions on how to download and configure the project for the owner of the repository. Several adjustments like the IP addresses might be necessary depending on who clones the project. For more detailed instructions, please refer to the "Installation and Configuration" section on Chapter 3 of the [Master Tesis](www.google.com) this project is developed for. 

# LiDAR's configurations
Online documentation is a bit unclear, so under the files folder of this project two instruction manuals (one for 2D and other for 3D LiDAR) as txt files are included. 

# Demonstration and explanation
Please refer to the following video to check a brief explanation of the project as well as a demonstration.
[Project](https://youtu.be/mxL48slc6j4)

# Special considerations

If the simulation is failing or it's too slow it might be due to Gazebo version. To fix that, run the following commands so you're sure you have the latest stable Gazebo version running:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb_release -cs\` main" > /etc/apt/sources.list.d/gazebo-stable.list' 
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
sudo apt update 
sudo apt upgrade
```

To be able to compile the project in your jackal robot be sure to download pcl libraries as specified in the "For my LiDAR project" section in the files/Setting up Jackal.txt file

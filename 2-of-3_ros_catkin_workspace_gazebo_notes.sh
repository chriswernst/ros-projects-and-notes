# All ROS related code will reside in the catkin workspace

# Need to make two directories:
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Initialize the ws:
catkin_init_workspace

# Return to top level directory:
cd ..

# Build the workspace:
catkin_make

# NOTE: ROS packages have two types of dependencies: Build and Run

# Launching Multiple Nodes at Once with ROSLaunch (Pg. 27 of Robotics NB)

source devel/setup.bash
# Source the setup script -lives in /home/robond/catkin_ws/devel

# We can now launch SimpleArm
roslaunch simple_arm robot_spawn.launch
# This launches Gazebo and displays the arm in an upright position 

# Determine which dependencies are missing(top of Pg.28):
cd ~/catkin_ws/src
source devel/setup.bash
rosdep check simple_arm
# If necessary:
rosdep install -i simple_arm  # (Pg. 28 of Robotics NB)

# Developing ROS Packages of our Own (Mid pg. 28):
cd ~/catkin_ws
source devel/setup.bash
# Set directory to top level and source setup

catkin_create_pkg first_package #[dependency1 dependency2]
# Create the package and name it

# ROS packages typically have various items that include:
	# Python Executables, src(C++ source files), msg, srv(service msg defs), include(headers), config, launch(automated way of starting node), 
	# URDF(universal robot description files), meshes(CAD in stl or dae), worlds(XML files for Gazebo environments)

# Top of Pg. 29 is next and continued in writing_ros_nodes_in_python.sh






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

# Determine which dependencies are missing:
cd ~/catkin_ws/src
source devel/setup.bash
rosdep check simple_arm
# If necessary:
rosdep install -i simple_arm  # (Pg. 28 of Robotics NB)



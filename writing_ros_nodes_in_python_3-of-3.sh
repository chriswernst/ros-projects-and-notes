# Writing ROS Nodes in Python
# Top of Pg. 29 in Robotics NB

# In this document, we're going to write 4 ROS Nodes:
	# 1. Hello World - 'hello'
	# 2. Moves both joints 180 deg back and forth - 'simple_mover'
	# 3. Allows for movement commands  - 'arm_mover'
	# 4. Use of subscribers 'look_away' 


### First, a test script of Hello World
cd ~/catkin_ws/src/simple_arm/
mkdir scripts
# Make directory if doesn't exist

cd scripts
echo '#!/bin/bash' >> hello
echo 'echo Hello World' >> hello
# Adds two new lines to the file 'hello'

# Now want to set permissions, rebuild the ws, source environment, run script
chmod u+x hello # Permissions
cd ~/catkin_ws
catkin_make # Builds the workspace
source devel/setup.bash # Source environment
rosrun simple_arm hello # run script - simply echos 'Hello World' to terminal

### END test Node of Hello World


### Now, for the 2nd ROS Node 'simple_mover' this publishes joint angle commands to simple_arm
cd ~/catkin_ws/src/simple_arm/
cd scripts
touch simple_mover # Creates the file
nano simple_mover # edit the script - add in code

# Want to set permissions, rebuild the ws, source environment, run script
chmod u+x simple_mover # Permissions
cd ~/catkin_ws
catkin_make # Builds the workspace
source devel/setup.bash # Source environment
roslaunch simple_arm robot_spawn.launch # This will launch Gazebo

# Now open a new terminal, and type
cd catkin_ws
catkin_make # Builds the workspace
source devel/setup.bash # Source environment
rosrun simple_arm simple_mover # run script - This makes the once stationary arm rotate around
# Starting to see how we might be able to program this bot to execute the Pick and Place Project!

# Note on rosrun command structure:
# rosrun folder_name_in_src file_name_from_scripts_folder
# In simple_arm folder, 2 files currently exist: 'hello' and 'simple_mover'

### END of simple_mover ROS Node (Bottom of Pg.29 of Robo NB)


### Now, for the 3rd ROS Node, 'arm_mover' (Top of Pg.30):

# arm_mover ROS node provides the service move_arm which allows other nodes to send movement_commands

# We need to create a new Service Definition -  the .srv files will live in the srv directory

# REMINDER: a service consists of a request to the service, and a response from
	# much like an ad-hoc pub/sub structure

# Let's create a new service for simple_arm: we'll call it 'GoToPosition' (Pg.30 of NB)

cd ~/catkin_ws/src/simple_arm/
mkdir srv
cd srv
touch GoToPosition.srv
nano GoToPosition.srv
# Add in the lines:
float64 joint_1  
float64 joint_2 # definition of the request
---

duration time_elapsed # definition of the response
# Service Definitions always contain two sections separated by '---'. First is the request, then response.

# Now, we need to alter the CMakeFile
cd ~/catkin_ws/src/simple_arm/
nano CMakeLists.txt

# Once inside, alter the find_package() to only include:
std_msgs
message_generation

# Uncomment the below to match:
add_service_files(
	FILES
	GoToPosition.srv
)

# Verify generate_messages is uncommented:
generate_messages(
	DEPENDENCIES
	std_msgs
)

# Now, verify the so package.xml (cd ~/catkin_ws/src/simple_arm/) so that it looks like this:
<buildtool_depend>catkin</buildtool_depend>
<build_depend>message_generation</build_depend>

<run_depend>controller_manager</run_depend>
<run_depend>effort_controllers</run_depend>
<run_depend>gazebo_plugins</run_depend>
<run_depend>gazebo_ros</run_depend>
<run_depend>gazebo_ros_control</run_depend>
<run_depend>joint_state_controller</run_depend>
<run_depend>joint_state_publisher</run_depend>
<run_depend>robot_state_publisher</run_depend>
<run_depend>message_runtime</run_depend>
<run_depend>xacro</run_depend>

# Now, build it, and source it: (Pg. 31)
cd ~/catkin_ws/
catkin_make # Build it
source devel/setup.bash

# Create the empty arm_mover script
cd ~/catkin_ws/src/simple_arm/scripts/
touch arm_mover
chmod u+x arm_mover

# Fill in arm_mover script with file 'arm_mover' ~71 lines of python

# Verify the contents of robot_spawn.launch
cd ~/catkin_ws/src/simple_arm/launch
nano robot_spawn.launch

# Make sure the following is within:
<node name="arm_mover" type="arm_mover" pkg="simple_arm">
	<rosparam>
	  min_joint_1_angle: 0
	  max_joint_1_angle: 1.57
	  min_joint_2_angle: 0
	  max_joint_2_angle: 1.0
	</rosparam>
</node>

# Check that it works. Exit all terminal and gazebo windows
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch
# This should start Gazebo

# Open a new Terminal, and type:
rosnode list # check that /arm_mover is listed as a node
rosservice list # check that /arm_mover/safe_move is listed as a service

# We now want to view the camera image at the end of the arm:
rqt_image_view /rbg_camera/image_raw
# This should display a gray image (since the camera is pointed up)
# If the camera stays grey, try using the dropdown in the gui, or:
rqt_image_view /rbg_camera/image_raw/theora

# In a new terminal:
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Now we want to move the arm
rosservice call /arm_mover/safe_move "joint_1: 1.57 
	joint_2: 1.57"
# It is crucial there is a line break above
# This should rotate the arm to be pointed just above the blocks

# We need to change the max_joint_2_angle to 1.57 radians ~90deg
rosparam set /arm_mover/max_joint_2_angle 1.57

# Now call back the move
rosservice call /arm_mover/safe_move "joint_1: 1.57 
	joint_2: 1.57"

# Have some fun with this, and try different radians:
rosservice call /arm_mover/safe_move "joint_1: 0 
	joint_2: 0"
	# Points the arm vertically
# NOTE: the base is joint_1, the shoulder is joint_2

### END ROS Node 'arm_mover'

# A brief on ROS Subscribers (Pg. 48 of NB)
# Subscribers are written in Python and take the general form:
sub1 =  rospy.subscriber("/topic_name", message_type, callback_function)
	# Where: 
	# message_type is the type of message being published on /topic_name
	# callback_function is the function that is executed with each incoming message

# For example, from look_away:
    def __init__(self):
        rospy.init_node('look_away')

        self.sub1 = rospy.Subscriber('/simple_arm/joint_states', 
                                     JointState, self.joint_states_callback)
        self.sub2 = rospy.Subscriber("rgb_camera/image_raw", 
                                     Image, self.look_away_callback)
        self.safe_move = rospy.ServiceProxy('/arm_mover/safe_move', 
                                     GoToPosition)

        self.last_position = None
        self.arm_moving = False

        rospy.spin()


### BEGIN ROS Node 'look_away' (Pg. 48 of Robo NB)
# An example of subscribers in action

# Create a new file 'look_away'
cd ~/catkin_ws/src/simple_arm/scripts
touch look_away
chmod u+x look_away

# Need to modify the launch file so look_away will launch with the rest of the nodes
cd ~/catkin_ws/src/simple_arm/launch
# Update the robot_spawn.launch file:
nano robot_spawn.launch
# Add the line:
<!-- The look away node -->
<node name="look_away" type="look_away" pkg="simple_arm"/>

# Now, go back into look_away and open the nano editor
# Paste the code in there from 'look_away'

# We can now launch simple_arm as before
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch

# Open a new terminal for the camera:
rqt_image_view /rbg_camera/image_raw

# Open a new terminal and build and source again
catkin_make
source devel/setup.bash

rosservice call /arm_mover/safe_move "joint_1: 0 
	joint_2: 0"
# This should make the arm point to the sky, but then rotate back toward the blocks





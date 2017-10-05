# PASSWORD for Nanodegree VM is robo-nd

# ctrl + alt + t   opens a new terminal window
source /opt/ros/kinetic/setup.bash
# ros tab tab gives available ros commands

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# The environment is now all set up! Time to begin TurtleSim!

# Start the ROS master process with:
roscore

# Open a new terminal window and type:
rosrun turtlesim turtlesim_node
# This should have opened up the blue TurtleSim window

# Open a new terminal window and type:
rosrun turtlesim turtle_teleop_key
# This will create a command console from the terminal window
# The arrow keys will move the turtle about

# To understand which ROS nodes are running, open a new terminal:
rosnode list
# This should return /rosout /teleop_turtle /turtlesim

# To understand which ROS topics exist:
rostopic list
# Returns /rosout /rosout_agg /turtle1/cmd_vel /turtle1/color_sensor /turtle1/pose

# To learn more about a topic, see the publishers/subscribers:
rostopic info turtle1/color_sensor

rosmsg list
# Returns a list of all of the messages

rosmsg show  geometry_msgs/Twist
# To get more information on a message

rosed geometry_msgs Twist.msg
# To learn more about a message/read the source code

rostopic echo /turtle1/cmd_vel
# This gives the real-time information for what is being published to that topic
rostopic echo /turtle1/color_sensor
# Color sensor information

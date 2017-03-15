# open_manipulator

# remotePC

$ roscore

$ roslaunch open_manipulator_moveit open_manipulator_demo.launch

$ rosrun open_manipulator_position_ctrl position_controller

$ rostopic pub /robotis/open_manipulator/pick_and_place std_msgs/String "init_joint"

# Raspberry PI 3

$ roslaunch ar_pose turtlebot3_pickandplace_ar_pose.launch

$ sudo chmod a+rw /dev/ttyUSB0
$ roslaunch open_manipulator_dynamixel_ctrl dynamixel_controller.launch

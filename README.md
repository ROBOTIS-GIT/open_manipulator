# open_manipulator chain

# Gazebo simulation

$ roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch

$ roslaunch open_manipulator_moveit open_manipulator_demo.launch use_gazebo:=true

# Platform (DYNAMIXEL X-Series)

$ roslaunch open_manipulator_dynamixel_ctrl dynamixel_controller.launch

$ roslaunch open_manipulator_moveit open_manipulator_demo.launch use_gazebo:=false


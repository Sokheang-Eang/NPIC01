# Check Camera ID
rs-enumerate-devices -s
# Run Micro-ROS
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
# Run Ball Tracking
ros2 run ball_tracking ball_blue
# Run Silo Tracking
ros2 run silo_tracking silo_blue
# Run Command Control
ros2 run robot_control robot
# Check rqt_graph
rqt_graph
# Check Robot Speed
ros2 topic echo /robot_speed
# Check Robot State
ros2 topic echo /robot_state


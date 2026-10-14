source "$HOME/ROS-Learning/ros2_week3_ws/setup_build.bash"
if [ ! -f "$HOME/ROS-Learning/ros2_week3_ws/install/local_setup.bash" ]; then
  echo "Build ~/ROS-Learning/ros2_week3_ws before loading its runtime environment"
  return 1
fi
source "$HOME/ROS-Learning/ros2_week3_ws/install/local_setup.bash"
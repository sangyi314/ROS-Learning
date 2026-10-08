source "$HOME/ROS-Learning/ros2_week2_ws/setup_build.bash"

if [ ! -f "$HOME/ROS-Learning/ros2_week2_ws/install/local_setup.bash" ]; then
  echo "Build ~/ROS-Learning/ros2_week2_ws first"
  return 1
fi

source "$HOME/ROS-Learning/ros2_week2_ws/install/local_setup.bash"
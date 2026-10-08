source "$HOME/ROS-Learning/ros2_week2_ws/setup_build.bash"
if [ ! -f "$HOME/ROS_Learning/ros2_week2_ws/setup_build.bash" ]; then
  echo "Build the workspace first: ~/ros2_week2_ws/install/local_setup.bash is missing"
  return 1
fi
source "$$HOME/ROS-Learning/ros2_week2_ws/setup_build.bash"
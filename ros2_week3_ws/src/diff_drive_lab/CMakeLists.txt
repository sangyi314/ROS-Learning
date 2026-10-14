cmake_minimum_required(VERSION 3.8)
project(diff_drive_lab)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

function(configure_program target)
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

function(add_math_test target)
  add_executable(${target} test/${target}.cpp)
  configure_program(${target})
endfunction()

function(add_ros_node target)
  add_executable(${target} src/${target}.cpp)
  configure_program(${target})
  ament_target_dependencies(${target} rclcpp geometry_msgs)
endfunction()

add_math_test(test_kinematics)
add_math_test(test_command_buffer)
add_ros_node(cmd_probe)

ament_package()
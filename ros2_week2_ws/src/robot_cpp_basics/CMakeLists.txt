cmake_minimum_required(VERSION 3.8)
project(robot_cpp_basics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

function(add_lesson_node target source_file)
  add_executable(${target} "${source_file}")

  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_target_dependencies(${target}
    rclcpp std_msgs robot_interfaces
  )

  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_lesson_node(hello_node src/hello_node.cpp)
add_lesson_node(number_publisher src/number_publisher.cpp)
add_lesson_node(number_subscriber src/number_subscriber.cpp)

add_lesson_node(sensor_publisher
  ../robot_interfaces/src/sensor_publisher.cpp
)

add_lesson_node(sensor_processor
  ../robot_interfaces/src/sensor_processor.cpp
)

ament_package()
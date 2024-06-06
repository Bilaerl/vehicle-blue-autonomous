cmake_minimum_required(VERSION 3.8)
project(vehicleblue)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator rclcpp geometry_msgs nav_msgs tf2)

# install navigator node
install(TARGETS navigator
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install gazebosim files
install(DIRECTORY gazebosim
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(loop_route)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_system_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

# include directories
include_directories(include)

# create library
add_library(looproute SHARED src/loop_route.cpp)
ament_target_dependencies(looproute rclcpp geometry_msgs autoware_auto_system_msgs autoware_auto_vehicle_msgs)

# create executable
add_executable(loop_route src/loop_route_node.cpp)
target_link_libraries(loop_route looproute)

# install targets
install(TARGETS
  loop_route
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

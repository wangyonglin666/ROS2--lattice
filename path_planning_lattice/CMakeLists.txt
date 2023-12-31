cmake_minimum_required(VERSION 3.8)
project(path_planning_lattice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lgsvl_msgs REQUIRED)

add_executable(lattice_planner 
                src/lattice_planner.cpp
                src/lqr_controller.cpp
                src/pid_controller.cpp
                src/frenet_optimal_trajectory.cpp
                src/reference_line.cpp
                )
target_include_directories(lattice_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lattice_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(lattice_planner "std_msgs" "rclcpp" "nav_msgs" "visualization_msgs" "tf2" "tf2_ros" "tf2_geometry_msgs" "lgsvl_msgs")# ipopt z ssl uv uWS


install(TARGETS lattice_planner
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
config
rviz
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

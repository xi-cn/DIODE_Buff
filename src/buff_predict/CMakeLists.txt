cmake_minimum_required(VERSION 3.8)
project(buff_predict)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PythonLibs 3.10)
find_package(global_msg REQUIRED)

set(PYTHON3_10_INLCUDE_DIRS "/usr/include/python3.10")


add_executable( buff_predict 
                src/buff_predict_node.cpp 
                src/kalman.cpp 
                src/solver.cpp 
                src/tracker.cpp)


include_directories(include 
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON3_10_INLCUDE_DIRS}
)

target_include_directories(buff_predict PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_link_libraries(buff_predict
  ceres
)

ament_target_dependencies(
  buff_predict
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "Eigen3"
  "Ceres"
  "PythonLibs"
  "global_msg"
)

install(TARGETS buff_predict
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

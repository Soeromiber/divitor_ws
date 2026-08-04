cmake_minimum_required(VERSION 3.8)
project(divitor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

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

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/mlx90640-library/headers/)

add_executable(imx219_camera src/imx219_camera_node.cpp)
ament_target_dependencies(imx219_camera rclcpp sensor_msgs OpenCV)

add_executable(mlx90640_camera
  src/mlx90640_camera_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mlx90640-library/functions/MLX90640_API.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mlx90640-library/functions/MLX90640_LINUX_I2C_Driver.cpp
)
ament_target_dependencies(mlx90640_camera
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)

install(TARGETS
  imx219_camera
  mlx90640_camera
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

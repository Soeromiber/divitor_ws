cmake_minimum_required(VERSION 3.8)
project(divitor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(MLX90640_THIRD_PARTY_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mlx90640-library/functions/MLX90640_API.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mlx90640-library/functions/MLX90640_LINUX_I2C_Driver.cpp
)

add_library(imx219_camera_node_component SHARED
  src/imx219_camera_node.cpp)
target_include_directories(imx219_camera_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(imx219_camera_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV)
rclcpp_components_register_node(imx219_camera_node_component
  PLUGIN "divitor_driver::IMX219CameraNode"
  EXECUTABLE imx219_camera_node)
set_target_properties(imx219_camera_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

add_library(mlx90640_camera_component SHARED
  src/mlx90640_camera.cpp
  src/mlx90640_camera_node.cpp
  ${MLX90640_THIRD_PARTY_SOURCES})
target_include_directories(mlx90640_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/mlx90640-library/headers>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mlx90640_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV
  cv_bridge)
rclcpp_components_register_node(mlx90640_camera_component
  PLUGIN "divitor_driver::MLX90640CameraNode"
  EXECUTABLE mlx90640_camera)
set_target_properties(mlx90640_camera PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS
  imx219_camera_node_component
  mlx90640_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  imx219_camera_node
  mlx90640_camera
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(
  imx219_camera_node_component
  mlx90640_camera_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV
  cv_bridge)

ament_package()

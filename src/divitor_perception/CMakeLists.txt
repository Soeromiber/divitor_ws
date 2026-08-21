cmake_minimum_required(VERSION 3.8)
project(divitor_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(YOLOS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/YOLOs-CPP" CACHE PATH "Path to YOLOs-CPP")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

if(NOT DEFINED ONNXRUNTIME_DIR)
    file(GLOB DOWNLOADED_ONNX_DIRS "${YOLOS_CPP_DIR}/onnxruntime-*")
    if(DOWNLOADED_ONNX_DIRS)
        list(GET DOWNLOADED_ONNX_DIRS 0 LAUNCH_ONNX_PATH)
        set(ONNXRUNTIME_DIR "${LAUNCH_ONNX_PATH}" CACHE PATH "Path to ONNX Runtime")
        message(STATUS "Found ONNX Runtime dependency at: ${ONNXRUNTIME_DIR}")
    endif()
endif()

add_subdirectory(${YOLOS_CPP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/YOLOs-CPP)

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

include_directories(
  include
  ${YOLOS_CPP_DIR}/include
  ${ONNXRUNTIME_DIR}/include
)

# Thermal Detector Component
add_library(thermal_detector_component SHARED
  src/thermal_detector.cpp
  src/thermal_detector_node.cpp)
target_include_directories(thermal_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thermal_detector_component
  rclcpp
  rclcpp_components
  OpenCV
  sensor_msgs)
rclcpp_components_register_node(thermal_detector_component
  PLUGIN "divitor_perception::ThermalDetectorNode"
  EXECUTABLE thermal_detector_node)
set_target_properties(thermal_detector_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# Adaptive Thermal Detector Component
add_library(adaptive_thermal_detector_component SHARED
  src/adaptive_thermal_detector_node.cpp)
target_include_directories(adaptive_thermal_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(adaptive_thermal_detector_component
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  sensor_msgs
  vision_msgs)
rclcpp_components_register_node(adaptive_thermal_detector_component
  PLUGIN "divitor_perception::AdaptiveThermalDetectorNode"
  EXECUTABLE adaptive_thermal_detector_node)
set_target_properties(adaptive_thermal_detector_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# Thermal Preprocessor Component
add_library(thermal_preprocessor_component SHARED
  src/thermal_preprocessor_node.cpp)
target_include_directories(thermal_preprocessor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thermal_preprocessor_component
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  sensor_msgs)
rclcpp_components_register_node(thermal_preprocessor_component
  PLUGIN "divitor_perception::ThermalPreprocessorNode"
  EXECUTABLE thermal_preprocessor_node)
set_target_properties(thermal_preprocessor_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# Thermal Colormapper Component
add_library(thermal_colormapper_component SHARED
  src/thermal_colormapper.cpp)
target_include_directories(thermal_colormapper_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thermal_colormapper_component
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  sensor_msgs)
rclcpp_components_register_node(thermal_colormapper_component
  PLUGIN "divitor_perception::ThermalColormapperNode"
  EXECUTABLE thermal_colormapper_node)
set_target_properties(thermal_colormapper_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# YOLO Detector Component
add_library(yolo_detector_component SHARED
  src/yolo_detector_node.cpp)
target_include_directories(yolo_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(yolo_detector_component
  rclcpp
  rclcpp_components
  OpenCV
  sensor_msgs
  cv_bridge)
target_link_libraries(yolo_detector_component
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)
rclcpp_components_register_node(yolo_detector_component
  PLUGIN "divitor_perception::YoloDetectorNode"
  EXECUTABLE yolo_detector_node)
set_target_properties(yolo_detector_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS
  thermal_detector_component
  yolo_detector_component
  thermal_preprocessor_component
  thermal_colormapper_component
  adaptive_thermal_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  thermal_detector_node
  yolo_detector_node
  thermal_preprocessor_node
  thermal_colormapper_node
  adaptive_thermal_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(
  thermal_detector_component
  yolo_detector_component
  thermal_preprocessor_component
  thermal_colormapper_component
  adaptive_thermal_detector_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  OpenCV
  sensor_msgs
  cv_bridge)

ament_package()

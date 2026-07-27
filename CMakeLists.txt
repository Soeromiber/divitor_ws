cmake_minimum_required(VERSION 3.8)
project(divitor_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(YOLOS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/YOLOs-CPP" CACHE PATH "Path to YOLOs-CPP")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

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

add_executable(thermal_detector_node src/thermal_detector_node.cpp)
ament_target_dependencies(thermal_detector_node
  rclcpp
  OpenCV
  sensor_msgs
)

add_executable(yolo_detector_node src/yolo_detector_node.cpp)
ament_target_dependencies(yolo_detector_node
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  ament_index_cpp
)
target_link_libraries(yolo_detector_node
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
)

install(TARGETS
  thermal_detector_node
  yolo_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/models/yolo26n.onnx
  ${CMAKE_CURRENT_SOURCE_DIR}/models/coco.names
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='divitor_components_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gscam',
                plugin='gscam::GSCam',
                name='imx219_camera',
                parameters=[{
                    'gscam_config': 'libcamerasrc ! video/x-raw,width=1640,height=1232,framerate=30/1,format=RGB ! queue max-size-buffers=1 leaky=upstream ! videoconvert',
                    'use_sensor_data_qos': True,
                    'camera.image_raw.compressed.jpeg_quality': 30,
                }],
                remappings=[
                    ('/camera/image_raw', 'sensors/camera/rgb/image_raw'),
                    ('/camera/image_raw/compressed', 'sensors/camera/rgb/image_raw/compressed'),
                    ('/camera/image_raw/theora', 'sensors/camera/rgb/image_raw/theora'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ComposableNode(
            #     package='divitor_driver',
            #     plugin='divitor_driver::IMX219CameraNode',
            #     name='imx219_camera',
            #     parameters=[{
            #         'width': 1640,
            #         'height': 1232,
            #     }],
            #     extra_arguments=[{'use_intra_process_comms': True}],
            # ),
            ComposableNode(
                package='divitor_driver',
                plugin='divitor_driver::MLX90640CameraNode',
                name='mlx90640_camera',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='divitor_perception',
                plugin='divitor_perception::ThermalPreprocessorNode',
                name='thermal_preprocessor',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='divitor_perception',
                plugin='divitor_perception::AdaptiveThermalDetectorNode',
                name='adaptive_thermal_detector',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='divitor_perception',
                plugin='divitor_perception::YoloDetectorNode',
                name='yolo_detector_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    rgb_width = LaunchConfiguration('rgb_width')
    rgb_height = LaunchConfiguration('rgb_height')
    rgb_framerate = LaunchConfiguration('rgb_framerate')
    rgb_format = LaunchConfiguration('rgb_format')

    gscam_config = PythonExpression([
        '"gst-launch-1.0 libcamerasrc ! video/x-raw,width=', rgb_width, ',height=', rgb_height, ',framerate=', rgb_framerate, '/1,format=', rgb_format, ' ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=upstream ! videoconvert"'
    ])

    # Define composable nodes
    gscam_node = ComposableNode(
        package='gscam',
        plugin='gscam::GSCam',
        name='gscam_node',
        remappings=[
            ('/camera/image_raw', '/rgb_camera/image_raw'),
            ('/camera/image_raw/compressed', '/rgb_camera/image_raw/compressed'),
            ('/camera/image_raw/theora', '/rgb_camera/image_raw/theora'),
        ],
        parameters=[{
            'gscam_config': gscam_config,
            'use_sensor_data_qos': True,
            'sync_sink': False,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    mlx90640_camera_node = ComposableNode(
        package='divitor_driver',
        plugin='divitor_driver::MLX90640CameraNode',
        name='mlx90640_camera',
        extra_arguments=[{'use_intra_process_comm': True}],
    )

    # thermal_detector_node = ComposableNode(
    #     package='divitor_perception',
    #     plugin='divitor_perception::ThermalDetector',
    #     name='thermal_detector_node',
    #     extra_arguments=[{'use_intra_process_comm': True}],
    # )

    # yolo_detector_node = ComposableNode(
    #     package='divitor_perception',
    #     plugin='divitor_perception::YoloDetector',
    #     name='yolo_detector_node',
    #     extra_arguments=[{'use_intra_process_comm': True}],
    # )

    # Create container and load all components
    container = ComposableNodeContainer(
        name='sensor_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            gscam_node,
            mlx90640_camera_node,
            # thermal_detector_node,
            # yolo_detector_node,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rgb_width', default_value='640', description='Width of the RGB camera image'),
        DeclareLaunchArgument('rgb_height', default_value='480', description='Height of the RGB camera image'),
        DeclareLaunchArgument('rgb_framerate', default_value='30', description='Framerate of the RGB camera image'),
        DeclareLaunchArgument('rgb_format', default_value='NV12', description='Format of the RGB camera image'),
        container,
    ])
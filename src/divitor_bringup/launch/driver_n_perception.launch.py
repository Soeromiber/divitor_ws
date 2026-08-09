from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    rgb_width = LaunchConfiguration('rgb_width')
    rgb_height = LaunchConfiguration('rgb_height')
    rgb_framerate = LaunchConfiguration('rgb_framerate')
    rgb_format = LaunchConfiguration('rgb_format')

    gscam_config = PythonExpression([
        '"gst-launch-1.0 libcamerasrc ! video/x-raw,width=', rgb_width, ',height=', rgb_height, ',framerate=', rgb_framerate, '/1,format=', rgb_format, ' ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=upstream ! videoconvert"'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rgb_width', default_value='640', description='Width of the RGB camera image'),
        DeclareLaunchArgument('rgb_height', default_value='480', description='Height of the RGB camera image'),
        DeclareLaunchArgument('rgb_framerate', default_value='30', description='Framerate of the RGB camera image'),
        DeclareLaunchArgument('rgb_format', default_value='NV12', description='Format of the RGB camera image'),
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_node',
            remappings=[
                ('/camera/image_raw', '/rgb_camera/image_raw'),
                ('/camera/image_raw/compressed', '/rgb_camera/image_raw/compressed'),
                ('/camera/image_raw/theora', '/rgb_camera/image_raw/theora'),
            ],
            parameters=[
                {
                    'gscam_config': gscam_config,
                    'use_sensor_data_qos': True,
                    'sync_sink': False,
                },
            ]
        ),
        Node(
            package='divitor_driver',
            executable='mlx90640_camera',
            name='mlx90640_camera',
            output='screen'
        ),
        Node(
            package='divitor_perception',
            executable='thermal_detector_node',
            name='thermal_detector_node',
            output='screen'
        ),
        Node(
            package='divitor_perception',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen'
        ),
    ])


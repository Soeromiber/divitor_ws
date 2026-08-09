from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    framerate = LaunchConfiguration('framerate')
    format = LaunchConfiguration('format')

    gscam_config = PythonExpression([
        '"libcamerasrc ! video/x-raw,width=', width, ',height=', height, ',framerate=', framerate, '/1,format=', format, ' ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=upstream ! videoconvert"'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='640', description='Width of the camera image'),
        DeclareLaunchArgument('height', default_value='480', description='Height of the camera image'),
        DeclareLaunchArgument('framerate', default_value='30', description='Framerate of the camera image'),
        DeclareLaunchArgument('format', default_value='NV12', description='Format of the camera image'),
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
                {'gscam_config': gscam_config},
            ]
        )
    ])
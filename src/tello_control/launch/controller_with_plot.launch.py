"""Launch Tello controller and pose plotter nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rigid_body_name_arg = DeclareLaunchArgument(
        'rigid_body_name',
        default_value='drone',
        description='Name of the rigid body in Motive/OptiTrack'
    )

    return LaunchDescription([
        rigid_body_name_arg,

        # Tello controller node
        Node(
            package='tello_control',
            executable='tello_controller',
            name='tello_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{'rigid_body_name': LaunchConfiguration('rigid_body_name')}]
        ),

        # Pose plotter node
        Node(
            package='tello_control',
            executable='pose_plotter',
            name='pose_plotter',
            output='screen',
            parameters=[{'rigid_body_name': LaunchConfiguration('rigid_body_name')}]
        ),
    ])

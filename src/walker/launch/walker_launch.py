from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def add_rosbag_recording(context, *args, **kwargs):
    enable_rosbag = context.perform_substitution(LaunchConfiguration('enable_rosbag'))
    if enable_rosbag.lower() == 'true':
        return [
            Node(
                package='ros2bag',
                executable='record',
                name='rosbag_record',
                output='screen',
                arguments=[
                    '--all',                      # Record all topics
                    '--exclude', '^/camera/.*',   # Exclude topics matching `/camera/*`
                    '--output', 'rosbag'          # Name of the output bag file
                ],
            )
        ]
    return []

def generate_launch_description():
    # Set TurtleBot3 model environment variable
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    
    # Specify the correct world file (turtlebot3_house.world)
    world_file = os.path.join(
        os.environ.get('GAZEBO_MODEL_PATH', '/opt/ros/humble/share/turtlebot3_gazebo/models/'),
        'worlds',
        'turtlebot3_world.world'
    )

    # Launch argument to enable/disable rosbag recording
    enable_rosbag_arg = DeclareLaunchArgument(
        'enable_rosbag',
        default_value='false',
        description='Flag to enable or disable rosbag recording'
    )

    # Launch the Gazebo simulation with the TurtleBot3 model and the specified world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                os.environ.get('TURTLEBOT3_GAZEBO_DIR', '/opt/ros/humble/share/turtlebot3_gazebo/launch/'),
                'turtlebot3_world.launch.py'
            )
        ]),
        launch_arguments={'world': world_file}.items()  # Ensure the world file is passed
    )

    # Launch the walker node
    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker_node',
        output='screen'
    )

    # Add rosbag recording with conditional execution
    rosbag_nodes = OpaqueFunction(function=add_rosbag_recording)

    return LaunchDescription([
        enable_rosbag_arg,  # Declare the launch argument
        gazebo_launch,
        walker_node,
        rosbag_nodes,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('api_url', default_value='https://www.bbrobotics.in/api'),
        DeclareLaunchArgument('api_key', default_value=''),
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('observation_interval_ms', default_value='100'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('ft_sensor_topic', default_value='/ft_sensor'),
        DeclareLaunchArgument('gripper_topic', default_value='/gripper/state'),

        Node(
            package='blackbox_recorder',
            executable='episode_recorder',
            name='blackbox_episode_recorder',
            parameters=[{
                'api_url': LaunchConfiguration('api_url'),
                'api_key': LaunchConfiguration('api_key'),
                'robot_id': LaunchConfiguration('robot_id'),
                'observation_interval_ms': LaunchConfiguration('observation_interval_ms'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'ft_sensor_topic': LaunchConfiguration('ft_sensor_topic'),
                'gripper_topic': LaunchConfiguration('gripper_topic'),
            }],
            output='screen',
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Get the comma-separated list of IDs (e.g., "robot1,robot2,robot3")
    robot_ids_str = LaunchConfiguration('robot_ids').perform(context)
    
    # Fallback to single 'robot_id' if 'robot_ids' is empty
    if not robot_ids_str:
        single_id = LaunchConfiguration('robot_id').perform(context)
        robot_ids = [single_id] if single_id else []
    else:
        robot_ids = [rid.strip() for rid in robot_ids_str.split(',')]

    api_key = LaunchConfiguration('api_key').perform(context)
    api_url = LaunchConfiguration('api_url').perform(context)
    obs_interval = LaunchConfiguration('observation_interval_ms').perform(context)

    nodes = []
    for rid in robot_ids:
        nodes.append(Node(
            package='blackbox_recorder',
            executable='episode_recorder',
            name=f'recorder_{rid}',
            namespace=rid,  # Each robot gets its own namespace
            parameters=[{
                'api_url': api_url,
                'api_key': api_key,
                'robot_id': rid,
                'observation_interval_ms': int(obs_interval),
                # Since we use namespaces, these relative topics will resolve to:
                # /{rid}/joint_states, /{rid}/ft_sensor, etc.
                'joint_states_topic': 'joint_states',
                'ft_sensor_topic': 'ft_sensor',
                'gripper_topic': 'gripper/state',
            }],
            output='screen',
        ))
    
    if not nodes:
        print("WARNING: No robot_ids provided. No recorder nodes will be started.")

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Use 'robot_ids' for multiple robots (comma-separated)
        DeclareLaunchArgument(
            'robot_ids', 
            default_value='', 
            description='Comma-separated list of robot IDs to record from'
        ),
        # Keep 'robot_id' for backward compatibility
        DeclareLaunchArgument(
            'robot_id', 
            default_value='', 
            description='Single robot ID (fallback if robot_ids is empty)'
        ),
        DeclareLaunchArgument('api_url', default_value='https://www.bbrobotics.in/api'),
        DeclareLaunchArgument('api_key', default_value=''),
        DeclareLaunchArgument('observation_interval_ms', default_value='100'),
        
        OpaqueFunction(function=launch_setup)
    ])

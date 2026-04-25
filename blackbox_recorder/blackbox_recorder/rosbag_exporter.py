"""
Black Box Robotics Rosbag Exporter — ROS 2 node that exports rosbag2 data into
structured episodes and pushes to the Black Box Robotics API.

Usage:
  ros2 run blackbox_recorder rosbag_exporter --ros-args \
    -p bag_path:=/path/to/rosbag \
    -p api_url:=http://localhost:3001/api \
    -p api_key:=pk_... \
    -p robot_id:=<uuid> \
    -p task_id:=pick_and_place
"""

import json
from datetime import datetime, timezone

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False


class RosbagExporter(Node):
    """Reads a rosbag2 file and exports episodes to Black Box Robotics."""

    def __init__(self):
        super().__init__('blackbox_rosbag_exporter')

        self.declare_parameter('bag_path', '')
        self.declare_parameter('api_url', 'https://www.bbrobotics.in/api')
        self.declare_parameter('api_key', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('task_id', 'unknown')
        self.declare_parameter('storage_id', 'sqlite3')

        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('ft_sensor_topic', 'ft_sensor')

        self.bag_path = self.get_parameter('bag_path').get_parameter_value().string_value
        self.api_url = self.get_parameter('api_url').get_parameter_value().string_value
        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.task_id = self.get_parameter('task_id').get_parameter_value().string_value
        self.storage_id = self.get_parameter('storage_id').get_parameter_value().string_value
        self.joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.ft_sensor_topic = self.get_parameter('ft_sensor_topic').get_parameter_value().string_value

        if not all([self.bag_path, self.api_key, self.robot_id]):
            self.get_logger().error('bag_path, api_key, and robot_id are required')
            raise ValueError('Missing required parameters')

        self.headers = {'x-api-key': self.api_key, 'Content-Type': 'application/json'}

        # Process on startup
        self.create_timer(0.1, self.process_bag_once)
        self._processed = False

    def process_bag_once(self):
        if self._processed:
            return
        self._processed = True

        if not HAS_ROSBAG:
            self.get_logger().error('rosbag2_py not available — install ros-humble-rosbag2-py')
            return

        self.get_logger().info(f'Processing bag: {self.bag_path}')

        reader = SequentialReader()
        storage_options = StorageOptions(uri=self.bag_path, storage_id=self.storage_id)
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        reader.open(storage_options, converter_options)

        # Get topic type map
        topic_types = {}
        for topic_info in reader.get_all_topics_and_types():
            topic_types[topic_info.name] = topic_info.type

        observations = []
        actions = []
        start_ns = None
        end_ns = None

        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()

            if start_ns is None:
                start_ns = timestamp_ns
            end_ns = timestamp_ns

            ts = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc).isoformat()

            if topic == self.joint_states_topic or topic == f'/{self.joint_states_topic}':
                msg_type = get_message(topic_types[topic])
                msg = deserialize_message(data, msg_type)
                observations.append({
                    'timestamp': ts,
                    'joint_states': {
                        'names': list(msg.name),
                        'positions': list(msg.position),
                        'velocities': list(msg.velocity),
                        'efforts': list(msg.effort),
                    },
                    'sensor_data': {},
                })
            elif topic == self.ft_sensor_topic or topic == f'/{self.ft_sensor_topic}':
                msg_type = get_message(topic_types[topic])
                msg = deserialize_message(data, msg_type)
                # Append force/torque to the latest observation
                if observations:
                    observations[-1]['sensor_data']['force_torque'] = {
                        'force': {'x': msg.wrench.force.x, 'y': msg.wrench.force.y, 'z': msg.wrench.force.z},
                        'torque': {'x': msg.wrench.torque.x, 'y': msg.wrench.torque.y, 'z': msg.wrench.torque.z},
                    }
            elif topic in ['blackbox/task_event', '/blackbox/task_event']:
                msg_type = get_message(topic_types[topic])
                msg = deserialize_message(data, msg_type)
                try:
                    event = json.loads(msg.data)
                    if event.get('event') == 'action':
                        actions.append({
                            'timestamp': ts,
                            'action_type': event.get('action_type', 'unknown'),
                            'parameters': event.get('parameters', {}),
                        })
                except json.JSONDecodeError:
                    pass

        if start_ns is None:
            self.get_logger().warn('Bag is empty')
            return

        start_time = datetime.fromtimestamp(start_ns / 1e9, tz=timezone.utc).isoformat()
        end_time = datetime.fromtimestamp(end_ns / 1e9, tz=timezone.utc).isoformat()

        # Limit observations to avoid huge payloads (downsample if needed)
        max_obs = 500
        original_count = len(observations)
        if len(observations) > max_obs:
            indices = np.linspace(0, len(observations) - 1, max_obs, dtype=int)
            observations = [observations[i] for i in indices]

        episode = {
            'robot_id': self.robot_id,
            'task_id': self.task_id,
            'start_time': start_time,
            'end_time': end_time,
            'success': None,
            'metadata': {
                'source': 'rosbag2',
                'bag_path': self.bag_path,
                'original_observations': original_count,
            },
            'observations': observations,
            'actions': actions,
        }

        self.get_logger().info(
            f'Extracted episode: {len(observations)} observations, {len(actions)} actions'
        )

        try:
            resp = requests.post(
                f'{self.api_url}/episodes',
                json=episode,
                headers=self.headers,
                timeout=60,
            )
            if resp.status_code == 201:
                eid = resp.json().get('data', {}).get('id', 'unknown')
                self.get_logger().info(f'Episode created: {eid}')
            else:
                self.get_logger().error(f'API error: {resp.status_code} — {resp.text}')
        except requests.RequestException as e:
            self.get_logger().error(f'Push failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RosbagExporter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

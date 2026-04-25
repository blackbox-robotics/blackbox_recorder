"""
Black Box Robotics Episode Recorder — ROS 2 node that captures structured robot episodes
and pushes them to the Black Box Robotics API.

Subscribes to:
  - /joint_states (sensor_msgs/JointState)
  - /ft_sensor (geometry_msgs/WrenchStamped)
  - /gripper/state (std_msgs/Float64)
  - /camera/image_raw (sensor_msgs/Image) — optional, for frame counting
  - /blackbox/task_event (std_msgs/String) — JSON task start/end signals

Publishes:
  - /blackbox/episode_status (std_msgs/String) — episode recording state
"""

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64, String


class EpisodeRecorder(Node):
    """Records structured robot episodes and pushes to the Black Box Robotics backend."""

    BUFFER_PATH = Path('/tmp/blackbox_episode_buffer.json')

    def __init__(self):
        super().__init__('blackbox_episode_recorder')

        # Parameters
        self.declare_parameter('api_url', 'http://localhost:3001/api')
        self.declare_parameter('api_key', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('max_observations', 1000)
        self.declare_parameter('observation_interval_ms', 100)

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('ft_sensor_topic', '/ft_sensor')
        self.declare_parameter('gripper_topic', '/gripper/state')

        self.api_url = self.get_parameter('api_url').get_parameter_value().string_value
        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.max_obs = self.get_parameter('max_observations').get_parameter_value().integer_value
        self.obs_interval_ms = self.get_parameter('observation_interval_ms').get_parameter_value().integer_value
        self.joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.ft_sensor_topic = self.get_parameter('ft_sensor_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value

        if not self.api_key or not self.robot_id:
            self.get_logger().error('api_key and robot_id parameters are required')
            raise ValueError('Missing required parameters: api_key, robot_id')

        self.headers = {'x-api-key': self.api_key, 'Content-Type': 'application/json'}

        # State
        self.recording = False
        self.current_task_id: Optional[str] = None
        self.episode_start: Optional[str] = None
        self.observations: list = []
        self.actions: list = []
        self.metadata: dict = {}

        # Latest sensor values
        self.latest_joint_state: Optional[dict] = None
        self.latest_ft: Optional[dict] = None
        self.latest_gripper: Optional[float] = None
        self.last_obs_time: float = 0.0

        # Attempt to recover leftover buffer from a previous crash
        self._recover_buffer()

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, sensor_qos)
        self.create_subscription(WrenchStamped, self.ft_sensor_topic, self.ft_cb, sensor_qos)
        self.create_subscription(Float64, self.gripper_topic, self.gripper_cb, sensor_qos)
        self.create_subscription(String, '/blackbox/task_event', self.task_event_cb, 10)

        # Publisher
        self.status_pub = self.create_publisher(String, '/blackbox/episode_status', 10)

        # Observation collection timer
        interval_sec = self.obs_interval_ms / 1000.0
        self.create_timer(interval_sec, self.collect_observation)

        self.get_logger().info(
            f'Black Box Episode Recorder initialized — robot_id={self.robot_id}, '
            f'api={self.api_url}, interval={self.obs_interval_ms}ms | '
            f'topics: joints={self.joint_states_topic} ft={self.ft_sensor_topic} gripper={self.gripper_topic}'
        )

    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = {
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
        }

    def ft_cb(self, msg: WrenchStamped):
        self.latest_ft = {
            'force': {'x': msg.wrench.force.x, 'y': msg.wrench.force.y, 'z': msg.wrench.force.z},
            'torque': {'x': msg.wrench.torque.x, 'y': msg.wrench.torque.y, 'z': msg.wrench.torque.z},
        }

    def gripper_cb(self, msg: Float64):
        self.latest_gripper = msg.data

    def task_event_cb(self, msg: String):
        """Handle task start/end events.

        Expected JSON format:
          Start: {"event": "start", "task_id": "pick_and_place", "metadata": {...}}
          Action: {"event": "action", "action_type": "grasp", "parameters": {...}}
          End:   {"event": "end", "success": true}
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid JSON in task_event: {msg.data}')
            return

        event = data.get('event')

        if event == 'start':
            self.start_episode(data.get('task_id', 'unknown'), data.get('metadata', {}))
        elif event == 'action' and self.recording:
            self.actions.append({
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'action_type': data.get('action_type', 'unknown'),
                'parameters': data.get('parameters', {}),
            })
        elif event == 'end':
            self.end_episode(data.get('success'))

    def start_episode(self, task_id: str, metadata: dict):
        if self.recording:
            self.get_logger().warn('Already recording — ending previous episode')
            self.end_episode(success=None)

        self.recording = True
        self.current_task_id = task_id
        self.episode_start = datetime.now(timezone.utc).isoformat()
        self.observations = []
        self.actions = []
        self.metadata = metadata

        self.publish_status('recording', task_id)
        self.get_logger().info(f'Episode started — task={task_id}')

    def _ros_time_sec(self) -> float:
        """Return the current ROS clock time in seconds (float)."""
        return self.get_clock().now().nanoseconds / 1e9

    def collect_observation(self):
        if not self.recording:
            return
        if len(self.observations) >= self.max_obs:
            return

        now = self._ros_time_sec()
        if (now - self.last_obs_time) * 1000 < self.obs_interval_ms * 0.9:
            return
        self.last_obs_time = now

        obs = {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'joint_states': self.latest_joint_state or {},
            'sensor_data': {},
        }

        if self.latest_ft:
            obs['sensor_data']['force_torque'] = self.latest_ft
        if self.latest_gripper is not None:
            obs['sensor_data']['gripper_position'] = self.latest_gripper

        self.observations.append(obs)
        self._flush_buffer()

    def end_episode(self, success: Optional[bool]):
        if not self.recording:
            self.get_logger().warn('Not recording — ignoring end event')
            return

        self.recording = False
        end_time = datetime.now(timezone.utc).isoformat()

        episode_data = {
            'robot_id': self.robot_id,
            'task_id': self.current_task_id,
            'start_time': self.episode_start,
            'end_time': end_time,
            'success': success,
            'metadata': self.metadata,
            'observations': self.observations,
            'actions': self.actions,
        }

        self.get_logger().info(
            f'Episode ended — task={self.current_task_id}, '
            f'observations={len(self.observations)}, actions={len(self.actions)}, '
            f'success={success}'
        )

        self.push_episode(episode_data)
        self.publish_status('idle', self.current_task_id or '')

    # ------------------------------------------------------------------
    # Local JSON buffer for crash recovery (RPN 360)
    # ------------------------------------------------------------------

    def _flush_buffer(self):
        """Write current in-progress episode data to the local buffer file."""
        try:
            buf = {
                'robot_id': self.robot_id,
                'task_id': self.current_task_id,
                'start_time': self.episode_start,
                'metadata': self.metadata,
                'observations': self.observations,
                'actions': self.actions,
            }
            tmp = self.BUFFER_PATH.with_suffix('.tmp')
            tmp.write_text(json.dumps(buf))
            tmp.replace(self.BUFFER_PATH)
        except OSError as e:
            self.get_logger().warn(f'Failed to write episode buffer: {e}')

    def _delete_buffer(self):
        """Remove the local buffer file after a successful upload."""
        try:
            self.BUFFER_PATH.unlink(missing_ok=True)
        except OSError:
            pass

    def _recover_buffer(self):
        """On startup, try to re-upload a leftover buffer from a previous crash."""
        if not self.BUFFER_PATH.exists():
            return
        self.get_logger().info('Found leftover episode buffer — attempting re-upload')
        try:
            data = json.loads(self.BUFFER_PATH.read_text())
            # Fill in end_time as unknown since the previous run crashed
            data.setdefault('end_time', None)
            data.setdefault('success', None)
            self._upload_episode(data, from_recovery=True)
        except (json.JSONDecodeError, OSError) as e:
            self.get_logger().warn(f'Could not read leftover buffer: {e}')

    # ------------------------------------------------------------------
    # Upload helpers
    # ------------------------------------------------------------------

    def push_episode(self, data: dict):
        """Push completed episode to the Black Box Robotics API."""
        # Ensure the final state is flushed to disk before uploading
        self._flush_buffer()
        self._upload_episode(data, from_recovery=False)

    def _upload_episode(self, data: dict, from_recovery: bool):
        """Attempt to POST episode data. Manages buffer file on success/failure."""
        try:
            resp = requests.post(
                f'{self.api_url}/episodes',
                json=data,
                headers=self.headers,
                timeout=30,
            )
            if resp.status_code == 201:
                episode_id = resp.json().get('data', {}).get('id', 'unknown')
                self.get_logger().info(f'Episode pushed successfully — id={episode_id}')
                self._delete_buffer()
            else:
                self.get_logger().error(
                    f'Failed to push episode: {resp.status_code} — {resp.text}'
                )
                self.get_logger().warn(
                    f'Episode buffer retained at {self.BUFFER_PATH} for recovery'
                )
        except requests.RequestException as e:
            self.get_logger().error(f'Failed to push episode: {e}')
            self.get_logger().warn(
                f'Episode buffer retained at {self.BUFFER_PATH} for recovery'
            )

    def publish_status(self, state: str, task_id: str):
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'task_id': task_id,
            'robot_id': self.robot_id,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EpisodeRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

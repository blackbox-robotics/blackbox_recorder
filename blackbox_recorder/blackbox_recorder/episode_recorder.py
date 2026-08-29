"""
Black Box Robotics Episode Recorder — ROS 2 node that captures structured robot episodes
and pushes them to the Black Box Robotics API.

Subscribes to:
  - joint_states (sensor_msgs/JointState)
  - ft_sensor (geometry_msgs/WrenchStamped)
  - gripper/state (std_msgs/Float64)
  - any topics listed in the extra_float_topics parameter (std_msgs/Float64) —
    for scalar sensors that don't fit the three built-in types (temperature,
    battery voltage, custom pressure/current sensors, etc.)
  - blackbox/task_event (std_msgs/String) — JSON task start/end signals

Publishes:
  - blackbox/episode_status (std_msgs/String) — episode recording state
"""

import json
import os
import queue
import threading
import uuid
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

from blackbox_recorder.offline_export import export_offline_session


class EpisodeRecorder(Node):
    """Records structured robot episodes and pushes to the Black Box Robotics backend."""

    BUFFER_PATH = Path('/tmp/blackbox_episode_buffer.json')

    def __init__(self):
        super().__init__('blackbox_episode_recorder')

        # Parameters
        self.declare_parameter('api_url', 'https://www.bbrobotics.in/api')
        self.declare_parameter('api_key', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('max_observations', 1000)
        self.declare_parameter('observation_interval_ms', 100)
        self.declare_parameter('offline_mode', False)
        self.declare_parameter('export_dir', os.path.expanduser('~/.blackbox/exports'))

        # Stream each observation to the dashboard as it's collected instead of
        # only uploading the whole episode at the end — real-time graphing
        # while a session is still recording. Additive: buffering + the
        # end-of-episode path below are unchanged and remain the fallback
        # whenever this can't reach the backend. Meaningless (and forced off)
        # under offline_mode, which has no backend to stream to.
        self.declare_parameter('live_stream_enabled', True)

        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('ft_sensor_topic', 'ft_sensor')
        self.declare_parameter('gripper_topic', 'gripper/state')

        # Extra scalar (std_msgs/Float64) sensor topics beyond the three built-in
        # ones. Format: "topic1:field_name1,topic2:field_name2" — e.g.
        # "/motor_temp:temperature,/battery/voltage:battery_voltage". Each field
        # shows up under that name in every observation's sensor_data. Empty by
        # default — no behavior change if unset.
        self.declare_parameter('extra_float_topics', '')

        self.api_url = self.get_parameter('api_url').get_parameter_value().string_value
        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.max_obs = self.get_parameter('max_observations').get_parameter_value().integer_value
        self.obs_interval_ms = self.get_parameter('observation_interval_ms').get_parameter_value().integer_value
        self.offline_mode = self.get_parameter('offline_mode').get_parameter_value().bool_value
        self.export_dir = self.get_parameter('export_dir').get_parameter_value().string_value
        self.live_stream_enabled = (
            self.get_parameter('live_stream_enabled').get_parameter_value().bool_value
            and not self.offline_mode
        )
        self.joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.ft_sensor_topic = self.get_parameter('ft_sensor_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        extra_float_topics_raw = self.get_parameter('extra_float_topics').get_parameter_value().string_value

        # api_key isn't meaningful in offline mode — nothing gets POSTed
        if not self.robot_id or (not self.offline_mode and not self.api_key):
            self.get_logger().error('robot_id is required (api_key also required unless offline_mode:=true)')
            raise ValueError('Missing required parameters')

        # robot_id must be the robot's dashboard UUID (Settings > Robots), not a
        # friendly name — the backend validates it as a UUID and rejects anything
        # else at upload time, so a bad value here silently wastes an entire
        # recording session before the operator finds out.
        try:
            uuid.UUID(self.robot_id)
        except ValueError:
            self.get_logger().error(
                f"robot_id '{self.robot_id}' is not a valid UUID — copy the robot's id from "
                'the dashboard (Settings > Robots), not a friendly name/slug'
            )
            raise ValueError('robot_id must be a valid UUID')

        self.headers = {'x-api-key': self.api_key, 'Content-Type': 'application/json'}

        # Parse extra_float_topics into (topic, field_name) pairs. A malformed
        # entry is logged and skipped rather than crashing the whole node —
        # one typo in an extra sensor shouldn't take down episode recording.
        self.extra_float_topics: list = []
        for entry in extra_float_topics_raw.split(','):
            entry = entry.strip()
            if not entry:
                continue
            if ':' not in entry:
                self.get_logger().warn(
                    f'Skipping malformed extra_float_topics entry (expected '
                    f'"topic:field_name"): {entry!r}'
                )
                continue
            topic, field_name = entry.split(':', 1)
            topic, field_name = topic.strip(), field_name.strip()
            if not topic or not field_name:
                self.get_logger().warn(f'Skipping malformed extra_float_topics entry: {entry!r}')
                continue
            self.extra_float_topics.append((topic, field_name))

        # State
        self.recording = False
        self.current_task_id: Optional[str] = None
        self.episode_start: Optional[str] = None
        self.observations: list = []
        self.actions: list = []
        self.metadata: dict = {}

        # Live-streaming: set once /episodes/start succeeds for the current
        # episode, cleared on end. None means "not streaming this episode" —
        # either live_stream_enabled is off, or the start call failed — in
        # which case behavior is identical to before this feature existed.
        self.live_episode_id: Optional[str] = None

        # Observation POSTs run on a background thread so a slow/dead network
        # can never stall the observation-collection timer. Bounded and
        # drop-oldest under overload — the local buffer flushed in
        # collect_observation() is the durable copy regardless of whether any
        # of this succeeds.
        self._live_queue: "queue.Queue" = queue.Queue(maxsize=200)
        self._live_worker = threading.Thread(target=self._live_worker_loop, daemon=True)
        self._live_worker.start()

        # Latest sensor values
        self.latest_joint_state: Optional[dict] = None
        self.latest_ft: Optional[dict] = None
        self.latest_gripper: Optional[float] = None
        self.latest_extra: dict = {}
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
        for topic, field_name in self.extra_float_topics:
            self.create_subscription(
                Float64, topic, self._make_extra_float_cb(field_name), sensor_qos
            )
        self.create_subscription(String, 'blackbox/task_event', self.task_event_cb, 10)

        # Publisher
        self.status_pub = self.create_publisher(String, 'blackbox/episode_status', 10)

        # Observation collection timer
        interval_sec = self.obs_interval_ms / 1000.0
        self.create_timer(interval_sec, self.collect_observation)

        extra_topics_str = ', '.join(f'{t}->{f}' for t, f in self.extra_float_topics) or 'none'
        self.get_logger().info(
            f'Black Box Episode Recorder initialized — robot_id={self.robot_id}, '
            f'api={self.api_url}, interval={self.obs_interval_ms}ms, '
            f'live_stream={"on" if self.live_stream_enabled else "off"} | '
            f'topics: joints={self.joint_states_topic} ft={self.ft_sensor_topic} '
            f'gripper={self.gripper_topic} extra=[{extra_topics_str}]'
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

    def _make_extra_float_cb(self, field_name: str):
        """Build a callback that stores an extra Float64 topic's value under field_name."""
        def _cb(msg: Float64):
            self.latest_extra[field_name] = msg.data
        return _cb

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
            action = {
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'action_type': data.get('action_type', 'unknown'),
                'parameters': data.get('parameters', {}),
            }
            self.actions.append(action)
            if self.live_episode_id:
                self._post_live_action(self.live_episode_id, action)
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
        self.live_episode_id = None

        if self.live_stream_enabled:
            self.live_episode_id = self._start_live_episode(task_id, self.episode_start)

        self.publish_status('recording', task_id)
        if self.live_episode_id:
            self.get_logger().info(f'Episode started — task={task_id}, live_episode_id={self.live_episode_id}')
        else:
            self.get_logger().info(f'Episode started — task={task_id} (buffering locally, no live stream)')

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
        if self.latest_extra:
            obs['sensor_data'].update(self.latest_extra)

        self.observations.append(obs)
        self._flush_buffer()

        if self.live_episode_id:
            self._queue_live_observation(self.live_episode_id, obs)

    def end_episode(self, success: Optional[bool]):
        if not self.recording:
            self.get_logger().warn('Not recording — ignoring end event')
            return

        self.recording = False
        end_time = datetime.now(timezone.utc).isoformat()
        live_episode_id = self.live_episode_id
        self.live_episode_id = None

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

        if live_episode_id:
            # Observations/actions already streamed in one at a time — finish
            # the existing episode rather than uploading it again, which
            # would duplicate everything that already made it live.
            if self._finish_live_episode(live_episode_id, end_time, success):
                self._delete_buffer()
            else:
                self.get_logger().error(
                    f'Could not finish live episode {live_episode_id} — it will stay shown as '
                    f'RECORDING on the dashboard until closed manually '
                    f'(PATCH {self.api_url}/episodes/{live_episode_id}/finish)'
                )
        else:
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
                # Recorded so a crash-recovered buffer can tell _recover_buffer
                # whether this episode already has a row (and observations)
                # server-side — see _recover_buffer for why that matters.
                'live_episode_id': self.live_episode_id,
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
        self.get_logger().info('Found leftover episode buffer — attempting recovery')
        try:
            data = json.loads(self.BUFFER_PATH.read_text())
        except (json.JSONDecodeError, OSError) as e:
            self.get_logger().warn(f'Could not read leftover buffer: {e}')
            return

        live_episode_id = data.pop('live_episode_id', None)
        if live_episode_id:
            # This episode's observations/actions already made it to the
            # server one at a time before the crash — only the finish call
            # was missed. Re-uploading the buffer as a new episode would
            # duplicate every one of them, so just close out the existing
            # episode instead of going through _upload_episode at all.
            self.get_logger().info(
                f'Buffer belongs to already-live-streamed episode {live_episode_id} — '
                f'finishing it, not re-uploading'
            )
            # end_time is unknown (crashed before the end event) — best
            # approximation is recovery time, not the actual session end.
            recovered_end_time = datetime.now(timezone.utc).isoformat()
            if self._finish_live_episode(live_episode_id, recovered_end_time, None):
                self._delete_buffer()
            else:
                self.get_logger().error(
                    f'Could not finish live episode {live_episode_id} after recovery — it will '
                    f'stay shown as RECORDING until closed manually '
                    f'(PATCH {self.api_url}/episodes/{live_episode_id}/finish)'
                )
            return

        # No live episode was ever started for this buffer — same recovery
        # path as before this feature existed.
        data.setdefault('end_time', None)
        data.setdefault('success', None)
        self._upload_episode(data, from_recovery=True)

    # ------------------------------------------------------------------
    # Upload helpers
    # ------------------------------------------------------------------

    def push_episode(self, data: dict):
        """Push completed episode to the Black Box Robotics API."""
        # Ensure the final state is flushed to disk before uploading
        self._flush_buffer()
        self._upload_episode(data, from_recovery=False)

    def _upload_episode(self, data: dict, from_recovery: bool):
        """
        POSTs episode data, unless offline_mode is set. On any failure (or in
        offline_mode), falls back to a durable session zip via the same
        offline_export module rosbag_exporter uses — same pipeline either way,
        network or none. This replaces the old /tmp-buffer-only fallback: a
        zip survives a power-cycle and can be physically moved off the drone,
        which a /tmp file cannot.
        """
        if not self.offline_mode:
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
                    return
                self.get_logger().error(
                    f'Failed to push episode: {resp.status_code} — {resp.text}. Falling back to local export.'
                )
            except requests.RequestException as e:
                self.get_logger().error(f'Failed to push episode: {e}. Falling back to local export.')

        zip_path, session_id = export_offline_session(data, self.export_dir)
        self.get_logger().info(f'Offline session exported: {zip_path} (session_id={session_id})')
        self._delete_buffer()

    # ------------------------------------------------------------------
    # Live streaming — best-effort, additive. Every method here fails
    # silently into "keep using the local buffer" rather than ever raising;
    # a dead network must never interrupt recording.
    # ------------------------------------------------------------------

    def _start_live_episode(self, task_id: str, start_time: str) -> Optional[str]:
        """POST /episodes/start. Short timeout — this runs once per episode
        on the ROS callback thread, not the hot observation-collection path,
        so a brief block here is acceptable."""
        try:
            resp = requests.post(
                f'{self.api_url}/episodes/start',
                json={'robot_id': self.robot_id, 'task_id': task_id, 'start_time': start_time},
                headers=self.headers,
                timeout=3,
            )
            if resp.status_code == 201:
                return resp.json().get('data', {}).get('id')
            self.get_logger().warn(
                f'Live episode start failed ({resp.status_code}) — this episode will only '
                f'appear on the dashboard once it ends'
            )
        except requests.RequestException as e:
            self.get_logger().warn(
                f'Live episode start failed ({e}) — this episode will only appear on the '
                f'dashboard once it ends'
            )
        return None

    def _finish_live_episode(self, episode_id: str, end_time: str, success: Optional[bool]) -> bool:
        """PATCH /episodes/:id/finish. Returns True on success."""
        try:
            resp = requests.patch(
                f'{self.api_url}/episodes/{episode_id}/finish',
                json={'end_time': end_time, 'success': success},
                headers=self.headers,
                timeout=5,
            )
            if resp.status_code == 200:
                self.get_logger().info(f'Live episode finished — id={episode_id}')
                return True
            self.get_logger().error(f'Failed to finish live episode {episode_id}: {resp.status_code} — {resp.text}')
        except requests.RequestException as e:
            self.get_logger().error(f'Failed to finish live episode {episode_id}: {e}')
        return False

    def _queue_live_observation(self, episode_id: str, obs: dict):
        """Non-blocking — hands off to the background worker thread. Drops
        the oldest queued point rather than blocking if the backend can't
        keep up; _flush_buffer() already made this observation durable
        locally regardless of whether it ever makes it live."""
        item = (episode_id, obs)
        try:
            self._live_queue.put_nowait(item)
        except queue.Full:
            try:
                self._live_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._live_queue.put_nowait(item)
            except queue.Full:
                pass

    def _live_worker_loop(self):
        """Runs for the lifetime of the node. One POST per queued observation;
        failures are logged at debug level and dropped — never retried, since
        a retry queue for a per-observation stream isn't worth the complexity
        the local buffer already gives every observation a durable copy."""
        while True:
            episode_id, obs = self._live_queue.get()
            try:
                requests.post(
                    f'{self.api_url}/episodes/{episode_id}/observations',
                    json=obs,
                    headers=self.headers,
                    timeout=5,
                )
            except requests.RequestException as e:
                self.get_logger().debug(f'Live observation POST failed (non-fatal): {e}')
            self._live_queue.task_done()

    def _post_live_action(self, episode_id: str, action: dict):
        """Actions are rare (task-level events, not per-tick sensor data) so
        a direct short-timeout call is fine — no need for the background
        queue collect_observation() uses."""
        try:
            requests.post(
                f'{self.api_url}/episodes/{episode_id}/actions',
                json=action,
                headers=self.headers,
                timeout=3,
            )
        except requests.RequestException as e:
            self.get_logger().warn(f'Live action POST failed (non-fatal, action stays in local buffer): {e}')

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

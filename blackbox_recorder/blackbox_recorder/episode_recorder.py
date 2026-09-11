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
import time
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

        # MAVLink upload: for a vehicle whose only live channel is the
        # MAVLink radio, with no separate internet path of its own. Recording
        # is completely unchanged — the recorder always writes the same
        # session zip via export_offline_session() it would in offline_mode.
        # The difference is what happens to that zip next: instead of sitting
        # in export_dir until someone physically retrieves it, a background
        # worker pushes it out over the SAME MAVLink link (MAVLink FTP write
        # — https://mavlink.io/en/services/ftp.html) to a ground-station
        # receiver (mavlink_upload_receiver.py) that imports it as it
        # arrives. Requires pymavlink — not a hard dependency of the package,
        # only imported if this is enabled.
        self.declare_parameter('mavlink_upload_enabled', False)
        self.declare_parameter('mavlink_connection', '')
        self.declare_parameter('mavlink_source_system', 42)
        # How often, while an episode is open, to push whatever's accumulated
        # since the last push — the MAVLink-transport equivalent of
        # live_stream_enabled's per-observation HTTP POSTs. MAVLink FTP is a
        # whole-file transfer with real per-transfer overhead, so this batches
        # into small periodic files rather than one file per observation.
        self.declare_parameter('mavlink_batch_interval_s', 2.0)

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
        self.mavlink_upload_enabled = self.get_parameter('mavlink_upload_enabled').get_parameter_value().bool_value
        self.mavlink_connection = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        self.mavlink_source_system = self.get_parameter('mavlink_source_system').get_parameter_value().integer_value
        self.mavlink_batch_interval_s = self.get_parameter('mavlink_batch_interval_s').get_parameter_value().double_value
        # Both mean "this vehicle has no direct path to the API" — offline_mode
        # leaves the zip for physical retrieval, mavlink_upload_enabled pushes
        # it out over the radio instead. Either way, never attempt HTTP.
        self.no_internet = self.offline_mode or self.mavlink_upload_enabled
        self.live_stream_enabled = (
            self.get_parameter('live_stream_enabled').get_parameter_value().bool_value
            and not self.no_internet
        )
        self.joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.ft_sensor_topic = self.get_parameter('ft_sensor_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        extra_float_topics_raw = self.get_parameter('extra_float_topics').get_parameter_value().string_value

        # api_key isn't meaningful when this vehicle never calls the API directly
        if not self.robot_id or (not self.no_internet and not self.api_key):
            self.get_logger().error(
                'robot_id is required (api_key also required unless offline_mode:=true or mavlink_upload_enabled:=true)'
            )
            raise ValueError('Missing required parameters')

        if self.mavlink_upload_enabled and not self.mavlink_connection:
            self.get_logger().error(
                'mavlink_upload_enabled is true but mavlink_connection is empty — set it to the '
                "link this companion computer uses to reach the ground station, e.g. "
                "'udpout:<ground-ip>:14552' or a serial device"
            )
            raise ValueError('mavlink_connection is required when mavlink_upload_enabled is true')

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

        # MAVLink upload worker: carries both the live batch files below and
        # the full end-of-episode zip (crash-recovery case only — see
        # _upload_episode). Unbounded — these are small, infrequent files,
        # not per-observation, so no drop-oldest pressure like the live
        # queue above. Single-threaded, so uploads happen strictly in order.
        self._mavlink_upload_queue: "queue.Queue" = queue.Queue()
        if self.mavlink_upload_enabled:
            self._mavlink_upload_worker = threading.Thread(target=self._mavlink_upload_worker_loop, daemon=True)
            self._mavlink_upload_worker.start()

        # MAVLink live batching: the MAVLink-transport equivalent of
        # live_stream_enabled's per-observation HTTP POSTs (see
        # _flush_mavlink_batch). mavlink_session_id is generated locally in
        # start_episode() — unlike live_episode_id, there's no server call
        # to get an ID from, since this vehicle has no direct API path.
        self.mavlink_session_id: Optional[str] = None
        self._mavlink_pending_observations: list = []
        self._mavlink_pending_actions: list = []
        self._mavlink_batch_seq: int = 0
        self._mavlink_batch_dir = os.path.join(self.export_dir, '.mavlink_batches')

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

        if self.mavlink_upload_enabled:
            self.create_timer(self.mavlink_batch_interval_s, self._flush_mavlink_batch)

        extra_topics_str = ', '.join(f'{t}->{f}' for t, f in self.extra_float_topics) or 'none'
        self.get_logger().info(
            f'Black Box Episode Recorder initialized — robot_id={self.robot_id}, '
            f'api={self.api_url}, interval={self.obs_interval_ms}ms, '
            f'live_stream={"on" if self.live_stream_enabled else "off"}, '
            f'mavlink_upload={"on (" + self.mavlink_connection + ")" if self.mavlink_upload_enabled else "off"} | '
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
            if self.mavlink_session_id:
                self._mavlink_pending_actions.append(action)
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

        if self.mavlink_upload_enabled:
            self.mavlink_session_id = str(uuid.uuid4())
            self._mavlink_pending_observations = []
            self._mavlink_pending_actions = []
            self._mavlink_batch_seq = 0

        self.publish_status('recording', task_id)
        if self.live_episode_id:
            self.get_logger().info(f'Episode started — task={task_id}, live_episode_id={self.live_episode_id}')
        elif self.mavlink_session_id:
            self.get_logger().info(f'Episode started — task={task_id}, mavlink_session_id={self.mavlink_session_id}')
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
        if self.mavlink_session_id:
            self._mavlink_pending_observations.append(obs)

    def end_episode(self, success: Optional[bool]):
        if not self.recording:
            self.get_logger().warn('Not recording — ignoring end event')
            return

        self.recording = False
        end_time = datetime.now(timezone.utc).isoformat()
        live_episode_id = self.live_episode_id
        self.live_episode_id = None
        # mavlink_session_id is intentionally NOT cleared yet — the final
        # _flush_mavlink_batch call below needs it (see the elif branch),
        # and clears it itself once that batch is queued.
        mavlink_session_id = self.mavlink_session_id

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
        elif mavlink_session_id:
            # Same idea, over MAVLink: send the closing batch (any remaining
            # pending observations/actions, plus end_time/success) so the
            # ground receiver finishes the episode it opened on the first
            # batch, then keep the local zip as a backup only — pushing it
            # too would duplicate everything already delivered live.
            self._flush_mavlink_batch(is_final=True, end_time=end_time, success=success)
            self.push_episode(episode_data, already_live_batched=True)
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
                'mavlink_session_id': self.mavlink_session_id,
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

        mavlink_session_id = data.pop('mavlink_session_id', None)
        if mavlink_session_id and self.mavlink_upload_enabled:
            # Same idea as live_episode_id above, over MAVLink: the ground
            # receiver already opened this episode from earlier batches, so
            # re-uploading the full buffer would duplicate them. Send only a
            # closing batch — any observations collected between the last
            # periodic flush and the crash are lost (bounded by
            # mavlink_batch_interval_s), but nothing already delivered is
            # duplicated, and the episode doesn't get stuck open forever.
            self.get_logger().info(
                f'Buffer belongs to already-live-batched MAVLink session {mavlink_session_id} — '
                f'sending closing batch only, not re-uploading'
            )
            recovered_end_time = datetime.now(timezone.utc).isoformat()
            try:
                os.makedirs(self._mavlink_batch_dir, exist_ok=True)
                batch_path = os.path.join(self._mavlink_batch_dir, f'livebatch_{mavlink_session_id}_recovery.json')
                with open(batch_path, 'w') as f:
                    json.dump({
                        'session_id': mavlink_session_id,
                        'robot_id': data.get('robot_id'),
                        'task_id': data.get('task_id'),
                        'start_time': data.get('start_time'),
                        'metadata': data.get('metadata', {}),
                        'seq': 0,
                        'is_final': True,
                        'observations': [],
                        'actions': [],
                        'end_time': recovered_end_time,
                        'success': None,
                    }, f)
                self._mavlink_upload_queue.put(batch_path)
            except OSError as e:
                self.get_logger().error(f'Failed to write MAVLink recovery closing batch: {e}')
            self._delete_buffer()
            return

        # No live episode was ever started for this buffer — same recovery
        # path as before this feature existed.
        data.setdefault('end_time', None)
        data.setdefault('success', None)
        self._upload_episode(data, from_recovery=True)

    # ------------------------------------------------------------------
    # Upload helpers
    # ------------------------------------------------------------------

    def push_episode(self, data: dict, already_live_batched: bool = False):
        """Push completed episode to the Black Box Robotics API."""
        # Ensure the final state is flushed to disk before uploading
        self._flush_buffer()
        self._upload_episode(data, from_recovery=False, already_live_batched=already_live_batched)

    def _upload_episode(self, data: dict, from_recovery: bool, already_live_batched: bool = False):
        """
        POSTs episode data, unless offline_mode or mavlink_upload_enabled is
        set. On any failure (or when this vehicle has no direct API path at
        all), falls back to a durable session zip via the same offline_export
        module rosbag_exporter uses — same pipeline either way. This replaces
        the old /tmp-buffer-only fallback: a zip survives a power-cycle and
        can be physically moved off the drone, which a /tmp file cannot.

        already_live_batched is True when this episode was already fully
        delivered via periodic MAVLink batches during recording (see
        _flush_mavlink_batch / end_episode) — in that case the zip is kept
        purely as a local durability backup, not re-pushed, since pushing it
        too would create a second, duplicate episode server-side. It's only
        False here for the crash-recovery path, where no live batching
        session existed for the leftover buffer (the node had just
        restarted) — that's the one case a full-zip MAVLink push is still
        the only way to deliver it.
        """
        if not self.no_internet:
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

        if self.mavlink_upload_enabled and not already_live_batched:
            self._mavlink_upload_queue.put(zip_path)
            self.get_logger().info(f'Queued for MAVLink upload: {zip_path}')
        elif self.mavlink_upload_enabled:
            self.get_logger().info(f'Already delivered via MAVLink live batching — {zip_path} kept as local backup only')

    # ------------------------------------------------------------------
    # MAVLink upload — pushes session zips to a ground-station receiver
    # (mavlink_upload_receiver.py) over MAVLink FTP, for a vehicle with no
    # other path to the API. Runs entirely on its own background thread;
    # never touches the ROS timer/observation-collection path. The zip
    # already exists on local storage (export_offline_session, above) before
    # this ever runs, so a failed or interrupted push never loses data — it
    # just means the zip is still sitting on the vehicle for the next retry
    # or physical retrieval, exactly like a pure offline_mode deployment.
    # ------------------------------------------------------------------

    def _mavlink_upload_worker_loop(self):
        """Runs for the lifetime of the node. Maintains one MAVLink FTP
        connection, reconnecting on failure, and pushes queued zips in
        order. A failed push is requeued with a backoff rather than dropped
        — the zip isn't going anywhere, so it's always worth trying again
        once the link (or the ground receiver) comes back."""
        try:
            from pymavlink import mavutil
            from pymavlink.mavftp import MAVFTP, MAVFTPSettings
        except ImportError:
            self.get_logger().error(
                'mavlink_upload_enabled is true but pymavlink is not installed — '
                'pip install pymavlink. MAVLink upload disabled; zips will accumulate '
                f'in {self.export_dir} for manual/physical retrieval.'
            )
            return

        ftp = None
        while True:
            zip_path = self._mavlink_upload_queue.get()
            filename = os.path.basename(zip_path)

            if ftp is None:
                try:
                    self.get_logger().info(f'Connecting to MAVLink ground receiver ({self.mavlink_connection})...')
                    master = mavutil.mavlink_connection(self.mavlink_connection, source_system=self.mavlink_source_system)
                    # A udpin (server) socket only replies to addresses it has
                    # already heard from — a purely receive-only wait here
                    # would deadlock against a receiver that's also only
                    # replying. Broadcast our own heartbeat while waiting so
                    # the server has something to reply to.
                    deadline = time.time() + 30
                    hb = None
                    while time.time() < deadline and hb is None:
                        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                        hb = master.wait_heartbeat(timeout=1)
                    if hb is None:
                        raise TimeoutError(f'No heartbeat from MAVLink ground receiver on {self.mavlink_connection} after 30s')
                    ftp = MAVFTP(
                        master,
                        target_system=master.target_system,
                        target_component=master.target_component,
                        settings=MAVFTPSettings([
                            ('debug', int, 0), ('pkt_loss_tx', int, 0), ('pkt_loss_rx', int, 0),
                            ('max_backlog', int, 5), ('burst_read_size', int, 80),
                            ('write_size', int, 80), ('write_qsize', int, 5),
                            ('idle_detection_time', float, 3.7), ('read_retry_time', float, 1.0),
                            ('retry_time', float, 0.5),
                        ]),
                    )
                except Exception as e:  # noqa: BLE001 — link may not be up yet; retry, never crash the node
                    self.get_logger().warn(f'MAVLink connection failed ({e}) — will retry')
                    ftp = None
                    self._mavlink_upload_queue.put(zip_path)
                    time.sleep(10)
                    continue

            try:
                ftp.cmd_put([zip_path, filename])
                ret = ftp.process_ftp_reply('CreateFile', timeout=60)
                if ret.error_code:
                    raise RuntimeError(str(ret))
                self.get_logger().info(f'MAVLink upload complete: {filename}')
            except Exception as e:  # noqa: BLE001 — one failed push must not kill the worker
                self.get_logger().warn(f'MAVLink upload failed for {filename} ({e}) — will retry')
                ftp = None  # connection state is unknown after a failure — reconnect clean next time
                self._mavlink_upload_queue.put(zip_path)
                time.sleep(10)

    def _flush_mavlink_batch(self, is_final: bool = False, end_time: Optional[str] = None, success: Optional[bool] = None):
        """Push whatever's accumulated in the pending observation/action
        lists since the last flush, as one small file — the MAVLink-transport
        equivalent of live_stream_enabled's per-observation HTTP POSTs.
        Called on a timer (mavlink_batch_interval_s) while an episode is
        open, and once more from end_episode with is_final=True to close it.

        The receiver (mavlink_upload_receiver.py) opens a live episode on
        the first batch it sees for a session_id and finishes it on
        is_final — same lifecycle as _start_live_episode/_finish_live_episode
        above, just relayed through MAVLink batches instead of the recorder
        calling those endpoints directly (it can't — no direct API path)."""
        if not self.mavlink_session_id:
            return  # not recording, or not in mavlink_upload_enabled mode

        # Skip empty non-final batches — nothing new since last flush, no
        # point spending a whole FTP transfer on it. Always send the final
        # one even if empty, since that's what signals "episode is done."
        if not is_final and not self._mavlink_pending_observations and not self._mavlink_pending_actions:
            return

        batch = {
            'session_id': self.mavlink_session_id,
            'robot_id': self.robot_id,
            'task_id': self.current_task_id,
            'start_time': self.episode_start,
            'metadata': self.metadata,
            'seq': self._mavlink_batch_seq,
            'is_final': is_final,
            'observations': self._mavlink_pending_observations,
            'actions': self._mavlink_pending_actions,
        }
        if is_final:
            batch['end_time'] = end_time
            batch['success'] = success

        self._mavlink_pending_observations = []
        self._mavlink_pending_actions = []

        try:
            os.makedirs(self._mavlink_batch_dir, exist_ok=True)
            batch_path = os.path.join(
                self._mavlink_batch_dir,
                f'livebatch_{self.mavlink_session_id}_{self._mavlink_batch_seq:04d}.json',
            )
            with open(batch_path, 'w') as f:
                json.dump(batch, f)
        except OSError as e:
            self.get_logger().warn(f'Failed to write MAVLink batch file: {e}')
            return

        self._mavlink_upload_queue.put(batch_path)
        self._mavlink_batch_seq += 1

        if is_final:
            self.get_logger().info(f'Queued final MAVLink batch for session {self.mavlink_session_id}')
            self.mavlink_session_id = None

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

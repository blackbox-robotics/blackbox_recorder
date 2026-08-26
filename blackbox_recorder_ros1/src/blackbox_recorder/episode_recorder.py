"""
Black Box Robotics Episode Recorder — ROS 1 node that captures structured robot episodes
and pushes them to the Black Box Robotics API.

ROS 1 port of the ROS 2 episode_recorder.py — same business logic (episode lifecycle,
crash-recovery buffer, HTTP upload), rebuilt on rospy instead of rclpy. Message types
(sensor_msgs/JointState, geometry_msgs/WrenchStamped, std_msgs/Float64, std_msgs/String)
are identical between ROS 1 and ROS 2, so those imports and field accesses are unchanged.

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
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import requests
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64, String

from blackbox_recorder.offline_export import export_offline_session


class EpisodeRecorder(object):
    """Records structured robot episodes and pushes to the Black Box Robotics backend."""

    BUFFER_PATH = Path('/tmp/blackbox_episode_buffer.json')

    def __init__(self):
        rospy.init_node('blackbox_episode_recorder')

        # Parameters — private (~) params, resolved relative to this node's namespace.
        # rospy.get_param returns the native Python type directly; no declare step
        # and no wrapper object, unlike rclpy.
        self.api_url = rospy.get_param('~api_url', 'https://www.bbrobotics.in/api')
        self.api_key = rospy.get_param('~api_key', '')
        self.robot_id = rospy.get_param('~robot_id', '')
        self.max_obs = int(rospy.get_param('~max_observations', 1000))
        self.obs_interval_ms = int(rospy.get_param('~observation_interval_ms', 100))
        self.offline_mode = bool(rospy.get_param('~offline_mode', False))
        self.export_dir = rospy.get_param('~export_dir', os.path.expanduser('~/.blackbox/exports'))

        self.joint_states_topic = rospy.get_param('~joint_states_topic', 'joint_states')
        self.ft_sensor_topic = rospy.get_param('~ft_sensor_topic', 'ft_sensor')
        self.gripper_topic = rospy.get_param('~gripper_topic', 'gripper/state')

        # Extra scalar (std_msgs/Float64) sensor topics beyond the three built-in
        # ones. Format: "topic1:field_name1,topic2:field_name2" — e.g.
        # "/motor_temp:temperature,/battery/voltage:battery_voltage". Each field
        # shows up under that name in every observation's sensor_data. Empty by
        # default — no behavior change if unset.
        extra_float_topics_raw = rospy.get_param('~extra_float_topics', '')

        # api_key isn't meaningful in offline mode — nothing gets POSTed
        if not self.robot_id or (not self.offline_mode and not self.api_key):
            rospy.logerr('robot_id is required (api_key also required unless offline_mode:=true)')
            raise ValueError('Missing required parameters')

        # robot_id must be the robot's dashboard UUID (Settings > Robots), not a
        # friendly name — the backend validates it as a UUID and rejects anything
        # else at upload time, so a bad value here silently wastes an entire
        # recording session before the operator finds out.
        try:
            uuid.UUID(self.robot_id)
        except ValueError:
            rospy.logerr(
                "robot_id '%s' is not a valid UUID — copy the robot's id from "
                'the dashboard (Settings > Robots), not a friendly name/slug' % self.robot_id
            )
            raise ValueError('robot_id must be a valid UUID')

        self.headers = {'x-api-key': self.api_key, 'Content-Type': 'application/json'}

        # Parse extra_float_topics into (topic, field_name) pairs. A malformed
        # entry is logged and skipped rather than crashing the whole node —
        # one typo in an extra sensor shouldn't take down episode recording.
        self.extra_float_topics = []
        for entry in extra_float_topics_raw.split(','):
            entry = entry.strip()
            if not entry:
                continue
            if ':' not in entry:
                rospy.logwarn(
                    'Skipping malformed extra_float_topics entry (expected '
                    '"topic:field_name"): %r' % entry
                )
                continue
            topic, field_name = entry.split(':', 1)
            topic, field_name = topic.strip(), field_name.strip()
            if not topic or not field_name:
                rospy.logwarn('Skipping malformed extra_float_topics entry: %r' % entry)
                continue
            self.extra_float_topics.append((topic, field_name))

        # State
        self.recording = False
        self.current_task_id = None  # type: Optional[str]
        self.episode_start = None  # type: Optional[str]
        self.observations = []
        self.actions = []
        self.metadata = {}

        # Latest sensor values
        self.latest_joint_state = None  # type: Optional[dict]
        self.latest_ft = None  # type: Optional[dict]
        self.latest_gripper = None  # type: Optional[float]
        self.latest_extra = {}
        self.last_obs_time = 0.0

        # Attempt to recover leftover buffer from a previous crash
        self._recover_buffer()

        # Subscriptions. ROS 1 has no QoS profile concept (no BEST_EFFORT/RELIABLE
        # split like ROS 2) — queue_size is the closest analogue to History
        # KEEP_LAST depth.
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_state_cb, queue_size=10)
        rospy.Subscriber(self.ft_sensor_topic, WrenchStamped, self.ft_cb, queue_size=10)
        rospy.Subscriber(self.gripper_topic, Float64, self.gripper_cb, queue_size=10)
        for topic, field_name in self.extra_float_topics:
            rospy.Subscriber(topic, Float64, self._make_extra_float_cb(field_name), queue_size=10)
        rospy.Subscriber('blackbox/task_event', String, self.task_event_cb, queue_size=10)

        # Publisher
        self.status_pub = rospy.Publisher('blackbox/episode_status', String, queue_size=10)

        # Observation collection timer. rospy.Timer callbacks always receive a
        # TimerEvent arg (unlike rclpy's create_timer, which takes none) —
        # collect_observation accepts an optional event param to match.
        interval_sec = self.obs_interval_ms / 1000.0
        rospy.Timer(rospy.Duration(interval_sec), self.collect_observation)

        extra_topics_str = ', '.join('%s->%s' % (t, f) for t, f in self.extra_float_topics) or 'none'
        rospy.loginfo(
            'Black Box Episode Recorder initialized (ROS 1) — robot_id=%s, api=%s, '
            'interval=%dms | topics: joints=%s ft=%s gripper=%s extra=[%s]' % (
                self.robot_id, self.api_url, self.obs_interval_ms,
                self.joint_states_topic, self.ft_sensor_topic, self.gripper_topic,
                extra_topics_str,
            )
        )

    def joint_state_cb(self, msg):
        self.latest_joint_state = {
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
        }

    def ft_cb(self, msg):
        self.latest_ft = {
            'force': {'x': msg.wrench.force.x, 'y': msg.wrench.force.y, 'z': msg.wrench.force.z},
            'torque': {'x': msg.wrench.torque.x, 'y': msg.wrench.torque.y, 'z': msg.wrench.torque.z},
        }

    def gripper_cb(self, msg):
        self.latest_gripper = msg.data

    def _make_extra_float_cb(self, field_name):
        """Build a callback that stores an extra Float64 topic's value under field_name."""
        def _cb(msg):
            self.latest_extra[field_name] = msg.data
        return _cb

    def task_event_cb(self, msg):
        """Handle task start/end events.

        Expected JSON format:
          Start: {"event": "start", "task_id": "pick_and_place", "metadata": {...}}
          Action: {"event": "action", "action_type": "grasp", "parameters": {...}}
          End:   {"event": "end", "success": true}
        """
        try:
            data = json.loads(msg.data)
        except ValueError:
            rospy.logwarn('Invalid JSON in task_event: %s' % msg.data)
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

    def start_episode(self, task_id, metadata):
        if self.recording:
            rospy.logwarn('Already recording — ending previous episode')
            self.end_episode(success=None)

        self.recording = True
        self.current_task_id = task_id
        self.episode_start = datetime.now(timezone.utc).isoformat()
        self.observations = []
        self.actions = []
        self.metadata = metadata

        self.publish_status('recording', task_id)
        rospy.loginfo('Episode started — task=%s' % task_id)

    def collect_observation(self, event=None):
        if not self.recording:
            return
        if len(self.observations) >= self.max_obs:
            return

        now = rospy.get_time()
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

    def end_episode(self, success):
        if not self.recording:
            rospy.logwarn('Not recording — ignoring end event')
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

        rospy.loginfo(
            'Episode ended — task=%s, observations=%d, actions=%d, success=%s' % (
                self.current_task_id, len(self.observations), len(self.actions), success,
            )
        )

        self.push_episode(episode_data)
        self.publish_status('idle', self.current_task_id or '')

    # ------------------------------------------------------------------
    # Local JSON buffer for crash recovery — identical to the ROS 2 version,
    # zero ROS dependency.
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
            rospy.logwarn('Failed to write episode buffer: %s' % e)

    def _delete_buffer(self):
        """Remove the local buffer file after a successful upload."""
        try:
            self.BUFFER_PATH.unlink()
        except OSError:
            pass

    def _recover_buffer(self):
        """On startup, try to re-upload a leftover buffer from a previous crash."""
        if not self.BUFFER_PATH.exists():
            return
        rospy.loginfo('Found leftover episode buffer — attempting re-upload')
        try:
            data = json.loads(self.BUFFER_PATH.read_text())
            # Fill in end_time as unknown since the previous run crashed
            data.setdefault('end_time', None)
            data.setdefault('success', None)
            self._upload_episode(data, from_recovery=True)
        except (ValueError, OSError) as e:
            rospy.logwarn('Could not read leftover buffer: %s' % e)

    # ------------------------------------------------------------------
    # Upload helpers
    # ------------------------------------------------------------------

    def push_episode(self, data):
        """Push completed episode to the Black Box Robotics API."""
        # Ensure the final state is flushed to disk before uploading
        self._flush_buffer()
        self._upload_episode(data, from_recovery=False)

    def _upload_episode(self, data, from_recovery):
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
                    '%s/episodes' % self.api_url,
                    json=data,
                    headers=self.headers,
                    timeout=30,
                )
                if resp.status_code == 201:
                    episode_id = resp.json().get('data', {}).get('id', 'unknown')
                    rospy.loginfo('Episode pushed successfully — id=%s' % episode_id)
                    self._delete_buffer()
                    return
                rospy.logerr('Failed to push episode: %d — %s. Falling back to local export.' % (resp.status_code, resp.text))
            except requests.RequestException as e:
                rospy.logerr('Failed to push episode: %s. Falling back to local export.' % e)

        zip_path, session_id = export_offline_session(data, self.export_dir)
        rospy.loginfo('Offline session exported: %s (session_id=%s)' % (zip_path, session_id))
        self._delete_buffer()

    def publish_status(self, state, task_id):
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'task_id': task_id,
            'robot_id': self.robot_id,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        })
        self.status_pub.publish(msg)


def main():
    try:
        EpisodeRecorder()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError):
        pass


if __name__ == '__main__':
    main()

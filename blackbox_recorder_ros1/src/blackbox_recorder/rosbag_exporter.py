"""
Black Box Robotics Rosbag Exporter — ROS 1 node that exports .bag recordings into
structured episodes and pushes them to the Black Box Robotics API.

.bag is ROS 1's own native format, so unlike the ROS 2 exporter (which requires
converting .bag to .mcap first, since rosbag2 doesn't read ROS 1 bags), this reads
your existing .bag files directly — no conversion step.

Usage:
  rosrun blackbox_recorder rosbag_exporter _bag_path:=/path/to/recording.bag \
    _api_url:=http://localhost:3001/api \
    _api_key:=pk_... \
    _robot_id:=<uuid> \
    _task_id:=pick_and_place

Offline / air-gapped mode (no live network, e.g. RF-denied environments):
  rosrun blackbox_recorder rosbag_exporter _bag_path:=/path/to/recording.bag \
    _offline_mode:=true \
    _export_dir:=/media/sdcard/blackbox_exports \
    _robot_id:=<uuid> \
    _task_id:=pick_and_place

  Writes a session_*.zip (episode.json + manifest.json + bag file) instead
  of POSTing. Move the zip off the drone (SD card / USB) and upload it via
  the dashboard's "Upload Offline Session" flow, or POST it directly to
  /api/episodes/import. If a live POST is attempted and fails, the episode
  falls back to this same local zip export rather than being dropped.
"""

import json
import os
from datetime import datetime, timezone

import requests
import rospy

from blackbox_recorder.offline_export import export_offline_session

try:
    import rosbag
    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False


def _downsample_indices(n, target):
    """Evenly spaced indices, n items down to at most target. Pure Python — no numpy dependency."""
    if n <= target:
        return list(range(n))
    step = (n - 1) / float(target - 1) if target > 1 else n
    return sorted(set(int(round(i * step)) for i in range(target)))


class RosbagExporter(object):
    """Reads a ROS 1 .bag file and exports one episode to Black Box Robotics."""

    def __init__(self):
        rospy.init_node('blackbox_rosbag_exporter')

        self.bag_path = rospy.get_param('~bag_path', '')
        self.api_url = rospy.get_param('~api_url', 'https://www.bbrobotics.in/api')
        self.api_key = rospy.get_param('~api_key', '')
        self.robot_id = rospy.get_param('~robot_id', '')
        self.task_id = rospy.get_param('~task_id', 'unknown')

        self.joint_states_topic = rospy.get_param('~joint_states_topic', 'joint_states')
        self.ft_sensor_topic = rospy.get_param('~ft_sensor_topic', 'ft_sensor')

        self.offline_mode = rospy.get_param('~offline_mode', False)
        self.export_dir = rospy.get_param('~export_dir', os.path.expanduser('~/.blackbox/exports'))

        # api_key/api_url aren't meaningful in offline mode — nothing gets POSTed
        required = (self.bag_path and self.robot_id) if self.offline_mode \
            else (self.bag_path and self.api_key and self.robot_id)
        if not required:
            rospy.logerr('bag_path and robot_id are required (api_key also required unless offline_mode:=true)')
            raise ValueError('Missing required parameters')

        self.headers = {'x-api-key': self.api_key, 'Content-Type': 'application/json'}

    def process_bag(self):
        if not HAS_ROSBAG:
            rospy.logerr('rosbag module not available — install ros-<distro>-rosbag')
            return

        rospy.loginfo('Processing bag: %s' % self.bag_path)

        observations = []
        actions = []
        start_t = None
        end_t = None

        topics = [self.joint_states_topic, self.ft_sensor_topic, 'blackbox/task_event']
        # Also accept globally-namespaced variants, same as the ROS 2 exporter did.
        topics += ['/' + t.lstrip('/') for t in topics]
        topics = list(dict.fromkeys(topics))  # de-dupe, preserve order

        bag = rosbag.Bag(self.bag_path, 'r')
        try:
            for topic, msg, t in bag.read_messages(topics=topics):
                if start_t is None:
                    start_t = t
                end_t = t

                ts = datetime.fromtimestamp(t.to_sec(), tz=timezone.utc).isoformat()

                if topic in (self.joint_states_topic, '/' + self.joint_states_topic.lstrip('/')):
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
                elif topic in (self.ft_sensor_topic, '/' + self.ft_sensor_topic.lstrip('/')):
                    # Append force/torque to the latest observation, same convention
                    # as the live recorder and the ROS 2 exporter.
                    if observations:
                        observations[-1]['sensor_data']['force_torque'] = {
                            'force': {'x': msg.wrench.force.x, 'y': msg.wrench.force.y, 'z': msg.wrench.force.z},
                            'torque': {'x': msg.wrench.torque.x, 'y': msg.wrench.torque.y, 'z': msg.wrench.torque.z},
                        }
                elif topic in ('blackbox/task_event', '/blackbox/task_event'):
                    try:
                        event = json.loads(msg.data)
                        if event.get('event') == 'action':
                            actions.append({
                                'timestamp': ts,
                                'action_type': event.get('action_type', 'unknown'),
                                'parameters': event.get('parameters', {}),
                            })
                    except ValueError:
                        pass
        finally:
            bag.close()

        if start_t is None:
            rospy.logwarn('Bag contained none of the expected topics: %s' % topics)
            return

        start_time = datetime.fromtimestamp(start_t.to_sec(), tz=timezone.utc).isoformat()
        end_time = datetime.fromtimestamp(end_t.to_sec(), tz=timezone.utc).isoformat()

        # Downsample to avoid huge payloads, same 500-observation cap as the ROS 2
        # exporter. Original count preserved in metadata either way.
        max_obs = 500
        original_count = len(observations)
        if len(observations) > max_obs:
            indices = _downsample_indices(len(observations), max_obs)
            observations = [observations[i] for i in indices]

        episode = {
            'robot_id': self.robot_id,
            'task_id': self.task_id,
            'start_time': start_time,
            'end_time': end_time,
            'success': None,
            'metadata': {
                'source': 'rosbag1',
                'bag_path': self.bag_path,
                'original_observations': original_count,
            },
            'observations': observations,
            'actions': actions,
        }

        rospy.loginfo(
            'Extracted episode: %d observations, %d actions' % (len(observations), len(actions))
        )

        if self.offline_mode:
            self._export_offline(episode)
            return

        try:
            resp = requests.post(
                '%s/episodes' % self.api_url,
                json=episode,
                headers=self.headers,
                timeout=60,
            )
            if resp.status_code == 201:
                eid = resp.json().get('data', {}).get('id', 'unknown')
                rospy.loginfo('Episode created: %s' % eid)
            else:
                rospy.logerr('API error: %d — %s. Falling back to local export.' % (resp.status_code, resp.text))
                self._export_offline(episode)
        except requests.RequestException as e:
            rospy.logerr('Push failed: %s. Falling back to local export.' % e)
            self._export_offline(episode)

    def _export_offline(self, episode):
        zip_path, session_id = export_offline_session(episode, self.export_dir, bag_path=self.bag_path)
        rospy.loginfo('Offline session exported: %s (session_id=%s)' % (zip_path, session_id))


def main():
    try:
        exporter = RosbagExporter()
        exporter.process_bag()
    except (rospy.ROSInterruptException, ValueError):
        pass


if __name__ == '__main__':
    main()

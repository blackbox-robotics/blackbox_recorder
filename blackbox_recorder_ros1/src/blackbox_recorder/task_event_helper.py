"""
BlackBoxEventPublisher — drop into your existing ROS 1 node to publish
task events to the Black Box Robotics episode recorder without writing JSON manually.

Usage inside your node:
    from blackbox_recorder.task_event_helper import BlackBoxEventPublisher

    class MyRobotNode(object):
        def __init__(self):
            rospy.init_node('my_robot')
            self.blackbox = BlackBoxEventPublisher()

        def run_task(self):
            self.blackbox.start_episode('pick_and_place', {'target': 'bolt_A'})
            success = self.do_pick_and_place()
            self.blackbox.log_action('grasp', {'object': 'bolt_A', 'force_N': 12.3})
            self.blackbox.end_episode(success=success)

CLI usage (for manual testing):
    rosrun blackbox_recorder task_event '{"event":"start","task_id":"pick_and_place"}'
    rosrun blackbox_recorder task_event '{"event":"end","success":true}'
"""

import json
import sys
import time

import rospy
from std_msgs.msg import String


class BlackBoxEventPublisher(object):
    """Publishes task lifecycle events to the Black Box Robotics episode recorder node."""

    TOPIC = 'blackbox/task_event'

    def __init__(self):
        # No Node object to attach to in rospy — the publisher just needs
        # init_node() to have already run somewhere (in your node's __init__,
        # same as the ROS 2 version expects a Node instance to be passed in).
        self._pub = rospy.Publisher(self.TOPIC, String, queue_size=10)

    def start_episode(self, task_id, metadata=None):
        """Signal the recorder to begin capturing a new episode."""
        self._publish({
            'event': 'start',
            'task_id': task_id,
            'metadata': metadata or {},
        })

    def log_action(self, action_type, parameters=None):
        """Record a discrete action taken during the current episode."""
        self._publish({
            'event': 'action',
            'action_type': action_type,
            'parameters': parameters or {},
        })

    def end_episode(self, success):
        """Signal the recorder to finalize and upload the current episode."""
        self._publish({
            'event': 'end',
            'success': success,
        })

    def _publish(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main():
    """
    One-shot CLI publisher for testing task events from the terminal.

    Examples:
        rosrun blackbox_recorder task_event '{"event":"start","task_id":"pick_and_place"}'
        rosrun blackbox_recorder task_event '{"event":"action","action_type":"grasp","parameters":{}}'
        rosrun blackbox_recorder task_event '{"event":"end","success":true}'
    """
    if len(sys.argv) < 2:
        print('Usage: rosrun blackbox_recorder task_event \'{"event":"start","task_id":"..."}\' ')
        sys.exit(1)

    payload = sys.argv[1]

    # Validate JSON before publishing
    try:
        json.loads(payload)
    except ValueError as e:
        print('Invalid JSON: %s' % e)
        sys.exit(1)

    rospy.init_node('blackbox_task_event_cli', anonymous=True)
    pub = rospy.Publisher('blackbox/task_event', String, queue_size=10)

    # Wait for actual discovery instead of a blind sleep — a one-shot CLI
    # process that publishes before the recorder node has subscribed will
    # silently drop the message. Poll get_num_connections() for up to 3s;
    # warn loudly if nobody ever connects, since that means episode_recorder
    # isn't running or isn't reachable.
    DISCOVERY_TIMEOUT_S = 3.0
    waited = 0.0
    while pub.get_num_connections() == 0 and waited < DISCOVERY_TIMEOUT_S:
        time.sleep(0.1)
        waited += 0.1

    if pub.get_num_connections() == 0:
        print(
            'WARNING: no subscriber found on blackbox/task_event after '
            '%.0fs — is episode_recorder running? '
            'Publishing anyway, but the recorder will not see this event.' % DISCOVERY_TIMEOUT_S
        )

    msg = String()
    msg.data = payload
    pub.publish(msg)

    time.sleep(0.1)
    rospy.loginfo('Published to blackbox/task_event: %s' % payload)


if __name__ == '__main__':
    main()

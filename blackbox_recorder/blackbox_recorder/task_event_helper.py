"""
BlackBoxEventPublisher — drop into your existing ROS 2 node to publish
task events to the Black Box Robotics episode recorder without writing JSON manually.

Usage inside your node:
    from blackbox_recorder.task_event_helper import BlackBoxEventPublisher

    class MyRobotNode(Node):
        def __init__(self):
            super().__init__('my_robot')
            self.blackbox = BlackBoxEventPublisher(self)

        def run_task(self):
            self.blackbox.start_episode('pick_and_place', {'target': 'bolt_A'})
            success = self.do_pick_and_place()
            self.blackbox.log_action('grasp', {'object': 'bolt_A', 'force_N': 12.3})
            self.blackbox.end_episode(success=success)

CLI usage (for manual testing):
    ros2 run blackbox_recorder task_event '{"event":"start","task_id":"pick_and_place"}'
    ros2 run blackbox_recorder task_event '{"event":"end","success":true}'
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BlackBoxEventPublisher:
    """Publishes task lifecycle events to the Black Box Robotics episode recorder node."""

    TOPIC = 'blackbox/task_event'

    def __init__(self, node: Node):
        self._pub = node.create_publisher(String, self.TOPIC, 10)

    def start_episode(self, task_id: str, metadata: dict = None) -> None:
        """Signal the recorder to begin capturing a new episode."""
        self._publish({
            'event': 'start',
            'task_id': task_id,
            'metadata': metadata or {},
        })

    def log_action(self, action_type: str, parameters: dict = None) -> None:
        """Record a discrete action taken during the current episode."""
        self._publish({
            'event': 'action',
            'action_type': action_type,
            'parameters': parameters or {},
        })

    def end_episode(self, success: bool) -> None:
        """Signal the recorder to finalize and upload the current episode."""
        self._publish({
            'event': 'end',
            'success': success,
        })

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main(args=None):
    """
    One-shot CLI publisher for testing task events from the terminal.

    Examples:
        ros2 run blackbox_recorder task_event '{"event":"start","task_id":"pick_and_place"}'
        ros2 run blackbox_recorder task_event '{"event":"action","action_type":"grasp","parameters":{}}'
        ros2 run blackbox_recorder task_event '{"event":"end","success":true}'
    """
    if len(sys.argv) < 2:
        print('Usage: ros2 run blackbox_recorder task_event \'{"event":"start","task_id":"..."}\' ')
        sys.exit(1)

    payload = sys.argv[1]

    # Validate JSON before publishing
    try:
        json.loads(payload)
    except json.JSONDecodeError as e:
        print(f'Invalid JSON: {e}')
        sys.exit(1)

    rclpy.init(args=args)
    node = rclpy.create_node('blackbox_task_event_cli')
    pub = node.create_publisher(String, 'blackbox/task_event', 10)

    # Wait for actual DDS discovery instead of a blind sleep — a one-shot CLI
    # process that publishes before the recorder node has discovered it will
    # silently drop the message (reliable QoS guarantees delivery to matched
    # subscribers, not to ones that haven't matched yet). Poll subscriber
    # count for up to 3s; warn loudly if nobody ever connects, since that
    # means the episode_recorder node isn't running or isn't reachable.
    DISCOVERY_TIMEOUT_S = 3.0
    waited = 0.0
    while pub.get_subscription_count() == 0 and waited < DISCOVERY_TIMEOUT_S:
        time.sleep(0.1)
        waited += 0.1

    if pub.get_subscription_count() == 0:
        print(
            f'WARNING: no subscriber found on blackbox/task_event after '
            f'{DISCOVERY_TIMEOUT_S}s — is episode_recorder running? '
            f'Publishing anyway, but the recorder will not see this event.'
        )

    msg = String()
    msg.data = payload
    pub.publish(msg)

    time.sleep(0.1)
    node.get_logger().info(f'Published to blackbox/task_event: {payload}')

    rclpy.shutdown()

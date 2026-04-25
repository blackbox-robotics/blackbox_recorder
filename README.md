# BlackBox — ROS 2 Integration Quickstart

Get your robot sending data to the BlackBox dashboard in under 10 minutes.

---

## Automated Setup (recommended)

One command installs, builds, and registers the recorder as a systemd service
that starts on boot and restarts on crash.

```bash
# From the BlackBox repo root on your robot machine:
BLACKBOX_API_URL=http://BLACKBOX_HOST:3001/api \
BLACKBOX_API_KEY=pk_YOUR_KEY \
BLACKBOX_ROBOT_ID=YOUR_ROBOT_UUID \
bash setup-robot.sh
```

Or run it interactively — it will prompt for any missing values:

```bash
bash setup-robot.sh
```

After setup, the recorder runs as `blackbox-recorder.service`. Skip to
[Step 4 — Wire Up Task Events](#step-4--wire-up-task-events).

---

## Manual Setup

Follow these steps if you prefer not to use the setup script.

### Prerequisites

- ROS 2 **Humble** or **Iron** (tested on Ubuntu 22.04)
- Your BlackBox backend running and reachable from the robot (local network or hosted)
- `python3-requests` installed on the robot machine

```bash
sudo apt install python3-requests
```

---

## Step 1 — Get Your Credentials

1. Open the BlackBox dashboard and go to **Settings**
2. Copy your **API Key** — it looks like `pk_a3f9...` (48 hex chars after the prefix)
3. In Settings, create a new **Robot** entry for this machine
4. Copy the **Robot UUID** shown after creation

You will need both values in Step 3.

---

## Step 2 — Install the Recorder Node

Copy the package into your ROS 2 workspace and build it:

```bash
# From the BlackBox repo root
cp -r blackbox_recorder ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select blackbox_recorder
source install/setup.bash
```

Verify the install:

```bash
ros2 pkg list | grep blackbox
# Should print: blackbox_recorder
```

---

## Step 3 — Launch the Recorder

### Option A — One-liner (quickest for testing)

```bash
ros2 run blackbox_recorder episode_recorder --ros-args \
  -p api_url:=http://BLACKBOX_HOST:3001/api \
  -p api_key:=pk_YOUR_KEY \
  -p robot_id:=YOUR_ROBOT_UUID
```

Replace `BLACKBOX_HOST` with the IP or hostname of your BlackBox backend.

### Option B — YAML config file (recommended for production)

Edit the config file:

```bash
nano ~/ros2_ws/src/blackbox_recorder/config/recorder_params.yaml
```

```yaml
blackbox_episode_recorder:
  ros__parameters:
    api_url: "http://BLACKBOX_HOST:3001/api"
    api_key: "pk_YOUR_KEY"
    robot_id: "YOUR_ROBOT_UUID"
    observation_interval_ms: 100   # collect sensor snapshot every 100ms
    max_observations: 1000         # cap per episode
    joint_states_topic: "/joint_states"
    ft_sensor_topic: "/ft_sensor"
    gripper_topic: "/gripper/state"
```

Then launch:

```bash
ros2 launch blackbox_recorder recorder_launch.py
```

### Option C — Embed in your robot's existing launch file

```python
# In your robot.launch.py
from launch_ros.actions import Node as RosNode

blackbox_recorder = RosNode(
    package='blackbox_recorder',
    executable='episode_recorder',
    name='blackbox_episode_recorder',
    parameters=[{
        'api_url': 'http://BLACKBOX_HOST:3001/api',
        'api_key': 'pk_YOUR_KEY',
        'robot_id': 'YOUR_ROBOT_UUID',
        'joint_states_topic': '/ur/joint_states',   # if your robot uses a prefix
    }],
    output='screen',
)

# Add blackbox_recorder to your LaunchDescription's action list
```

---

## Step 4 — Wire Up Task Events

The recorder listens on `/blackbox/task_event` for JSON strings that tell it
when a task starts and ends. You control this from your robot's own code.

### Option A — Use the helper class (cleanest)

```python
from blackbox_recorder.task_event_helper import BlackBoxEventPublisher

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.blackbox = BlackBoxEventPublisher(self)

    def execute_task(self, target: str):
        # 1. Tell BlackBox a new episode is starting
        self.blackbox.start_episode('pick_and_place', metadata={'target': target})

        # 2. Run your existing robot logic — nothing changes here
        success = self.run_motion_planner(target)

        # 3. Log discrete actions mid-episode (optional)
        self.blackbox.log_action('grasp', {'object': target, 'force_threshold_N': 15})

        # 4. Tell BlackBox the episode ended
        self.blackbox.end_episode(success=success)
```

### Option B — Publish the JSON yourself

```python
import json
from std_msgs.msg import String

task_pub = self.create_publisher(String, '/blackbox/task_event', 10)

def start(task_id, metadata=None):
    msg = String()
    msg.data = json.dumps({'event': 'start', 'task_id': task_id, 'metadata': metadata or {}})
    task_pub.publish(msg)

def end(success: bool):
    msg = String()
    msg.data = json.dumps({'event': 'end', 'success': success})
    task_pub.publish(msg)
```

### Option C — CLI (for manual testing only)

```bash
# Start an episode
ros2 run blackbox_recorder task_event '{"event":"start","task_id":"pick_and_place"}'

# Log an action mid-run
ros2 run blackbox_recorder task_event '{"event":"action","action_type":"grasp","parameters":{"object":"bolt_A"}}'

# End the episode (success)
ros2 run blackbox_recorder task_event '{"event":"end","success":true}'

# End the episode (failure)
ros2 run blackbox_recorder task_event '{"event":"end","success":false}'
```

---

## Step 5 — Verify Data Is Flowing

After sending a task end event, open the BlackBox dashboard:

- **Episodes** tab — your episode appears within a few seconds
- Click the episode to see the joint state timeline, sensor data, and any actions you logged
- **Fleet** tab — your robot shows as active with a live heartbeat

To confirm the recorder is receiving your topics:

```bash
# Check what the recorder has subscribed to
ros2 node info /blackbox_episode_recorder

# Check the recorder's live status
ros2 topic echo /blackbox/episode_status
```

---

## Topic Remapping

The recorder defaults to standard ROS 2 topic names. If your robot uses
namespaced or non-standard names, override them with parameters — no remapping
YAML needed.

### Common robots

**Universal Robots (UR5 / UR10)**
```bash
ros2 run blackbox_recorder episode_recorder --ros-args \
  -p api_key:=pk_... -p robot_id:=... \
  -p joint_states_topic:=/ur/joint_states
```

**Franka Panda**
```bash
ros2 run blackbox_recorder episode_recorder --ros-args \
  -p api_key:=pk_... -p robot_id:=... \
  -p joint_states_topic:=/franka/joint_states \
  -p ft_sensor_topic:=/franka_state_controller/F_ext
```

**Custom namespaced robot**
```bash
ros2 run blackbox_recorder episode_recorder --ros-args \
  -p api_key:=pk_... -p robot_id:=... \
  -p joint_states_topic:=/robot1/joint_states \
  -p ft_sensor_topic:=/robot1/force_torque \
  -p gripper_topic:=/robot1/gripper/position
```

**No F/T sensor or gripper** — leave those parameters at their defaults.
The recorder silently skips sensors that publish nothing; only `joint_states`
contributes non-empty `sensor_data` in observations.

---

## Importing Historical Data (rosbag2)

If you have existing rosbag2 recordings, import them without re-running experiments:

```bash
ros2 run blackbox_recorder rosbag_exporter --ros-args \
  -p bag_path:=/path/to/your/rosbag2_dir \
  -p api_url:=http://BLACKBOX_HOST:3001/api \
  -p api_key:=pk_YOUR_KEY \
  -p robot_id:=YOUR_ROBOT_UUID \
  -p task_id:=pick_and_place
```

The exporter reads `/joint_states` and `/ft_sensor` from the bag. If your bag
uses different topic names:

```bash
  -p joint_states_topic:=/ur/joint_states \
  -p ft_sensor_topic:=/ur/wrench
```

Large bags are auto-downsampled to 500 observations to keep payload size
reasonable. The original count is stored in episode metadata.

---

## Crash Recovery

The recorder buffers the current episode to `/tmp/blackbox_episode_buffer.json`
every 100ms. If the node crashes or the robot reboots mid-task:

- On next startup the node detects the leftover buffer
- It re-uploads the episode automatically (marked with `end_time: null`)
- The buffer file is deleted after a successful upload

No data loss from power cuts or node crashes.

---

## Troubleshooting

| Symptom | Most likely cause | Fix |
|---------|-------------------|-----|
| Node exits immediately on launch | `api_key` or `robot_id` parameter is empty | Set both parameters before launching |
| Episodes never appear in dashboard | Wrong `robot_id` | UUID must match a robot you created in Settings |
| `joint_states` field empty in episodes | Your robot publishes to a different topic name | Set `-p joint_states_topic:=/your/topic` |
| API 401 Unauthorized | Wrong or expired API key | Re-copy key from Settings > API Keys |
| API 404 Not Found | `robot_id` does not belong to your org | Create the robot in Settings first |
| Network timeout on episode push | Backend unreachable | Check `api_url`, firewall rules, backend health |
| "Episode buffer retained" in logs | Upload failed — network was down | Episode will re-upload automatically on next start |
| "Already recording" warning | Previous episode never received an `end` event | Always publish `{"event":"end",...}` even on error paths |
| No data in Fleet tab | WebSocket blocked | Check port 3001 is open and `api_url` is correct |

---

## All Parameters Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `api_url` | `http://localhost:3001/api` | BlackBox backend base URL |
| `api_key` | *(required)* | API key from dashboard Settings |
| `robot_id` | *(required)* | Robot UUID from dashboard Settings |
| `observation_interval_ms` | `100` | How often to snapshot sensors (ms) |
| `max_observations` | `1000` | Max observations per episode |
| `joint_states_topic` | `/joint_states` | ROS topic for joint state data |
| `ft_sensor_topic` | `/ft_sensor` | ROS topic for force/torque sensor |
| `gripper_topic` | `/gripper/state` | ROS topic for gripper position |

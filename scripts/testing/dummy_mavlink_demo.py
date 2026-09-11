#!/usr/bin/env python3
"""
DEMO / TEST FIXTURE — simulates a vehicle running episode_recorder.py in
mavlink_upload_enabled mode, publishing fake observations so you can watch
a live episode appear on the BlackBox dashboard without real hardware.

Produces the exact same batch JSON shape and MAVLink FTP push mechanism the
real recorder uses (see episode_recorder.py's _flush_mavlink_batch) — this
is a clean standalone reimplementation (no ROS/rclpy dependency) so it's
easy to run directly, not a copy loaded via import tricks.

Run this alongside mavlink_upload_receiver.py, which must be pointed at a
real BlackBox API (e.g. https://demo.bbrobotics.in/api) with a valid API
key — that's what actually makes the episode show up live on the dashboard.

    # Terminal 1 — the ground receiver (talks to the real API)
    python3 mavlink_upload_receiver.py \\
      --connection udpin:0.0.0.0:14560 \\
      --incoming-dir /tmp/mavlink-demo-incoming \\
      --api-url https://demo.bbrobotics.in/api \\
      --api-key pk_your_key

    # Terminal 2 — this script (simulates the vehicle)
    python3 dummy_mavlink_demo.py \\
      --connection udpout:127.0.0.1:14560 \\
      --robot-id <a UUID — see Settings > Robots on the dashboard, or
                  leave unset to auto-generate and auto-register one>

Requires: pymavlink (pip install pymavlink).
"""

import argparse
import json
import logging
import math
import os
import sys
import time
import uuid
from datetime import datetime, timezone

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP, MAVFTPSettings

log = logging.getLogger("dummy_mavlink_demo")


def connect_mavftp(connection: str, source_system: int) -> MAVFTP:
    """A udpin (server) socket only replies to addresses it has already
    heard from (pymavlink's mavudp.write() sends only to self.clients,
    populated by recv()) — so a purely receive-only wait_heartbeat() on
    this side deadlocks forever against a receiver that's also only
    replying. Broadcast our own heartbeat while waiting so the server has
    something to reply to, same as any real MAVLink client/GCS does."""
    log.info("Connecting to %s ...", connection)
    master = mavutil.mavlink_connection(connection, source_system=source_system)
    log.info("Waiting for heartbeat from the receiver...")
    deadline = time.time() + 30
    hb = None
    while time.time() < deadline and hb is None:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        hb = master.wait_heartbeat(timeout=1)
    if hb is None:
        raise TimeoutError(f"No heartbeat from receiver on {connection} after 30s")
    log.info("Heartbeat from system %d component %d", master.target_system, master.target_component)
    return MAVFTP(
        master,
        target_system=master.target_system,
        target_component=master.target_component,
        settings=MAVFTPSettings([
            ("debug", int, 0), ("pkt_loss_tx", int, 0), ("pkt_loss_rx", int, 0),
            ("max_backlog", int, 5), ("burst_read_size", int, 80),
            ("write_size", int, 80), ("write_qsize", int, 5),
            ("idle_detection_time", float, 3.7), ("read_retry_time", float, 1.0),
            ("retry_time", float, 0.5),
        ]),
    )


def push_batch(ftp: MAVFTP, batch_dir: str, batch: dict) -> bool:
    filename = f"livebatch_{batch['session_id']}_{batch['seq']:04d}.json"
    path = os.path.join(batch_dir, filename)
    with open(path, "w") as f:
        json.dump(batch, f)

    ftp.cmd_put([path, filename])
    ret = ftp.process_ftp_reply("CreateFile", timeout=30)
    if ret.error_code:
        log.warning("Push failed for %s: %s", filename, ret)
        return False
    log.info(
        "Pushed batch seq=%d (%d observations, %d actions)%s",
        batch["seq"], len(batch["observations"]), len(batch["actions"]),
        " [final]" if batch["is_final"] else "",
    )
    return True


def make_observation(t: float) -> dict:
    """A smooth fake joint-state + sensor trace so the dashboard's live
    chart shows something visibly moving, not just noise."""
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "joint_states": {
            "names": ["shoulder", "elbow", "wrist"],
            "positions": [math.sin(t / 3.0), math.cos(t / 4.0), math.sin(t / 2.0) * 0.5],
            "velocities": [math.cos(t / 3.0) / 3.0, -math.sin(t / 4.0) / 4.0, math.cos(t / 2.0) / 4.0],
            "efforts": [],
        },
        "sensor_data": {
            "force_torque": {
                "force": {"x": math.sin(t) * 2, "y": math.cos(t) * 2, "z": 9.8 + math.sin(t / 5.0)},
                "torque": {"x": 0.0, "y": 0.0, "z": math.sin(t / 2.0)},
            },
            "gripper_position": (math.sin(t / 6.0) + 1) / 2,
            "battery_voltage": 23.5 - (t / 600.0),  # slow simulated drain
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", required=True, help="MAVLink connection to the receiver, e.g. udpout:127.0.0.1:14560")
    parser.add_argument("--robot-id", default=None, help="Robot UUID (from the dashboard). Omit to auto-generate — auto-registers as a new robot on first batch, same as a real recorder.")
    parser.add_argument("--task-id", default="mavlink_live_demo")
    parser.add_argument("--source-system", type=int, default=77)
    parser.add_argument("--batch-interval-s", type=float, default=2.0)
    parser.add_argument("--observation-interval-s", type=float, default=0.5)
    parser.add_argument("--duration-s", type=float, default=0, help="Stop and close the episode after this many seconds. 0 = run until Ctrl+C.")
    parser.add_argument("--batch-dir", default="/tmp/dummy_mavlink_demo_batches")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    os.makedirs(args.batch_dir, exist_ok=True)

    robot_id = args.robot_id or str(uuid.uuid4())
    session_id = str(uuid.uuid4())
    log.info("robot_id=%s task_id=%s session_id=%s", robot_id, args.task_id, session_id)
    if not args.robot_id:
        log.info("No --robot-id given — using a fresh UUID. It'll auto-register as a new robot, same as any first-time recorder.")

    ftp = connect_mavftp(args.connection, args.source_system)

    start_time = datetime.now(timezone.utc).isoformat()
    seq = 0
    pending_observations = []
    pending_actions = []
    t0 = time.time()
    last_obs = 0.0
    last_batch = time.time()

    log.info("Recording started — Ctrl+C to end the episode cleanly at any time")
    try:
        while True:
            now = time.time()
            elapsed = now - t0

            if args.duration_s and elapsed >= args.duration_s:
                break

            if now - last_obs >= args.observation_interval_s:
                pending_observations.append(make_observation(elapsed))
                last_obs = now

            # A fake action partway through, once, just to show the actions
            # list isn't empty on the episode detail page.
            if 4.5 < elapsed < 5.5 and not pending_actions and seq == 0:
                pending_actions.append({
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "action_type": "demo_marker",
                    "parameters": {"note": "MAVLink live demo action"},
                })

            if now - last_batch >= args.batch_interval_s and (pending_observations or pending_actions):
                batch = {
                    "session_id": session_id, "robot_id": robot_id, "task_id": args.task_id,
                    "start_time": start_time, "metadata": {"source": "dummy_mavlink_demo"},
                    "seq": seq, "is_final": False,
                    "observations": pending_observations, "actions": pending_actions,
                }
                if push_batch(ftp, args.batch_dir, batch):
                    seq += 1
                    pending_observations = []
                    pending_actions = []
                last_batch = now

            time.sleep(0.1)
    except KeyboardInterrupt:
        log.info("Interrupted — closing episode")

    final_batch = {
        "session_id": session_id, "robot_id": robot_id, "task_id": args.task_id,
        "start_time": start_time, "metadata": {"source": "dummy_mavlink_demo"},
        "seq": seq, "is_final": True,
        "observations": pending_observations, "actions": pending_actions,
        "end_time": datetime.now(timezone.utc).isoformat(), "success": True,
    }
    push_batch(ftp, args.batch_dir, final_batch)
    log.info("Episode closed. Check the dashboard's Episodes tab for robot_id=%s", robot_id)
    return 0


if __name__ == "__main__":
    sys.exit(main())

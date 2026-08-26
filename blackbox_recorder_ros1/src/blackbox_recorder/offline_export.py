"""
Shared offline/air-gapped session export — used by both episode_recorder.py
(live topic-collector) and rosbag_exporter.py (post-hoc bag parser) so a
completed episode is captured the same way regardless of which node produced
it, whenever the live API is unreachable (or offline_mode is set explicitly).

Writes a session_*.zip (episode.json + manifest.json + optional bag file) to
a durable export_dir. Move the zip off the drone (SD card / USB) later and
upload it via the dashboard's "Upload Offline Session" flow, or POST it
directly to /api/episodes/import.
"""

import hashlib
import json
import os
import uuid
import zipfile
from datetime import datetime, timezone


def export_offline_session(episode, export_dir, bag_path=None):
    """Writes episode + manifest + optional bag to a zip. Returns (zip_path, session_id)."""
    if not os.path.exists(export_dir):
        os.makedirs(export_dir)

    session_id = str(uuid.uuid4())
    episode_bytes = json.dumps(episode, sort_keys=True).encode('utf-8')
    checksum = hashlib.sha256(episode_bytes).hexdigest()
    manifest = {
        'session_id': session_id,
        'robot_id': episode.get('robot_id'),
        'task_id': episode.get('task_id'),
        'checksum': checksum,
        'created_at': datetime.now(timezone.utc).isoformat(),
    }

    ts = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    robot_id = episode.get('robot_id', 'unknown')
    task_id = episode.get('task_id', 'unknown')
    zip_path = os.path.join(export_dir, 'session_%s_%s_%s.zip' % (robot_id, task_id, ts))

    zf = zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED)
    try:
        zf.writestr('episode.json', episode_bytes)
        zf.writestr('manifest.json', json.dumps(manifest, indent=2))
        if bag_path and os.path.exists(bag_path):
            zf.write(bag_path, os.path.join('bag', os.path.basename(bag_path)))
    finally:
        zf.close()

    return zip_path, session_id

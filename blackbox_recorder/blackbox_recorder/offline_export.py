"""
Shared offline/air-gapped session export — used by both episode_recorder.py
(live topic-collector) and rosbag_exporter.py (post-hoc bag parser) so a
completed episode is captured the same way regardless of which node produced
it, whenever the live API is unreachable (or offline_mode is set explicitly).

Writes a session_*.zip (episode.json + manifest.json + optional bag files) to
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


def export_offline_session(episode: dict, export_dir: str, bag_path: str = None) -> tuple:
    """Writes episode + manifest + optional bag to a zip. Returns (zip_path, session_id)."""
    os.makedirs(export_dir, exist_ok=True)

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
    zip_path = os.path.join(export_dir, f'session_{robot_id}_{task_id}_{ts}.zip')

    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
        zf.writestr('episode.json', episode_bytes)
        zf.writestr('manifest.json', json.dumps(manifest, indent=2))
        if bag_path and os.path.exists(bag_path):
            _add_bag_to_zip(zf, bag_path)

    return zip_path, session_id


def _add_bag_to_zip(zf: zipfile.ZipFile, bag_path: str) -> None:
    """rosbag2 bag_path is usually a directory (metadata.yaml + .db3/.mcap) — bundle it whole."""
    bag_arcroot = os.path.join('bag', os.path.basename(os.path.normpath(bag_path)))
    if os.path.isdir(bag_path):
        for root, _dirs, files in os.walk(bag_path):
            for fname in files:
                full = os.path.join(root, fname)
                rel = os.path.relpath(full, bag_path)
                zf.write(full, os.path.join(bag_arcroot, rel))
    else:
        zf.write(bag_path, bag_arcroot)

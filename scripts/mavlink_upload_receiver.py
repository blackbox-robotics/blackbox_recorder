#!/usr/bin/env python3
"""
MAVLink Upload Receiver — ground-station side. Receives files PUSHED by the
recorder over the MAVLink link (via MAVLink FTP write/upload —
https://mavlink.io/en/services/ftp.html) and relays each one to the BlackBox
API as it arrives. Only this receiver, sitting at the ground station where
internet actually exists, ever talks to the BlackBox API — the recorder
itself never does in this mode.

Two kinds of file arrive, handled differently:

  livebatch_<session_id>_<seq>.json — the normal case. The recorder
  (mavlink_upload_enabled + a live episode recording) periodically pushes
  whatever observations/actions accumulated since the last batch — the
  MAVLink-transport equivalent of the HTTP live-streaming path's
  per-observation POSTs (see episode_recorder.py's _flush_mavlink_batch).
  On the first batch for a session_id, this receiver opens a live episode
  (POST /episodes/start) the same way the HTTP path would; each batch's
  observations/actions are relayed to /episodes/:id/observations and
  /episodes/:id/actions; the batch marked is_final closes it
  (PATCH /episodes/:id/finish). session_id -> episode_id is persisted to
  disk so a receiver restart mid-episode doesn't lose track and open a
  duplicate.

  session_<robot_id>_<task_id>_<ts>.zip — the exceptional case. Only sent
  for a crash-recovered buffer where no live batching session existed (the
  node had just restarted) — see episode_recorder.py's _recover_buffer.
  Imported whole via POST /api/episodes/import, same as a manually
  retrieved offline session zip.

Run this on the ground station (wherever the MAVLink link and internet both
reach) — NOT on the vehicle.

Implements the minimum MAVLink FTP server opcode subset a real upload needs:
ResetSessions, CreateFile, WriteFile (out-of-order tolerant — writes go to
the correct offset regardless of arrival order, matching what a real client
does under packet loss), TerminateSession. No download/list support — this
receiver only ever accepts uploads, it never serves files back.

Requires: pymavlink (pip install pymavlink), requests.

Usage:
    python3 mavlink_upload_receiver.py \\
      --connection udpin:0.0.0.0:14552 \\
      --incoming-dir /var/blackbox/mavlink-incoming \\
      --api-url https://demo.bbrobotics.in/api \\
      --api-key pk_your_key

--connection needs its own forwarding target on the link, separate from the
passive telemetry ingest port — MAVLink FTP is stateful per session and this
receiver only speaks the write side of it.
"""

import argparse
import json
import logging
import os
import struct
import sys
import time

import requests
from pymavlink import mavutil
from pymavlink.mavftp_op import (
    FTP_OP,
    OP_Ack,
    OP_CreateFile,
    OP_Nack,
    OP_ResetSessions,
    OP_TerminateSession,
    OP_WriteFile,
)

log = logging.getLogger("mavlink_upload_receiver")

HEADER_FMT = "<HBBBBBBI"
HEADER_LEN = struct.calcsize(HEADER_FMT)

SESSION_ZIP_PREFIX = "session_"
SESSION_ZIP_SUFFIX = ".zip"
LIVE_BATCH_PREFIX = "livebatch_"
LIVE_BATCH_SUFFIX = ".json"

# FtpError codes this server needs to Nack with (mirrors pymavlink's own enum
# — not importing FtpError here since only Fail/FileProtected are needed and
# the raw ints keep this file's server logic self-contained).
_ERR_FAIL = 1
_ERR_FILE_PROTECTED = 9


def parse_op(data: bytes) -> FTP_OP:
    seq, session, opcode, size, req_opcode, burst_complete, _pad, offset = struct.unpack(
        HEADER_FMT, data[:HEADER_LEN]
    )
    payload = data[HEADER_LEN:HEADER_LEN + size] if size else None
    return FTP_OP(seq, session, opcode, size, req_opcode, burst_complete, offset, payload)


def send_op(master, target_system: int, target_component: int, op: FTP_OP) -> None:
    packed = op.pack()
    padded = bytes(packed) + b"\x00" * (251 - len(packed))
    master.mav.file_transfer_protocol_send(0, target_system, target_component, padded)


def import_zip_to_dashboard(api_url: str, api_key: str, filename: str, path: str) -> bool:
    try:
        with open(path, "rb") as f:
            resp = requests.post(
                f"{api_url.rstrip('/')}/episodes/import",
                headers={"X-API-Key": api_key},
                files={"archive": (filename, f, "application/zip")},
                timeout=60,
            )
    except (requests.RequestException, OSError) as e:
        log.error("Import POST failed for %s: %s", filename, e)
        return False

    if resp.status_code in (200, 201):
        body = resp.json()
        dup = body.get("duplicate", False)
        episode_id = body.get("data", {}).get("id", "unknown")
        log.info(
            "%s %s -> episode %s",
            "Already imported (duplicate)" if dup else "Imported",
            filename,
            episode_id,
        )
        return True

    log.error("Import rejected for %s: %s %s", filename, resp.status_code, resp.text)
    return False


class SessionMap:
    """Persists session_id -> episode_id across receiver restarts, so a
    restart mid-episode doesn't lose track and open a duplicate on the next
    batch for a session it already has an episode for."""

    def __init__(self, path: str):
        self.path = path
        try:
            with open(path) as f:
                self._map: dict = json.load(f)
        except (OSError, json.JSONDecodeError):
            self._map = {}

    def get(self, session_id: str):
        return self._map.get(session_id)

    def set(self, session_id: str, episode_id: str) -> None:
        self._map[session_id] = episode_id
        self._save()

    def discard(self, session_id: str) -> None:
        self._map.pop(session_id, None)
        self._save()

    def _save(self) -> None:
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._map, f)
        os.replace(tmp, self.path)


def relay_batch_to_dashboard(api_url: str, api_key: str, session_map: "SessionMap", batch: dict) -> bool:
    """Relays one live batch to the same endpoints the HTTP live-streaming
    path uses. Returns True if the batch was fully handled (safe to delete
    the local file) — False only when opening the episode itself failed,
    since nothing in the batch can be relayed without an episode_id. A
    failed individual observation/action POST is logged and skipped rather
    than aborting the whole batch — matches the recorder's own
    best-effort-live philosophy (the full zip stays the durability backstop,
    not this path)."""
    headers = {"X-API-Key": api_key}
    session_id = batch["session_id"]
    episode_id = session_map.get(session_id)

    if episode_id is None:
        try:
            resp = requests.post(
                f"{api_url.rstrip('/')}/episodes/start",
                headers=headers,
                json={
                    "robot_id": batch["robot_id"],
                    "task_id": batch["task_id"],
                    "start_time": batch["start_time"],
                },
                timeout=15,
            )
        except requests.RequestException as e:
            log.error("Failed to open live episode for session %s: %s", session_id, e)
            return False
        if resp.status_code != 201:
            log.error("Failed to open live episode for session %s: %s %s", session_id, resp.status_code, resp.text)
            return False
        episode_id = resp.json()["data"]["id"]
        session_map.set(session_id, episode_id)
        log.info("Opened live episode %s for MAVLink session %s", episode_id, session_id)

    for obs in batch.get("observations", []):
        try:
            r = requests.post(f"{api_url.rstrip('/')}/episodes/{episode_id}/observations", headers=headers, json=obs, timeout=10)
            if r.status_code not in (200, 201):
                log.warning("Observation POST rejected for episode %s: %s %s", episode_id, r.status_code, r.text)
        except requests.RequestException as e:
            log.warning("Observation POST failed for episode %s: %s", episode_id, e)

    for act in batch.get("actions", []):
        try:
            r = requests.post(f"{api_url.rstrip('/')}/episodes/{episode_id}/actions", headers=headers, json=act, timeout=10)
            if r.status_code not in (200, 201):
                log.warning("Action POST rejected for episode %s: %s %s", episode_id, r.status_code, r.text)
        except requests.RequestException as e:
            log.warning("Action POST failed for episode %s: %s", episode_id, e)

    log.info(
        "Relayed batch seq=%s for episode %s (%d observations, %d actions)%s",
        batch.get("seq"), episode_id, len(batch.get("observations", [])), len(batch.get("actions", [])),
        " [final]" if batch.get("is_final") else "",
    )

    if batch.get("is_final"):
        try:
            r = requests.patch(
                f"{api_url.rstrip('/')}/episodes/{episode_id}/finish",
                headers=headers,
                json={"end_time": batch.get("end_time"), "success": batch.get("success")},
                timeout=15,
            )
            if r.status_code == 200:
                log.info("Finished live episode %s (session %s)", episode_id, session_id)
            else:
                log.error(
                    "Could not finish episode %s: %s %s — it will stay shown as RECORDING until closed manually",
                    episode_id, r.status_code, r.text,
                )
        except requests.RequestException as e:
            log.error("Could not finish episode %s: %s — it will stay shown as RECORDING until closed manually", episode_id, e)
        session_map.discard(session_id)

        # Optional — only present when the sender attaches one (e.g. the
        # dummy demo script on a failed episode). The real recorder never
        # sends this; failures are reported separately in every other path
        # too (e.g. the dashboard's Findings panel), not auto-created from
        # episode success/failure.
        failure = batch.get("failure")
        if failure:
            try:
                r = requests.post(
                    f"{api_url.rstrip('/')}/episodes/{episode_id}/failures",
                    headers=headers,
                    json=failure,
                    timeout=15,
                )
                if r.status_code == 201:
                    log.info("Recorded failure for episode %s (%s)", episode_id, failure.get("failure_mode"))
                else:
                    log.error("Could not record failure for episode %s: %s %s", episode_id, r.status_code, r.text)
            except requests.RequestException as e:
                log.error("Could not record failure for episode %s: %s", episode_id, e)

    return True


class UploadSession:
    """One in-progress file upload, keyed by MAVLink FTP session id. Writes
    land at whatever offset the client sends — real clients under packet
    loss retry gaps out of order, so this must not assume sequential
    arrival."""

    def __init__(self, path: str):
        self.path = path
        self.fh = open(path, "r+b" if os.path.exists(path) else "w+b")
        self.max_offset_seen = 0

    def write(self, offset: int, data: bytes) -> None:
        self.fh.seek(offset)
        self.fh.write(data)
        self.max_offset_seen = max(self.max_offset_seen, offset + len(data))

    def close(self) -> None:
        self.fh.flush()
        self.fh.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", required=True, help="MAVLink connection string, e.g. udpin:0.0.0.0:14552")
    parser.add_argument("--incoming-dir", required=True, help="Local directory to stage received zips before import")
    parser.add_argument("--api-url", required=True)
    parser.add_argument("--api-key", required=True)
    parser.add_argument("--keep-files", action="store_true", help="Don't delete local zips after successful import")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    os.makedirs(args.incoming_dir, exist_ok=True)
    session_map = SessionMap(os.path.join(args.incoming_dir, ".mavlink_sessions.json"))

    master = mavutil.mavlink_connection(args.connection)
    log.info("Listening for MAVLink FTP uploads on %s -> %s", args.connection, args.incoming_dir)

    sessions: dict = {}  # MAVLink FTP session id -> UploadSession (in-progress file transfers)
    last_heartbeat = 0.0

    while True:
        now = time.time()
        if now - last_heartbeat > 1.0:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0,
            )
            last_heartbeat = now

        m = master.recv_match(type="FILE_TRANSFER_PROTOCOL", blocking=True, timeout=1.0)
        if m is None:
            continue

        req = parse_op(bytes(m.payload))
        log.debug("<- %s", req)
        target_system = m.get_srcSystem()
        target_component = m.get_srcComponent()

        if req.opcode == OP_ResetSessions:
            for s in sessions.values():
                s.close()
            sessions.clear()
            reply = FTP_OP(req.seq, req.session, OP_Ack, 0, req.opcode, 0, 0, None)

        elif req.opcode == OP_CreateFile:
            fname = req.payload.decode("ascii") if req.payload else ""
            safe_name = os.path.basename(fname)  # never trust a client-supplied path
            is_zip = safe_name.startswith(SESSION_ZIP_PREFIX) and safe_name.endswith(SESSION_ZIP_SUFFIX)
            is_batch = safe_name.startswith(LIVE_BATCH_PREFIX) and safe_name.endswith(LIVE_BATCH_SUFFIX)
            if not (is_zip or is_batch):
                log.warning("Refusing CreateFile for unexpected filename: %r", fname)
                reply = FTP_OP(req.seq, req.session, OP_Nack, 1, req.opcode, 0, 0, bytes([_ERR_FILE_PROTECTED]))
            else:
                dest = os.path.join(args.incoming_dir, safe_name)
                try:
                    sessions[req.session] = UploadSession(dest)
                    log.info("Receiving %s (session %d)", safe_name, req.session)
                    reply = FTP_OP(req.seq, req.session, OP_Ack, 0, req.opcode, 0, 0, None)
                except OSError as e:
                    log.error("Failed to open %s for writing: %s", dest, e)
                    reply = FTP_OP(req.seq, req.session, OP_Nack, 1, req.opcode, 0, 0, bytes([_ERR_FAIL]))

        elif req.opcode == OP_WriteFile:
            sess = sessions.get(req.session)
            if sess is None or req.payload is None:
                reply = FTP_OP(req.seq, req.session, OP_Nack, 1, req.opcode, 0, req.offset, bytes([_ERR_FAIL]))
            else:
                sess.write(req.offset, req.payload)
                # Ack echoes the same offset back — that's how the client's
                # write_recv_idx tracking (mavftp.py __handle_write_reply)
                # confirms which block landed.
                reply = FTP_OP(req.seq, req.session, OP_Ack, 0, req.opcode, 0, req.offset, None)

        elif req.opcode == OP_TerminateSession:
            sess = sessions.pop(req.session, None)
            reply = FTP_OP(req.seq, req.session, OP_Ack, 0, req.opcode, 0, 0, None)
            if sess is not None:
                sess.close()
                filename = os.path.basename(sess.path)
                log.info("Upload complete: %s (%d bytes)", filename, sess.max_offset_seen)

                if filename.startswith(LIVE_BATCH_PREFIX):
                    try:
                        with open(sess.path) as f:
                            batch = json.load(f)
                        handled = relay_batch_to_dashboard(args.api_url, args.api_key, session_map, batch)
                    except (OSError, json.JSONDecodeError, KeyError) as e:
                        log.error("Malformed batch file %s: %s", filename, e)
                        handled = False
                    if handled:
                        if not args.keep_files:
                            os.unlink(sess.path)
                    else:
                        log.warning("Batch relay failed — leaving %s in %s for manual retry", filename, args.incoming_dir)
                else:
                    if import_zip_to_dashboard(args.api_url, args.api_key, filename, sess.path):
                        if not args.keep_files:
                            os.unlink(sess.path)
                    else:
                        log.warning("Import failed — leaving %s in %s for manual retry", filename, args.incoming_dir)

        else:
            log.warning("Unhandled opcode %d — Nacking", req.opcode)
            reply = FTP_OP(req.seq, req.session, OP_Nack, 1, req.opcode, 0, 0, bytes([7]))  # UnknownCommand

        log.debug("-> %s", reply)
        send_op(master, target_system, target_component, reply)

    return 0


if __name__ == "__main__":
    sys.exit(main())

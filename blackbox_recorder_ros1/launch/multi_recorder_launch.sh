#!/usr/bin/env bash
# Multi-robot launcher for ROS 1 — roslaunch XML has no clean way to loop over
# a dynamic comma-separated list (unlike the ROS 2 Python launch file, which
# uses OpaqueFunction for this). Idiomatic ROS 1 answer: one roslaunch process
# per robot, each namespaced, run in parallel from a shell loop.
#
# Usage:
#   ./multi_recorder_launch.sh \
#     api_key:=pk_your_key \
#     robot_ids:="arm_01,arm_02,arm_03"
#
# Each recorder gets its own namespace — arm_01 resolves to
# /arm_01/joint_states, /arm_01/blackbox/task_event, etc., same as the
# ROS 2 multi-robot launcher.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

API_URL="https://www.bbrobotics.in/api"
API_KEY=""
ROBOT_IDS=""
OBS_INTERVAL_MS="100"
EXTRA_FLOAT_TOPICS=""

for arg in "$@"; do
  case "$arg" in
    api_url:=*) API_URL="${arg#api_url:=}" ;;
    api_key:=*) API_KEY="${arg#api_key:=}" ;;
    robot_ids:=*) ROBOT_IDS="${arg#robot_ids:=}" ;;
    observation_interval_ms:=*) OBS_INTERVAL_MS="${arg#observation_interval_ms:=}" ;;
    extra_float_topics:=*) EXTRA_FLOAT_TOPICS="${arg#extra_float_topics:=}" ;;
    *) echo "Unknown argument: $arg" >&2; exit 1 ;;
  esac
done

if [[ -z "$ROBOT_IDS" ]]; then
  echo "ERROR: robot_ids:= is required, e.g. robot_ids:=\"arm_01,arm_02\"" >&2
  exit 1
fi
if [[ -z "$API_KEY" ]]; then
  echo "ERROR: api_key:= is required" >&2
  exit 1
fi

PIDS=()
cleanup() {
  echo ""
  echo "Stopping all recorders..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup INT TERM EXIT

IFS=',' read -ra IDS <<< "$ROBOT_IDS"
for rid in "${IDS[@]}"; do
  rid="$(echo "$rid" | xargs)"  # trim whitespace
  [[ -z "$rid" ]] && continue
  echo "Launching recorder for robot: $rid"
  roslaunch blackbox_recorder recorder_launch.launch \
    ns:="$rid" \
    robot_id:="$rid" \
    api_key:="$API_KEY" \
    api_url:="$API_URL" \
    observation_interval_ms:="$OBS_INTERVAL_MS" \
    extra_float_topics:="$EXTRA_FLOAT_TOPICS" &
  PIDS+=("$!")
done

if [[ ${#PIDS[@]} -eq 0 ]]; then
  echo "WARNING: No robot_ids provided. No recorder nodes were started." >&2
  exit 1
fi

wait

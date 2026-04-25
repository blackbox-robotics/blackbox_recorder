#!/usr/bin/env bash
# =============================================================================
# BlackBox Robot Setup
# Installs the episode recorder and registers it as a systemd service that
# starts on boot and restarts automatically on crash.
#
# Run (fully automated):
#   BLACKBOX_API_URL=http://192.168.1.10:3001/api \
#   BLACKBOX_API_KEY=pk_... \
#   BLACKBOX_ROBOT_ID=<uuid> \
#   bash setup-robot.sh
#
# Run (interactive — will prompt for missing values):
#   bash setup-robot.sh
#
# To update an existing install, run the same command again.
# =============================================================================

set -euo pipefail

# ── Terminal colors ───────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${BLUE}[blackbox]${NC} $*"; }
ok()    { echo -e "${GREEN}[  ok  ]${NC} $*"; }
warn()  { echo -e "${YELLOW}[ warn ]${NC} $*"; }
die()   { echo -e "${RED}[error ]${NC} $*" >&2; exit 1; }
step()  { echo -e "\n${BOLD}── $* ──${NC}"; }

# ── Config (env vars override interactive prompts) ────────────────────────────
BLACKBOX_API_URL="${BLACKBOX_API_URL:-}"
BLACKBOX_API_KEY="${BLACKBOX_API_KEY:-}"
BLACKBOX_ROBOT_ID="${BLACKBOX_ROBOT_ID:-}"
BLACKBOX_WS="${BLACKBOX_WS:-$HOME/blackbox_ws}"
SERVICE_NAME="blackbox-recorder"

# ── Locate package source relative to this script ────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PKG_SRC="$REPO_ROOT/ros-nodes/blackbox_recorder"

# ── Helpers ───────────────────────────────────────────────────────────────────
prompt_plain() {
    local -n _ref=$1
    [[ -n "${_ref:-}" ]] && return
    read -rp "$2: " _ref
}
prompt_secret() {
    local -n _ref=$1
    [[ -n "${_ref:-}" ]] && return
    read -rsp "$2: " _ref; echo
}

detect_ros_distro() {
    # If already sourced, trust $ROS_DISTRO
    [[ -n "${ROS_DISTRO:-}" ]] && { echo "$ROS_DISTRO"; return; }
    # Otherwise scan /opt/ros for known distros in preference order
    for d in jazzy iron humble; do
        [[ -d "/opt/ros/$d" ]] && { echo "$d"; return; }
    done
    die "ROS 2 not found under /opt/ros. Install ROS 2 Humble or Iron first:\n  https://docs.ros.org/en/humble/Installation.html"
}

check_colcon() {
    command -v colcon &>/dev/null && return
    info "colcon not found — installing..."
    sudo apt-get install -y -q python3-colcon-common-extensions \
        || pip3 install colcon-common-extensions --quiet \
        || die "Cannot install colcon. Install it manually and re-run."
}

# ── Main ──────────────────────────────────────────────────────────────────────
main() {
    echo -e "\n${BOLD}BlackBox Episode Recorder — Robot Setup${NC}"
    echo    "════════════════════════════════════════"

    # ── 0. Sanity checks ─────────────────────────────────────────────────────
    [[ -d "$PKG_SRC" ]] \
        || die "blackbox_recorder package not found at:\n  $PKG_SRC\n  Run this script from within the BlackBox repo."

    command -v sudo &>/dev/null || die "sudo is required."

    # ── 1. Collect credentials ────────────────────────────────────────────────
    step "Credentials"
    prompt_plain  BLACKBOX_API_URL  "BlackBox backend URL (e.g. http://192.168.1.10:3001/api)"
    prompt_secret BLACKBOX_API_KEY  "API key (pk_...)"
    prompt_plain  BLACKBOX_ROBOT_ID "Robot UUID (from dashboard Settings)"

    [[ "$BLACKBOX_API_URL"  =~ ^https?:// ]] || die "API URL must start with http:// or https://"
    [[ "$BLACKBOX_API_KEY"  =~ ^pk_       ]] || die "API key must start with 'pk_'"
    [[ -n "$BLACKBOX_ROBOT_ID"            ]] || die "Robot UUID cannot be empty"

    # ── 2. Detect ROS 2 ───────────────────────────────────────────────────────
    step "ROS 2 environment"
    ROS_DISTRO_FOUND=$(detect_ros_distro)
    ROS_SETUP="/opt/ros/${ROS_DISTRO_FOUND}/setup.bash"
    ok "Distro: $ROS_DISTRO_FOUND  (setup: $ROS_SETUP)"

    # ── 3. System dependencies ────────────────────────────────────────────────
    step "Dependencies"
    if command -v apt-get &>/dev/null; then
        info "Installing python3-requests via apt..."
        sudo apt-get install -y -q python3-requests
    else
        info "apt not available — trying pip3..."
        pip3 install requests --quiet || warn "Could not install requests — assuming it is already present"
    fi
    check_colcon
    ok "Dependencies satisfied"

    # ── 4. Workspace & package ────────────────────────────────────────────────
    step "Workspace: $BLACKBOX_WS"
    mkdir -p "$BLACKBOX_WS/src"

    # Fresh copy of package (handles re-installs cleanly)
    rm -rf "$BLACKBOX_WS/src/blackbox_recorder"
    cp -r  "$PKG_SRC" "$BLACKBOX_WS/src/blackbox_recorder"
    ok "Package copied"

    # ── 5. Build ─────────────────────────────────────────────────────────────
    step "Build (colcon)"
    (
        # shellcheck source=/dev/null
        source "$ROS_SETUP"
        cd "$BLACKBOX_WS"
        colcon build \
            --packages-select blackbox_recorder \
            --event-handlers console_cohesion+
    )
    ok "Build complete"

    # ── 6. Write params file (mode 600 — contains API key) ───────────────────
    step "Configuration"
    PARAMS_FILE="$BLACKBOX_WS/recorder_params.yaml"
    cat > "$PARAMS_FILE" <<YAML
blackbox_episode_recorder:
  ros__parameters:
    api_url: "${BLACKBOX_API_URL}"
    api_key: "${BLACKBOX_API_KEY}"
    robot_id: "${BLACKBOX_ROBOT_ID}"
    observation_interval_ms: 100
    max_observations: 1000
    joint_states_topic: "/joint_states"
    ft_sensor_topic: "/ft_sensor"
    gripper_topic: "/gripper/state"
YAML
    chmod 600 "$PARAMS_FILE"
    ok "Config written  →  $PARAMS_FILE  (mode 600)"

    # ── 7. Write launcher script ──────────────────────────────────────────────
    # A dedicated script is cleaner than an inline bash -c in the unit file.
    LAUNCHER="$BLACKBOX_WS/start-recorder.sh"
    cat > "$LAUNCHER" <<BASH
#!/bin/bash
# Auto-generated by setup-robot.sh — do not edit.
# Re-run setup-robot.sh to regenerate.
set -e
# shellcheck source=/dev/null
source ${ROS_SETUP}
# shellcheck source=/dev/null
source ${BLACKBOX_WS}/install/setup.bash
exec ros2 run blackbox_recorder episode_recorder \\
    --ros-args --params-file ${PARAMS_FILE}
BASH
    chmod +x "$LAUNCHER"
    ok "Launcher written →  $LAUNCHER"

    # ── 8. Install systemd service ────────────────────────────────────────────
    step "systemd service: $SERVICE_NAME"
    SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
    CURRENT_USER="$(id -un)"

    sudo tee "$SERVICE_FILE" > /dev/null <<UNIT
[Unit]
Description=BlackBox Episode Recorder
Documentation=https://github.com/blackrobotics/blackbox
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${CURRENT_USER}
Environment=HOME=${HOME}
ExecStart=${LAUNCHER}
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=blackbox-recorder

[Install]
WantedBy=multi-user.target
UNIT

    sudo systemctl daemon-reload
    sudo systemctl enable  "$SERVICE_NAME"
    sudo systemctl restart "$SERVICE_NAME"
    ok "Service enabled and started"

    # ── 9. Verify ─────────────────────────────────────────────────────────────
    sleep 3
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        ok "Service is active (running)"
    else
        warn "Service did not reach active state — check logs below:"
        journalctl -u "$SERVICE_NAME" -n 20 --no-pager || true
    fi

    # ── Done ──────────────────────────────────────────────────────────────────
    echo -e "\n${BOLD}${GREEN}Setup complete.${NC}\n"
    echo -e "  ${BOLD}Status:${NC}     sudo systemctl status $SERVICE_NAME"
    echo -e "  ${BOLD}Live logs:${NC}  journalctl -u $SERVICE_NAME -f"
    echo -e "  ${BOLD}Stop:${NC}       sudo systemctl stop $SERVICE_NAME"
    echo -e "  ${BOLD}Update:${NC}     Re-run this script"
    echo -e "  ${BOLD}Uninstall:${NC}  sudo systemctl disable --now $SERVICE_NAME && sudo rm $SERVICE_FILE"
    echo ""
    echo -e "  ${BOLD}Verify topics:${NC}"
    echo -e "    source $ROS_SETUP && ros2 topic echo /blackbox/episode_status"
    echo ""
    echo -e "  ${BOLD}Send a test episode:${NC}"
    # shellcheck disable=SC2016
    echo -e "    source $ROS_SETUP && source $BLACKBOX_WS/install/setup.bash && \\"
    echo -e "    ros2 run blackbox_recorder task_event '{\"event\":\"start\",\"task_id\":\"test\"}'"
    echo -e "    ros2 run blackbox_recorder task_event '{\"event\":\"end\",\"success\":true}'"
    echo ""
}

main "$@"

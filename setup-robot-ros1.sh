#!/usr/bin/env bash
# =============================================================================
# BlackBox Robot Setup — ROS 1
#
# Installs the episode recorder and registers it as a systemd service that
# starts on boot and restarts on crash. ROS 1 counterpart to setup-robot.sh.
#
# Targets ROS 1 Noetic (Python 3). Melodic and older default to Python 2,
# which this package does not support (it uses pathlib, Python 3.4+ only).
# If you're on Melodic, either run rosdep/catkin under a Python 3 rospy build
# or use the ROS 2 package instead if migrating is on the table.
#
# Run (fully automated):
#   BLACKBOX_API_KEY=pk_... \
#   BLACKBOX_ROBOT_ID=<uuid> \
#   bash setup-robot-ros1.sh
#
# Run (interactive — will prompt for missing values):
#   bash setup-robot-ros1.sh
#
# To update an existing install, run the same command again.
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${BLUE}[blackbox]${NC} $*"; }
ok()    { echo -e "${GREEN}[  ok  ]${NC} $*"; }
warn()  { echo -e "${YELLOW}[ warn ]${NC} $*"; }
die()   { echo -e "${RED}[error ]${NC} $*" >&2; exit 1; }
step()  { echo -e "\n${BOLD}── $* ──${NC}"; }

# ── Config (env vars override interactive prompts) ────────────────────────────
BLACKBOX_API_URL="${BLACKBOX_API_URL:-https://www.bbrobotics.in/api}"
BLACKBOX_API_KEY="${BLACKBOX_API_KEY:-}"
BLACKBOX_ROBOT_ID="${BLACKBOX_ROBOT_ID:-}"
BLACKBOX_WS="${BLACKBOX_WS:-$HOME/blackbox_ws}"
SERVICE_NAME="blackbox-recorder"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$SCRIPT_DIR/blackbox_recorder_ros1"

prompt_secret() {
    local -n _ref=$1
    [[ -n "${_ref:-}" ]] && return
    read -rsp "$2: " _ref; echo
}

detect_ros_distro() {
    [[ -n "${ROS_DISTRO:-}" ]] && { echo "$ROS_DISTRO"; return; }
    for d in noetic melodic; do
        [[ -d "/opt/ros/$d" ]] && { echo "$d"; return; }
    done
    die "ROS 1 not found under /opt/ros. Install ROS 1 Noetic first:\n  http://wiki.ros.org/noetic/Installation"
}

check_catkin_tools() {
    command -v catkin_make &>/dev/null && return
    die "catkin_make not found. Install ros-\$ROS_DISTRO-catkin (usually part of ros-\$ROS_DISTRO-desktop)."
}

main() {
    echo -e "\n${BOLD}BlackBox Episode Recorder — Robot Setup (ROS 1)${NC}"
    echo    "════════════════════════════════════════"

    # ── 0. Sanity checks ─────────────────────────────────────────────────────
    [[ -d "$PKG_SRC" ]] \
        || die "blackbox_recorder_ros1 package not found at:\n  $PKG_SRC\n  Run this script from within the BlackBox repo."

    command -v sudo &>/dev/null || die "sudo is required."

    # ── 1. Collect credentials ────────────────────────────────────────────────
    step "Credentials"
    info "Find your API Key at: https://www.bbrobotics.in/settings"
    prompt_secret BLACKBOX_API_KEY  "Enter your API Key (Secret Key starting with pk_)"

    if [[ -z "$BLACKBOX_ROBOT_ID" ]]; then
        read -rp "Enter a unique ID for this robot (e.g. my-robot-01) [skip to auto-generate]: " BLACKBOX_ROBOT_ID
        if [[ -z "$BLACKBOX_ROBOT_ID" ]]; then
            if command -v uuidgen &>/dev/null; then
                BLACKBOX_ROBOT_ID=$(uuidgen | tr '[:upper:]' '[:lower:]')
            elif [[ -f /proc/sys/kernel/random/uuid ]]; then
                BLACKBOX_ROBOT_ID=$(cat /proc/sys/kernel/random/uuid)
            else
                BLACKBOX_ROBOT_ID="robot-$(date +%s)"
            fi
            info "Auto-generated Robot ID: $BLACKBOX_ROBOT_ID"
        fi
    fi

    [[ "$BLACKBOX_API_URL"  =~ ^https?:// ]] || die "API URL must start with http:// or https://"
    [[ "$BLACKBOX_API_KEY"  =~ ^pk_       ]] || die "API key must start with 'pk_'"
    [[ -n "$BLACKBOX_ROBOT_ID"            ]] || die "Robot ID cannot be empty"

    # ── 2. Detect ROS 1 ───────────────────────────────────────────────────────
    step "ROS 1 environment"
    ROS_DISTRO_FOUND=$(detect_ros_distro)
    ROS_SETUP="/opt/ros/${ROS_DISTRO_FOUND}/setup.bash"
    ok "Distro: $ROS_DISTRO_FOUND  (setup: $ROS_SETUP)"

    if [[ "$ROS_DISTRO_FOUND" == "melodic" ]]; then
        warn "Melodic defaults to Python 2 — this package requires Python 3 (pathlib)."
        warn "Proceeding, but the build will fail unless your rospy is Python 3."
    fi

    # ── 3. System dependencies ────────────────────────────────────────────────
    step "Dependencies"
    if command -v apt-get &>/dev/null; then
        info "Installing python3-requests via apt..."
        sudo apt-get install -y -q python3-requests "ros-${ROS_DISTRO_FOUND}-catkin"
    else
        info "apt not available — trying pip3..."
        pip3 install requests --quiet || warn "Could not install requests — assuming it is already present"
    fi
    check_catkin_tools
    ok "Dependencies satisfied"

    # ── 4. Workspace & package ────────────────────────────────────────────────
    step "Workspace: $BLACKBOX_WS"
    mkdir -p "$BLACKBOX_WS/src"

    rm -rf "$BLACKBOX_WS/src/blackbox_recorder"
    cp -r  "$PKG_SRC" "$BLACKBOX_WS/src/blackbox_recorder"
    ok "Package copied"

    # ── 5. Build ─────────────────────────────────────────────────────────────
    step "Build (catkin_make)"
    (
        set +u
        # shellcheck source=/dev/null
        source "$ROS_SETUP"
        set -u
        cd "$BLACKBOX_WS"
        catkin_make --pkg blackbox_recorder
    )
    ok "Build complete"

    # ── 6. Write params file (mode 600 — contains API key) ───────────────────
    step "Configuration"
    PARAMS_FILE="$BLACKBOX_WS/recorder_params.yaml"
    cat > "$PARAMS_FILE" <<YAML
api_url: "${BLACKBOX_API_URL}"
api_key: "${BLACKBOX_API_KEY}"
robot_id: "${BLACKBOX_ROBOT_ID}"
observation_interval_ms: 100
max_observations: 1000
joint_states_topic: "joint_states"
ft_sensor_topic: "ft_sensor"
gripper_topic: "gripper/state"
YAML
    chmod 600 "$PARAMS_FILE"
    ok "Config written  →  $PARAMS_FILE  (mode 600)"

    # ── 7. Write launcher script ──────────────────────────────────────────────
    LAUNCHER="$BLACKBOX_WS/start-recorder.sh"
    cat > "$LAUNCHER" <<BASH
#!/bin/bash
# Auto-generated by setup-robot-ros1.sh — do not edit.
# Re-run setup-robot-ros1.sh to regenerate.
set -e
# shellcheck source=/dev/null
source ${ROS_SETUP}
# shellcheck source=/dev/null
source ${BLACKBOX_WS}/devel/setup.bash
exec rosrun blackbox_recorder episode_recorder \\
    __name:=blackbox_episode_recorder \\
    _api_url:="${BLACKBOX_API_URL}" \\
    _api_key:="${BLACKBOX_API_KEY}" \\
    _robot_id:="${BLACKBOX_ROBOT_ID}"
BASH
    chmod +x "$LAUNCHER"
    ok "Launcher written →  $LAUNCHER"

    # ── 8. Install systemd service ────────────────────────────────────────────
    step "systemd service: $SERVICE_NAME"
    if ! [ -d /run/systemd/system ]; then
        warn "systemd not detected (PID 1 is not systemd)."
    fi
    SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
    CURRENT_USER="$(id -un)"

    sudo tee "$SERVICE_FILE" > /dev/null <<UNIT
[Unit]
Description=BlackBox Episode Recorder (ROS 1)
Documentation=https://github.com/blackbox-robotics/blackbox_recorder
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

    # Requires a roscore already running (ROS 1 has no equivalent of ROS 2's
    # daemonless discovery) — this unit does not start one for you, since a
    # shared roscore is normally owned by whatever launches the rest of the
    # robot's stack, not by BlackBox.
    warn "This service expects roscore to already be running before it starts."
    warn "If nothing else on this machine starts roscore, add that as a separate"
    warn "systemd unit or start it in your robot's own boot sequence."

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
    echo -e "    source $ROS_SETUP && rostopic echo blackbox/episode_status"
    echo ""
    echo -e "  ${BOLD}Send a test episode:${NC}"
    # shellcheck disable=SC2016
    echo -e "    source $ROS_SETUP && source $BLACKBOX_WS/devel/setup.bash && \\"
    echo -e "    rosrun blackbox_recorder task_event '{\"event\":\"start\",\"task_id\":\"test\"}'"
    echo -e "    rosrun blackbox_recorder task_event '{\"event\":\"end\",\"success\":true}'"
    echo ""
}

main "$@"

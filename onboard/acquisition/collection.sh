#!/usr/bin/env bash

set -Eeuo pipefail


# ============================================================
# Drone Ranger - UAV Sensor Data Collection
#
# Usage:
#
#.   first time run:chmod +x ~/natasha/drone_ranger/scripts/collection.sh
#   ./collection.sh [session_name]
#   ./collection.sh backyard_01
#
# ============================================================


# ------------------------------------------------------------
# Paths
# ------------------------------------------------------------

PROJECT_ROOT="$HOME/natasha/drone_ranger"

LAUNCH_FILE="$PROJECT_ROOT/launch/uav_sensor_stack.launch.py"

BAG_ROOT="$PROJECT_ROOT/rosbags"

LOG_ROOT="$PROJECT_ROOT/logs"


# ------------------------------------------------------------
# Session name
# ------------------------------------------------------------

SESSION_NAME="${1:-test}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

RUN_NAME="${SESSION_NAME}_${TIMESTAMP}"

BAG_DIR="$BAG_ROOT/$RUN_NAME"

LOG_DIR="$LOG_ROOT/$RUN_NAME"


mkdir -p "$BAG_ROOT"
mkdir -p "$LOG_DIR"


# ------------------------------------------------------------
# Save console output
# ------------------------------------------------------------

RUN_LOG="$LOG_DIR/collection.log"

exec > >(tee -a "$RUN_LOG") 2>&1


echo
echo "============================================================"
echo " Drone Ranger - UAV Sensor Data Collection"
echo "============================================================"
echo
echo "Session:   $SESSION_NAME"
echo "Timestamp: $TIMESTAMP"
echo
echo "Bag:"
echo "  $BAG_DIR"
echo
echo "Log:"
echo "  $RUN_LOG"
echo


# ------------------------------------------------------------
# ROS 2 environment
# ------------------------------------------------------------

echo "[1/5] Loading ROS 2 Humble..."

source /opt/ros/humble/setup.bash


# ------------------------------------------------------------
# OAK Python environment
# ------------------------------------------------------------

echo "[2/5] Loading OAK-D Python environment..."

source "$HOME/oak34_ros/bin/activate"


# ------------------------------------------------------------
# YDLIDAR workspace
# ------------------------------------------------------------

echo "[3/5] Loading YDLIDAR workspace..."

source "$HOME/lidar_T-mini-plus/yahboomcar_ws/install/setup.bash"


# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------

STACK_PID=""

cleanup()
{
    echo
    echo "============================================================"
    echo " Stopping UAV sensor stack..."
    echo "============================================================"

    if [[ -n "${STACK_PID}" ]]; then
        if kill -0 "$STACK_PID" 2>/dev/null; then

            echo "Stopping ROS launch process: $STACK_PID"

            kill -INT "$STACK_PID" 2>/dev/null || true

            wait "$STACK_PID" 2>/dev/null || true
        fi
    fi

    echo
    echo "Sensor stack stopped."
    echo "Collection finished."
    echo
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM


# ------------------------------------------------------------
# Start sensor stack
# ------------------------------------------------------------

echo
echo "[4/5] Starting UAV sensor stack..."
echo

ros2 launch "$LAUNCH_FILE" &

STACK_PID=$!

echo
echo "ROS launch PID: $STACK_PID"
echo


# ------------------------------------------------------------
# Wait for sensor topics
# ------------------------------------------------------------

echo "Waiting for sensor topics..."

REQUIRED_TOPICS=(
    "/oak/rgb/image_raw"
    "/oak/depth/image_raw"
    "/scan"
)

MAX_WAIT=30

for ((i=1; i<=MAX_WAIT; i++)); do

    TOPICS="$(ros2 topic list 2>/dev/null || true)"

    ALL_READY=true

    for topic in "${REQUIRED_TOPICS[@]}"; do

        if ! grep -qx "$topic" <<< "$TOPICS"; then
            ALL_READY=false
            break
        fi

    done

    if $ALL_READY; then
        echo
        echo "[OK] Required sensor topics detected."
        break
    fi

    printf "\rWaiting... %2d / %d s" "$i" "$MAX_WAIT"

    sleep 1

done


echo
echo


# ------------------------------------------------------------
# Final status
# ------------------------------------------------------------

echo "Current topics:"
echo

ros2 topic list | grep -E \
"oak|scan|tf|foxglove" \
|| true

echo


# ------------------------------------------------------------
# ROS bag
# ------------------------------------------------------------

echo "[5/5] Starting ROS bag recording..."
echo

echo "Recording:"
echo
echo "  /oak/rgb/image_raw"
echo "  /oak/rgb/camera_info"
echo "  /oak/depth/image_raw"
echo "  /oak/depth/camera_info"
echo "  /scan"
echo "  /tf"
echo "  /tf_static"
echo

echo "Output:"
echo "  $BAG_DIR"
echo

echo "------------------------------------------------------------"
echo " Press Ctrl+C to stop recording"
echo "------------------------------------------------------------"
echo


ros2 bag record \
    -o "$BAG_DIR" \
    /oak/rgb/image_raw \
    /oak/rgb/camera_info \
    /oak/depth/image_raw \
    /oak/depth/camera_info \
    /scan \
    /tf \
    /tf_static
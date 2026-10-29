#!/usr/bin/env bash
# =============================================================================
# stop.bash - stop the swarm stack started by sim.launch.py or real.launch.py
#
#   ./stop.bash            auto-detect platform (SWARM_REAL_HW=1 -> real, else sim)
#   ./stop.bash --sim      force sim mode   (Gazebo, SITL, MAVROS, bridges, TFs)
#   ./stop.bash --real     force real mode  (MAVROS, TFs only)
#   ./stop.bash --force    skip the "is any drone armed?" safety check
#
# Safety: refuses to stop anything while a drone reports armed. On a real drone,
# killing MAVROS mid-flight leaves the FCU holding its last GUIDED target with no
# script and no companion link. In sim, killing SITL freezes the drones mid-air.
# Mission scripts (circle_swarm.py, ...) are not touched; stop them with Ctrl+C.
# =============================================================================

MODE=""
FORCE=0
for arg in "$@"; do
    case "$arg" in
        --sim)   MODE="sim" ;;
        --real)  MODE="real" ;;
        --force) FORCE=1 ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "Unknown option: $arg (see --help)"; exit 2 ;;
    esac
done
if [ -z "$MODE" ]; then
    if [ "${SWARM_REAL_HW:-0}" = "1" ]; then MODE="real"; else MODE="sim"; fi
fi

# Processes started by each platform's launch file
COMMON='ros2 launch swarm|mavros_node|ros2 launch mavros|static_transform_publisher'
SIM_ONLY='sim_vehicle|arducopter|mavproxy|gz sim|ruby.*gz|parameter_bridge|ros_gz_sim|ros_gz_bridge'
if [ "$MODE" = "sim" ]; then PATTERN="$COMMON|$SIM_ONLY"; else PATTERN="$COMMON"; fi

echo "[stop] Platform: $MODE"

# PIDs of this script and every shell above it - never signal those
SELF_PIDS=" $$ "
p=$PPID
while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do
    SELF_PIDS="$SELF_PIDS$p "
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
done

targets() {     # matching PIDs, excluding ourselves and our parents
    for pid in $(pgrep -f "$PATTERN" 2>/dev/null); do
        case "$SELF_PIDS" in *" $pid "*) ;; *) echo "$pid" ;; esac
    done
}
running() { [ -n "$(targets)" ]; }

if ! running; then
    echo "[stop] Nothing running."
    exit 0
fi

# --- Safety check: any drone armed? -----------------------------------------
if [ "$FORCE" -eq 0 ]; then
    if command -v ros2 >/dev/null 2>&1; then
        TOPICS=$(timeout 5 ros2 topic list 2>/dev/null | grep -E '/mavros/state$')
        ARMED=""
        CHECK_FAILED=0
        for t in $TOPICS; do
            OUT=$(timeout 4 ros2 topic echo --once "$t" 2>/dev/null)
            if [ -z "$OUT" ]; then
                CHECK_FAILED=1
            elif echo "$OUT" | grep -q "armed: true"; then
                ARMED="$ARMED ${t%%/mavros/state}"
            fi
        done
        if [ -n "$ARMED" ]; then
            echo "[stop] REFUSING: armed drone(s):$ARMED"
            echo "[stop] Land them first (or use --force if you really mean it)."
            exit 1
        fi
        if [ "$MODE" = "real" ] && { [ -z "$TOPICS" ] || [ "$CHECK_FAILED" -eq 1 ]; }; then
            echo "[stop] REFUSING: could not confirm all drones are disarmed (no state received)."
            echo "[stop] Check the drones, then re-run with --force."
            exit 1
        fi
    elif [ "$MODE" = "real" ]; then
        echo "[stop] REFUSING: 'ros2' not found, so armed state can't be checked. Source the workspace or use --force."
        exit 1
    else
        echo "[stop] Warning: 'ros2' not found; skipping armed check (sim)."
    fi
fi

# --- Stop: Ctrl+C -> SIGTERM -> SIGKILL, waiting between each -----------------
stop_with() {   # $1 = signal, $2 = seconds to wait for exit
    local pids
    pids=$(targets)
    [ -n "$pids" ] && kill "-$1" $pids 2>/dev/null
    for _ in $(seq 1 "$2"); do
        running || return 0
        sleep 1
    done
    return 1
}

echo "[stop] Stopping..."
stop_with INT 5 || stop_with TERM 3 || stop_with KILL 2

ros2 daemon stop >/dev/null 2>&1   # clear stale ROS discovery info (harmless if absent)

if running; then
    echo "[stop] Still running:"
    for pid in $(targets); do ps -o pid=,args= -p "$pid"; done
    exit 1
fi
echo "[stop] All clear."
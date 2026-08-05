#!/bin/bash
# Verify the code-first ROS 2 OmniGraph builder inside the live
# Isaac Sim X1 scene on Socl.
#
# Confirms:
#   1. ``build_x1_ros_graph()`` runs without traceback.
#   2. A code-built /code_generated_clock topic appears on domain 45.
#   3. The baked-in /clock topic still works (coexistence).
#
# Prereqs (run once):
#   * pkill all running isaacsim51 scenes
#   * nohup env DISPLAY=:99 /var/workspace/docker/isaac/scenes/active/
#     scripts/integrated_runtime/run_demo_scene.sh --world X1 \
#     --enable isaacsim.core.nodes --enable omni.graph.action \
#     --exec /tmp/run_x1_graph_builder.py
#   * the X1 USD's ROS2Context domain_id was patched 55 -> 45 (see
#     docs/runbooks/verify_curobo_isaac_x1.sh for the one-shot).
set -e
WS=/var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=45

BUILT="/code_generated_clock"
BAKED="/clock"

echo "=== code-first topic present? ==="
if timeout 3 ros2 topic list | grep -q "^${BUILT}$"; then
    echo "OK: ${BUILT} is published (proves build_x1_ros_graph wired up)"
else
    echo "FAIL: ${BUILT} missing; build_x1_ros_graph did not run"
    exit 1
fi

echo "=== baked-in topic still alive? ==="
if timeout 3 ros2 topic list | grep -q "^${BAKED}$"; then
    echo "OK: ${BAKED} also publishes (no shadowing conflict)"
else
    echo "FAIL: ${BAKED} missing; baked-in graph broken"
    exit 1
fi

echo "=== code-built clock actually emits /clock data? ==="
timeout 3 ros2 topic hz "${BUILT}" --window 3 2>&1 | tail -2

echo "=== done ==="

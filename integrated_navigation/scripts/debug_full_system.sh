#!/bin/bash
# ===================================================
# Integrated Navigation Debug Launch Script
# Gazebo + Faster-LIO + INS + DWA Navigation + Kakao API
# ===================================================

echo "Starting Integrated Navigation System..."

# 1. Kakao API Debug (Map 초기화)
echo "[1/6] Launching Kakao API Debug..."
roslaunch kakao_api debug_kakao.launch rviz:=false &
KAKAO_PID=$!

# WebSocket 연결 대기
echo "Waiting for WebSocket servers..."
for i in {1..20}; do
    if nc -z localhost 8765 2>/dev/null && nc -z localhost 8766 2>/dev/null; then
        echo "WebSocket servers ready!"
        break
    fi
    sleep 0.5
done
sleep 2

# 2. Gazebo Spawn
echo "[2/6] Launching Gazebo..."
roslaunch dwa "gazebo_spawn(empty_world).launch" z_position:=0.2 &
# roslaunch dwa "gazebo_spawn.launch" z_position:=0.2 &
GAZEBO_PID=$!
sleep 2

# 3. INS Fusion (GPS + IMU) - Launch FIRST to initialize datum
echo "[3/6] Launching INS Fusion..."
roslaunch ins ins_fusion.launch &
INS_PID=$!
sleep 2

# 4. Faster-LIO Mapping - Launch AFTER Gazebo is ready
echo "[4/6] Launching Faster-LIO..."
roslaunch faster_lio mapping_ouster32.launch rviz:=false &
FASTLIO_PID=$!
sleep 2

# 5. DWA Navigation
echo "[5/6] Launching DWA Navigation..."
roslaunch dwa dwa_navigation.launch enable_rviz:=false &
DWA_PID=$!

# 6. RViz 시각화
echo "[6/6] Launching RViz..."
rosrun rviz rviz -d $(rospack find integrated_navigation)/rviz/integrated_navigation.rviz &
RVIZ_PID=$!

echo "All systems launched!"

# Cleanup on exit
trap "echo 'Shutting down...'; kill $KAKAO_PID $GAZEBO_PID $FASTLIO_PID $TF_PID $INS_PID $DWA_PID $RVIZ_PID 2>/dev/null; exit" SIGINT SIGTERM

wait

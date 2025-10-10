#!/bin/bash
# ===================================================
# Integrated Navigation Debug Launch Script
# Gazebo + Faster-LIO + INS + DWA Navigation + Kakao API
# ===================================================

echo "Starting Integrated Navigation System..."

# 1. Gazebo Spawn
echo "[1/6] Launching Gazebo..."
roslaunch gazebo_simulation gazebo_spawn.launch world_name:="empty" &
# roslaunch gazebo_simulation gazebo_spawn_TF.launch world_name:="empty" &
GAZEBO_PID=$!
sleep 3

# 2. INS Fusion (GPS + IMU) - Launch FIRST to initialize datum
echo "[2/6] Launching INS Fusion..."
roslaunch ins ins.launch &
INS_PID=$!
echo "⏳ Waiting for INS to initialize datum and publish utm→map TF..."
sleep 5

# 3. Kakao API Debug (INS TF 감지 후 실행)
echo "[3/6] Launching Kakao API..."
roslaunch kakao_api debug_kakao_api.launch rviz:=false &
KAKAO_PID=$!

# WebSocket 연결 대기
echo "⏳ Waiting for WebSocket server..."
for i in {1..20}; do
    if nc -z localhost 8765 2>/dev/null; then
        echo "✅ WebSocket server ready!"
        break
    fi
    sleep 0.5
done
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

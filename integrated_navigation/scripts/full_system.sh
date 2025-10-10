#!/bin/bash
# ===================================================
# Integrated Navigation Debug Launch Script
# Gazebo + Faster-LIO + FRNet + INS + DWA Navigation + Kakao API
# ===================================================

echo "Starting Integrated Navigation System..."

# # 1. INS Fusion (GPS + IMU) - Launch FIRST to initialize datum
# echo "[1/7] Launching INS Fusion..."
# roslaunch ins ins_faster_lio.launch &
# INS_PID=$!
# echo "⏳ Waiting for INS to initialize datum and publish utm→map TF..."
# sleep 5

# # 2. Faster-LIO Mapping - Launch AFTER Gazebo is ready
# echo "[2/7] Launching Faster-LIO..."
# roslaunch faster_lio mapping_ouster32.launch rviz:=false &
# FASTLIO_PID=$!
# sleep 2

# 1-2. INS Fusion (GPS + IMU) - Launch FIRST to initialize datum
echo "[1/7] Launching INS Fusion..."
roslaunch ins ins_wheel.launch &
INS_PID=$!
echo "⏳ Waiting for INS to initialize datum and publish utm→map TF..."
sleep 5


# 3. Kakao API Debug (INS TF 감지 후 실행)
echo "[3/7] Launching Kakao API..."
roslaunch kakao_api kakao_api.launch &
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

# 4. FRNet Segmentation
echo "[4/7] Launching FRNet..."
roslaunch frnet_ros frnet.launch enable_rviz:=false &
FRNET_PID=$!
sleep 2

# 5. DWA Navigation (with Waypoint Manager for Kakao API)
echo "[5/7] Launching DWA Navigation..."
roslaunch dwa dwa_navigation.launch enable_rviz:=false odom_topic:=/faster_lio/odom enable_waypoint_manager:=true lidar_topic:=/FRNet/points &
# roslaunch dwa dwa_navigation.launch enable_rviz:=false odom_topic:=/faster_lio/odom enable_waypoint_manager:=true &
DWA_PID=$!

# 6. RViz 시각화
echo "[6/7] Launching RViz..."
rosrun rviz rviz -d $(rospack find integrated_navigation)/rviz/integrated_navigation.rviz &
RVIZ_PID=$!

# 7. Robot Base 실행
echo "[7/7] Launching Robot_Base..."
roslaunch husky_base base.launch port:=/dev/ttyUSB0 &
Robot_Base_PID=$!

echo "All systems launched!"

# Cleanup on exit
trap "echo 'Shutting down...'; kill $KAKAO_PID $FASTLIO_PID $FRNET_PID $TF_PID $INS_PID $DWA_PID $RVIZ_PID 2>/dev/null; exit" SIGINT SIGTERM

wait

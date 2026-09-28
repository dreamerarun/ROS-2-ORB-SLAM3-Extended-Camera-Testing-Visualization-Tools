#!/bin/bash
# Script to cleanly restart ORB-SLAM3 live camera system

echo "=========================================="
echo "Restarting ORB-SLAM3 Live Camera System"
echo "=========================================="

# Kill any existing processes
echo "Stopping existing nodes..."
pkill -9 mono_node_cpp 2>/dev/null
pkill -9 live_mono_camera 2>/dev/null
pkill -9 python3 2>/dev/null

# Wait for processes to terminate
echo "Waiting for cleanup..."
sleep 2

# Clear any stuck ROS2 processes
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null

echo "Starting nodes..."
echo ""
echo "=========================================="
echo "INSTRUCTIONS:"
echo "=========================================="
echo "1. Wait for C++ node to say 'Waiting to finish handshake'"
echo "2. Python node will auto-connect"
echo "3. Point camera at TEXTURED surfaces (not blank walls)"
echo "4. Move camera SLOWLY side-to-side"
echo "5. Wait for 'INITIALIZING' -> 'TRACKING'"
echo "6. Press Q in any window to quit"
echo "=========================================="
echo ""

# Start in separate terminal windows if available
if command -v gnome-terminal &> /dev/null; then
    echo "Launching in separate terminals..."
    
    # Terminal 1: C++ node
    gnome-terminal -- bash -c "
        echo '=== C++ SLAM Node ===';
        ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp;
        echo 'C++ node exited. Press Enter to close...';
        read
    " &
    
    sleep 3
    
    # Terminal 2: Python camera
    gnome-terminal -- bash -c "
        echo '=== Python Camera Driver ===';
        ros2 run ros2_orb_slam3 live_mono_camera.py --ros-args \
            -p settings_name:=Webcam \
            -p camera_id:=0 \
            -p fps:=20 \
            -p width:=640 \
            -p height:=480 \
            -p show_camera:=True;
        echo 'Camera driver exited. Press Enter to close...';
        read
    " &
    
elif command -v xterm &> /dev/null; then
    echo "Launching in xterm..."
    xterm -hold -e "ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp" &
    sleep 3
    xterm -hold -e "ros2 run ros2_orb_slam3 live_mono_camera.py --ros-args -p settings_name:=Webcam -p camera_id:=0 -p fps:=20 -p width:=640 -p height:=480 -p show_camera:=True" &
else
    echo "ERROR: No terminal emulator found (gnome-terminal or xterm)"
    echo "Please run manually in two terminals:"
    echo ""
    echo "Terminal 1:"
    echo "  ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp"
    echo ""
    echo "Terminal 2 (after Terminal 1 shows 'Waiting to finish handshake'):"
    echo "  ros2 run ros2_orb_slam3 live_mono_camera.py --ros-args \\"
    echo "      -p settings_name:=Webcam \\"
    echo "      -p camera_id:=0 \\"
    echo "      -p fps:=20 \\"
    echo "      -p width:=640 \\"
    echo "      -p height:=480 \\"
    echo "      -p show_camera:=True"
fi

echo ""
echo "Nodes launched!"

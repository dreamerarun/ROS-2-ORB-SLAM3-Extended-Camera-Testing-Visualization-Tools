#!/usr/bin/env python3
"""
Live Camera Feed Driver for ORB-SLAM3 with Real-time Visualization
Compatible with ros2_orb_slam3 mono_node_cpp

Author: Based on mono_driver_node.py by Azmyin Md. Kamal
Modified for live camera feed and real-time visualization

Usage:
    ros2 run ros2_orb_slam3 live_mono_camera.py --ros-args \
        -p settings_name:=EuRoC \
        -p camera_id:=0 \
        -p fps:=30
"""

import sys
import time
import numpy as np
import cv2
from pathlib import Path
from threading import Thread, Lock

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class LiveMonoCamera(Node):
    def __init__(self, node_name="live_mono_camera"):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("fps", 30)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        
        # Get parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.camera_id = int(self.get_parameter('camera_id').value)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        
        # State variables
        self.send_config = True
        self.camera_ready = False
        self.current_pose = None
        self.trajectory = []
        self.lock = Lock()
        
        # CV Bridge
        self.br = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera {self.camera_id}")
            sys.exit(1)
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps} FPS")
        
        # ROS2 Topics
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.sub_pose_name = "/orb_slam3/camera_pose"
        
        # Publishers
        self.publish_exp_config_ = self.create_publisher(
            String, self.pub_exp_config_name, 1
        )
        self.publish_img_msg_ = self.create_publisher(
            Image, self.pub_img_to_agent_name, 1
        )
        self.publish_timestep_msg_ = self.create_publisher(
            Float64, self.pub_timestep_to_agent_name, 1
        )
        
        # Subscribers
        self.subscribe_exp_ack_ = self.create_subscription(
            String, self.sub_exp_ack_name, self.ack_callback, 10
        )
        self.subscribe_pose_ = self.create_subscription(
            PoseStamped, self.sub_pose_name, self.pose_callback, 10
        )
        
        # Frame counter
        self.frame_id = 0
        self.exp_config_msg = self.settings_name
        
        # Visualization
        self.viz_running = True
        self.viz_thread = Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info("Live Camera Driver initialized")
        self.get_logger().info(f"Configuration: {self.exp_config_msg}")
        self.get_logger().info("Waiting for handshake with C++ node...")
    
    def ack_callback(self, msg):
        """Handle acknowledgment from C++ node"""
        self.get_logger().info(f"Received: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False
            self.camera_ready = True
            self.get_logger().info("Handshake complete! Starting camera feed...")
    
    def pose_callback(self, msg):
        """Process camera pose updates"""
        with self.lock:
            position = msg.pose.position
            self.current_pose = {
                'x': position.x,
                'y': position.y,
                'z': position.z,
                'timestamp': time.time()
            }
            self.trajectory.append([position.x, position.y, position.z])
            if len(self.trajectory) > 1000:
                self.trajectory.pop(0)
    
    def handshake_with_cpp_node(self):
        """Send configuration to C++ node"""
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    
    def send_frame(self):
        """Capture and send a single frame"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return False
        
        # Convert to grayscale (ORB-SLAM3 typically uses grayscale)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Create timestamp (nanoseconds since epoch)
        timestamp = time.time_ns() / 1e9  # Convert to seconds with decimals
        
        try:
            # Create ROS messages
            img_msg = self.br.cv2_to_imgmsg(gray_frame, encoding="mono8")
            timestep_msg = Float64()
            timestep_msg.data = timestamp
            
            # Publish (timestep first, then image)
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg)
            
            self.frame_id += 1
            return True
            
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return False
    
    def draw_trajectory_2d(self, width=800, height=600, scale=100):
        """Draw 2D trajectory map"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        with self.lock:
            if len(self.trajectory) < 2:
                # Draw grid even without trajectory
                for i in range(0, width, 50):
                    cv2.line(img, (i, 0), (i, height), (30, 30, 30), 1)
                for i in range(0, height, 50):
                    cv2.line(img, (0, i), (width, i), (30, 30, 30), 1)
                
                cv2.putText(img, "Waiting for SLAM initialization...", 
                           (width//2 - 200, height//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                return img
            
            trajectory = np.array(self.trajectory)
        
        center_x = width // 2
        center_y = height // 2
        
        # Draw grid
        for i in range(0, width, 50):
            cv2.line(img, (i, 0), (i, height), (30, 30, 30), 1)
        for i in range(0, height, 50):
            cv2.line(img, (0, i), (width, i), (30, 30, 30), 1)
        
        # Draw trajectory with color gradient
        for i in range(1, len(trajectory)):
            pt1 = (
                int(center_x + trajectory[i-1][0] * scale),
                int(center_y - trajectory[i-1][2] * scale)
            )
            pt2 = (
                int(center_x + trajectory[i][0] * scale),
                int(center_y - trajectory[i][2] * scale)
            )
            
            color_ratio = i / len(trajectory)
            color = (0, int(255 * color_ratio), int(255 * (1 - color_ratio)))
            cv2.line(img, pt1, pt2, color, 2)
        
        # Draw current position
        if self.current_pose:
            curr_pt = (
                int(center_x + self.current_pose['x'] * scale),
                int(center_y - self.current_pose['z'] * scale)
            )
            cv2.circle(img, curr_pt, 8, (0, 255, 0), -1)
            cv2.circle(img, curr_pt, 12, (0, 255, 0), 2)
        
        # Info overlay
        info = [
            f"Trajectory Points: {len(self.trajectory)}",
            f"Scale: {scale}x",
            f"Frame: {self.frame_id}",
        ]
        
        if self.current_pose:
            info.extend([
                f"X: {self.current_pose['x']:.3f}m",
                f"Y: {self.current_pose['y']:.3f}m",
                f"Z: {self.current_pose['z']:.3f}m"
            ])
        
        y_offset = 25
        for text in info:
            cv2.putText(img, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_offset += 30
        
        # Instructions
        cv2.putText(img, "Press 'Q' to quit | 'R' to reset | 'S' to save",
                   (10, height - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return img
    
    def visualization_loop(self):
        """Real-time visualization thread"""
        cv2.namedWindow('ORB-SLAM3: Trajectory Map', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ORB-SLAM3: Trajectory Map', 800, 600)
        
        while self.viz_running and rclpy.ok():
            trajectory_img = self.draw_trajectory_2d()
            cv2.imshow('ORB-SLAM3: Trajectory Map', trajectory_img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Quit requested")
                self.viz_running = False
                break
            elif key == ord('r'):
                with self.lock:
                    self.trajectory = []
                self.get_logger().info("Trajectory reset")
            elif key == ord('s'):
                self.save_trajectory()
        
        cv2.destroyAllWindows()
    
    def save_trajectory(self):
        """Save trajectory to file"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{timestamp}.txt"
        
        with self.lock:
            if len(self.trajectory) == 0:
                self.get_logger().warn("No trajectory to save")
                return
            trajectory = np.array(self.trajectory)
        
        np.savetxt(filename, trajectory, fmt='%.6f',
                  header='x y z', comments='')
        self.get_logger().info(f"Trajectory saved to {filename}")
    
    def cleanup(self):
        """Cleanup resources"""
        self.viz_running = False
        if self.viz_thread.is_alive():
            self.viz_thread.join(timeout=2)
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
    
    def destroy_node(self):
        """Override destroy to cleanup"""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiveMonoCamera("live_mono_camera")
    
    # Create rate controller
    rate = node.create_rate(node.fps)
    
    # Handshake loop
    node.get_logger().info("Starting handshake...")
    while node.send_config and rclpy.ok():
        node.handshake_with_cpp_node()
        rclpy.spin_once(node, timeout_sec=0.01)
        if not node.send_config:
            break
    
    if not node.camera_ready:
        node.get_logger().error("Handshake failed!")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Main camera loop
    node.get_logger().info("Starting camera feed...")
    try:
        while rclpy.ok() and node.viz_running:
            # Send frame
            if not node.send_frame():
                break
            
            # Process callbacks
            rclpy.spin_once(node, timeout_sec=0.001)
            
            # Maintain FPS
            rate.sleep()
            
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

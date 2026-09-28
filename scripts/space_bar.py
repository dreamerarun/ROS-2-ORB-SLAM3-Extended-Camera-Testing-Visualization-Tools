#!/usr/bin/env python3
"""
Live Camera Feed with Spacebar Frame Capture for SLAM Initialization
Continuously captures frames - press SPACEBAR to send frames to ORB-SLAM3

Author: Modified for spacebar-triggered frame sending
"""

import sys
import time
import numpy as np
import cv2
from pathlib import Path
from threading import Thread, Lock
from collections import deque
import math

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class LiveMonoCameraSpacebar(Node):
    def __init__(self, node_name="live_mono_camera"):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter("settings_name", "Webcam_Sensitive")
        self.declare_parameter("camera_id", 4)
        self.declare_parameter("fps", 20)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("show_camera", True)
        
        # Get parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.camera_id = int(self.get_parameter('camera_id').value)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.show_camera = bool(self.get_parameter('show_camera').value)
        
        # State variables
        self.send_config = True
        self.camera_ready = False
        self.slam_initialized = False
        self.current_pose = None
        self.trajectory = []
        self.lock = Lock()
        
        # Frame sending control
        self.send_frames = False  # Only send when spacebar is pressed
        self.frame_capture_enabled = False  # Continuous capture flag
        
        # Performance monitoring
        self.frame_times = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.actual_fps = 0.0
        self.frames_sent = 0
        
        # CV Bridge
        self.br = CvBridge()
        
        # Initialize camera
        self.get_logger().info(f"Attempting to open camera {self.camera_id}...")
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
        
        # Current frame storage
        self.current_frame = None
        self.current_gray = None
        
        # Initialization timer
        self.init_start_time = None
        self.capture_start_time = None
        
        # Visualization
        self.viz_running = True
        self.viz_thread = Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Live Camera Driver with Spacebar Control")
        self.get_logger().info(f"Configuration: {self.exp_config_msg}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Waiting for handshake with C++ node...")
    
    def ack_callback(self, msg):
        """Handle acknowledgment from C++ node"""
        self.get_logger().info(f"Received: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False
            self.camera_ready = True
            self.init_start_time = time.time()
            self.get_logger().info("✓ Handshake complete! Ready to capture frames...")
            self.get_logger().info("=" * 60)
            self.get_logger().info("PRESS SPACEBAR to start/stop sending frames to SLAM")
            self.get_logger().info("Camera will show live feed continuously")
            self.get_logger().info("=" * 60)
    
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
            
            # Check if SLAM initialized
            if not self.slam_initialized and len(self.trajectory) > 5:
                self.slam_initialized = True
                init_time = time.time() - self.capture_start_time if self.capture_start_time else 0
                self.get_logger().info(f"🎉 SLAM INITIALIZED after {init_time:.1f} seconds!")
    
    def handshake_with_cpp_node(self):
        """Send configuration to C++ node"""
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    
    def capture_and_display_frame(self):
        """Capture frame and update display (always running)"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return False
        
        # Store original frame
        self.current_frame = frame.copy()
        
        # Resize if needed
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        
        # Convert to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.current_gray = gray_frame.copy()
        
        return True
    
    def send_frame_to_slam(self):
        """Send the current frame to SLAM system"""
        if self.current_gray is None:
            return False
        
        # Create timestamp
        timestamp = time.time_ns() / 1e9
        
        try:
            # Create ROS messages
            img_msg = self.br.cv2_to_imgmsg(self.current_gray, encoding="mono8")
            timestep_msg = Float64()
            timestep_msg.data = timestamp
            
            # Publish
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg)
            
            self.frame_id += 1
            self.frames_sent += 1
            
            # Calculate FPS
            current_time = time.time()
            frame_time = current_time - self.last_frame_time
            self.frame_times.append(frame_time)
            self.last_frame_time = current_time
            
            if len(self.frame_times) > 0:
                self.actual_fps = 1.0 / (sum(self.frame_times) / len(self.frame_times))
            
            return True
            
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return False
    
    def draw_camera_view(self):
        """Draw camera feed with overlay information"""
        if self.current_frame is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        display = self.current_frame.copy()
        h, w = display.shape[:2]
        
        # Status overlay
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, display, 0.4, 0, display)
        
        # Status indicator
        if self.slam_initialized:
            status_color = (0, 255, 0)
            status_text = "TRACKING"
        elif self.send_frames:
            status_color = (0, 255, 255)
            status_text = "INITIALIZING"
        elif self.camera_ready:
            status_color = (255, 255, 0)
            status_text = "READY"
        else:
            status_color = (0, 0, 255)
            status_text = "WAITING"
        
        cv2.putText(display, status_text, (10, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
        
        # Capture status
        capture_status = "SENDING TO SLAM" if self.send_frames else "PAUSED (Press SPACE)"
        capture_color = (0, 255, 0) if self.send_frames else (100, 100, 100)
        cv2.putText(display, capture_status, (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, capture_color, 2)
        
        # Info
        cv2.putText(display, f"Frames Sent: {self.frames_sent} | FPS: {self.actual_fps:.1f}",
                   (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display, f"Trajectory: {len(self.trajectory)} pts",
                   (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Instructions at bottom
        if not self.slam_initialized:
            overlay = display.copy()
            cv2.rectangle(overlay, (0, h - 80), (w, h), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)
            
            cv2.putText(display, "SPACEBAR: Start/Stop Frame Capture",
                       (w//2 - 180, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display, "Move camera smoothly while capturing",
                       (w//2 - 160, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Grayscale preview
        if self.current_gray is not None:
            gray_small = cv2.resize(self.current_gray, (160, 120))
            gray_bgr = cv2.cvtColor(gray_small, cv2.COLOR_GRAY2BGR)
            display[h-125:h-5, w-165:w-5] = gray_bgr
            cv2.rectangle(display, (w-165, h-125), (w-5, h-5), (255, 255, 255), 2)
            cv2.putText(display, "Grayscale", (w-160, h-130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return display
    
    def draw_trajectory_2d(self, width=800, height=600, scale=100):
        """Draw 2D trajectory map"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        with self.lock:
            if len(self.trajectory) < 2:
                # Draw grid
                for i in range(0, width, 50):
                    cv2.line(img, (i, 0), (i, height), (30, 30, 30), 1)
                for i in range(0, height, 50):
                    cv2.line(img, (0, i), (width, i), (30, 30, 30), 1)
                
                # Waiting message
                if self.slam_initialized:
                    msg = "Starting trajectory..."
                elif self.send_frames:
                    msg = "Waiting for SLAM initialization..."
                else:
                    msg = "Press SPACEBAR to start capturing"
                
                text_size = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                x_pos = (width - text_size[0]) // 2
                cv2.putText(img, msg, (x_pos, height // 2),
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
        
        # Draw trajectory
        for i in range(1, len(trajectory)):
            pt1 = (
                int(center_x + trajectory[i-1][0] * scale),
                int(center_y - trajectory[i-1][2] * scale)
            )
            pt2 = (
                int(center_x + trajectory[i][0] * scale),
                int(center_y - trajectory[i][2] * scale)
            )
            
            # Clip points to image bounds
            pt1 = (max(0, min(width-1, pt1[0])), max(0, min(height-1, pt1[1])))
            pt2 = (max(0, min(width-1, pt2[0])), max(0, min(height-1, pt2[1])))
            
            color_ratio = i / len(trajectory)
            color = (0, int(255 * color_ratio), int(255 * (1 - color_ratio)))
            cv2.line(img, pt1, pt2, color, 2)
        
        # Current position
        if self.current_pose:
            curr_pt = (
                int(center_x + self.current_pose['x'] * scale),
                int(center_y - self.current_pose['z'] * scale)
            )
            curr_pt = (max(0, min(width-1, curr_pt[0])), max(0, min(height-1, curr_pt[1])))
            cv2.circle(img, curr_pt, 8, (0, 255, 0), -1)
            cv2.circle(img, curr_pt, 10, (255, 255, 255), 2)
        
        # Info panel
        cv2.rectangle(img, (5, 5), (280, 140), (0, 0, 0), -1)
        cv2.rectangle(img, (5, 5), (280, 140), (100, 100, 100), 2)
        
        info = [
            f"Points: {len(self.trajectory)}",
            f"Frames Sent: {self.frames_sent}",
            f"FPS: {self.actual_fps:.1f}",
            f"Status: {'TRACKING' if self.slam_initialized else 'INIT'}",
        ]
        
        y_offset = 30
        for text in info:
            cv2.putText(img, text, (15, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 30
        
        return img
    
    def visualization_loop(self):
        """Real-time visualization thread"""
        if self.show_camera:
            cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Camera Feed', 640, 480)
        
        cv2.namedWindow('ORB-SLAM3: Trajectory', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ORB-SLAM3: Trajectory', 800, 600)
        
        scale = 100
        
        while self.viz_running and rclpy.ok():
            if self.show_camera:
                camera_img = self.draw_camera_view()
                cv2.imshow('Camera Feed', camera_img)
            
            trajectory_img = self.draw_trajectory_2d(scale=scale)
            cv2.imshow('ORB-SLAM3: Trajectory', trajectory_img)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                self.get_logger().info("Quit requested by user")
                self.viz_running = False
                break
            elif key == ord(' '):  # Spacebar
                self.send_frames = not self.send_frames
                if self.send_frames:
                    self.capture_start_time = time.time()
                    self.get_logger().info("▶ Started sending frames to SLAM")
                else:
                    self.get_logger().info("⏸ Paused sending frames")
            elif key == ord('r'):
                with self.lock:
                    self.trajectory = []
                    self.current_pose = None
                    self.slam_initialized = False
                    self.frames_sent = 0
                self.get_logger().info("Trajectory reset")
            elif key == ord('+') or key == ord('='):
                scale = min(scale + 10, 500)
                self.get_logger().info(f"Scale: {scale}")
            elif key == ord('-') or key == ord('_'):
                scale = max(scale - 10, 10)
                self.get_logger().info(f"Scale: {scale}")
        
        cv2.destroyAllWindows()
    
    def cleanup(self):
        """Cleanup resources"""
        self.get_logger().info("Cleaning up...")
        self.viz_running = False
        if self.viz_thread.is_alive():
            self.viz_thread.join(timeout=2)
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Cleanup complete")
    
    def destroy_node(self):
        """Override destroy to cleanup"""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LiveMonoCameraSpacebar("live_mono_camera")
    except Exception as e:
        print(f"Failed to initialize node: {e}")
        rclpy.shutdown()
        return
    
    rate = node.create_rate(node.fps)
    
    # Handshake
    node.get_logger().info("Starting handshake...")
    handshake_timeout = 60
    handshake_start = time.time()
    
    while node.send_config and rclpy.ok():
        if time.time() - handshake_start > handshake_timeout:
            node.get_logger().error("Handshake timeout!")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        node.handshake_with_cpp_node()
        rclpy.spin_once(node, timeout_sec=0.01)
        
        if not node.send_config:
            break
    
    if not node.camera_ready:
        node.get_logger().error("Handshake failed!")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Main loop - continuously capture and optionally send frames
    node.get_logger().info("=" * 60)
    node.get_logger().info("✓ System ready!")
    node.get_logger().info("Camera will show LIVE feed continuously")
    node.get_logger().info("Press SPACEBAR to start/stop sending frames to SLAM")
    node.get_logger().info("Press 'Q' to quit, 'R' to reset trajectory")
    node.get_logger().info("=" * 60)
    
    try:
        while rclpy.ok() and node.viz_running:
            # Always capture frames for display
            if not node.capture_and_display_frame():
                time.sleep(0.1)
                continue
            
            # Only send to SLAM when spacebar is pressed
            if node.send_frames and node.camera_ready:
                node.send_frame_to_slam()
            
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.001)
            rate.sleep()
            
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl+C)")
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

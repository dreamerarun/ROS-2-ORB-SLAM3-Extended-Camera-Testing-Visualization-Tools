#!/usr/bin/env python3
"""
Live Camera Feed with Initialization Helper
Adds visual guidance for proper SLAM initialization motion

Author: Modified for better initialization support
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


class LiveMonoCameraWithHelper(Node):
    def __init__(self, node_name="live_mono_camera"):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter("settings_name", "Webcam_Sensitive")
        self.declare_parameter("camera_id", 4)
        self.declare_parameter("fps", 20)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("show_camera", True)
        self.declare_parameter("init_helper", True)  # Show initialization guidance
        
        # Get parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.camera_id = int(self.get_parameter('camera_id').value)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.show_camera = bool(self.get_parameter('show_camera').value)
        self.init_helper = bool(self.get_parameter('init_helper').value)
        
        # State variables
        self.send_config = True
        self.camera_ready = False
        self.slam_initialized = False
        self.current_pose = None
        self.trajectory = []
        self.lock = Lock()
        
        # Feature tracking for initialization helper
        self.prev_gray = None
        self.feature_detector = cv2.ORB_create(nfeatures=500)
        self.optical_flow_motion = 0.0
        self.motion_history = deque(maxlen=30)
        
        # Performance monitoring
        self.frame_times = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.actual_fps = 0.0
        
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
        
        # Visualization
        self.viz_running = True
        self.viz_thread = Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Live Camera Driver with Init Helper initialized")
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
            self.get_logger().info("✓ Handshake complete! Starting camera feed...")
    
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
                init_time = time.time() - self.init_start_time if self.init_start_time else 0
                self.get_logger().info(f"🎉 SLAM INITIALIZED after {init_time:.1f} seconds!")
    
    def handshake_with_cpp_node(self):
        """Send configuration to C++ node"""
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    
    def calculate_motion(self, gray_frame):
        """Calculate optical flow motion for initialization guidance"""
        if self.prev_gray is None:
            self.prev_gray = gray_frame.copy()
            return 0.0
        
        try:
            # Detect features
            kp = self.feature_detector.detect(self.prev_gray, None)
            if len(kp) < 10:
                self.prev_gray = gray_frame.copy()
                return 0.0
            
            # Convert to points
            p0 = np.array([k.pt for k in kp[:100]], dtype=np.float32).reshape(-1, 1, 2)
            
            # Calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray_frame, p0, None,
                winSize=(21, 21), maxLevel=3
            )
            
            if p1 is None or st is None:
                self.prev_gray = gray_frame.copy()
                return 0.0
            
            # Select good points
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            
            if len(good_new) < 5:
                self.prev_gray = gray_frame.copy()
                return 0.0
            
            # Calculate average motion
            motion = np.mean(np.sqrt(np.sum((good_new - good_old)**2, axis=1)))
            
            self.prev_gray = gray_frame.copy()
            return float(motion)
            
        except Exception as e:
            self.prev_gray = gray_frame.copy()
            return 0.0
    
    def send_frame(self):
        """Capture and send a single frame"""
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
        
        # Calculate motion for initialization helper
        if self.init_helper and not self.slam_initialized:
            motion = self.calculate_motion(gray_frame)
            self.motion_history.append(motion)
            self.optical_flow_motion = motion
        
        # Create timestamp
        timestamp = time.time_ns() / 1e9
        
        try:
            # Create ROS messages
            img_msg = self.br.cv2_to_imgmsg(gray_frame, encoding="mono8")
            timestep_msg = Float64()
            timestep_msg.data = timestamp
            
            # Publish
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg)
            
            self.frame_id += 1
            
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
    
    def draw_initialization_helper(self, display):
        """Draw initialization guidance overlay"""
        h, w = display.shape[:2]
        
        # Create overlay
        overlay = display.copy()
        
        if not self.slam_initialized:
            # Time elapsed
            elapsed = time.time() - self.init_start_time if self.init_start_time else 0
            
            # Motion indicator
            motion_avg = np.mean(self.motion_history) if len(self.motion_history) > 0 else 0
            
            # Determine status
            if motion_avg < 2:
                status = "MOVE CAMERA SIDEWAYS!"
                color = (0, 0, 255)  # Red
                instruction = "Too little motion - move left/right 20cm"
            elif motion_avg > 15:
                status = "TOO FAST - SLOW DOWN!"
                color = (0, 165, 255)  # Orange
                instruction = "Move more slowly and smoothly"
            else:
                status = "GOOD MOTION - Keep going!"
                color = (0, 255, 0)  # Green
                instruction = "Continue smooth left-right motion"
            
            # Draw motion bar
            bar_width = int((min(motion_avg, 20) / 20.0) * (w - 40))
            cv2.rectangle(overlay, (20, h - 60), (w - 20, h - 40), (50, 50, 50), -1)
            cv2.rectangle(overlay, (20, h - 60), (20 + bar_width, h - 40), color, -1)
            
            # Text
            cv2.rectangle(overlay, (0, h - 150), (w, h - 70), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)
            
            cv2.putText(display, status, (w//2 - 200, h - 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display, instruction, (w//2 - 250, h - 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display, f"Elapsed: {elapsed:.1f}s | Motion: {motion_avg:.1f}",
                       (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Draw motion direction arrows
            center_x, center_y = w // 2, h // 2
            arrow_phase = (time.time() * 2) % 2  # Oscillate
            if arrow_phase < 1:
                # Left arrow
                cv2.arrowedLine(display, (center_x, center_y), (center_x - 100, center_y),
                              (0, 255, 255), 3, tipLength=0.3)
            else:
                # Right arrow
                cv2.arrowedLine(display, (center_x, center_y), (center_x + 100, center_y),
                              (0, 255, 255), 3, tipLength=0.3)
        
        return display
    
    def draw_camera_view(self):
        """Draw camera feed with overlay information"""
        if self.current_frame is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        display = self.current_frame.copy()
        h, w = display.shape[:2]
        
        # Add initialization helper if not initialized
        if self.init_helper and not self.slam_initialized:
            display = self.draw_initialization_helper(display)
        
        # Status overlay
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, display, 0.5, 0, display)
        
        # Status indicator
        if self.slam_initialized:
            status_color = (0, 255, 0)
            status_text = "TRACKING"
        elif self.camera_ready:
            status_color = (0, 255, 255)
            status_text = "INITIALIZING"
        else:
            status_color = (0, 0, 255)
            status_text = "WAITING"
        
        cv2.putText(display, status_text, (10, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
        
        # Info
        cv2.putText(display, f"Frame: {self.frame_id} | FPS: {self.actual_fps:.1f}",
                   (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display, f"Trajectory: {len(self.trajectory)} pts",
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Grayscale preview
        if self.current_gray is not None:
            gray_small = cv2.resize(self.current_gray, (160, 120))
            gray_bgr = cv2.cvtColor(gray_small, cv2.COLOR_GRAY2BGR)
            display[h-125:h-5, w-165:w-5] = gray_bgr
            cv2.rectangle(display, (w-165, h-125), (w-5, h-5), (255, 255, 255), 2)
        
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
                else:
                    msg = "Waiting for SLAM initialization..."
                
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
            
            color_ratio = i / len(trajectory)
            color = (0, int(255 * color_ratio), int(255 * (1 - color_ratio)))
            cv2.line(img, pt1, pt2, color, 2)
        
        # Current position
        if self.current_pose:
            curr_pt = (
                int(center_x + self.current_pose['x'] * scale),
                int(center_y - self.current_pose['z'] * scale)
            )
            cv2.circle(img, curr_pt, 8, (0, 255, 0), -1)
        
        # Info
        cv2.rectangle(img, (5, 5), (250, 120), (0, 0, 0), -1)
        cv2.rectangle(img, (5, 5), (250, 120), (100, 100, 100), 1)
        
        info = [
            f"Points: {len(self.trajectory)}",
            f"Frame: {self.frame_id}",
            f"FPS: {self.actual_fps:.1f}",
        ]
        
        y_offset = 25
        for text in info:
            cv2.putText(img, text, (10, y_offset),
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
            elif key == ord('r'):
                with self.lock:
                    self.trajectory = []
                    self.current_pose = None
                    self.slam_initialized = False
                self.get_logger().info("Trajectory reset")
        
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
        node = LiveMonoCameraWithHelper("live_mono_camera")
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
    
    # Main loop
    node.get_logger().info("=" * 60)
    node.get_logger().info("✓ System ready!")
    node.get_logger().info("FOLLOW THE ON-SCREEN MOTION GUIDANCE")
    node.get_logger().info("Move camera LEFT-RIGHT slowly and smoothly")
    node.get_logger().info("=" * 60)
    
    try:
        while rclpy.ok() and node.viz_running:
            if not node.send_frame():
                time.sleep(0.1)
                continue
            
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

#!/usr/bin/env python3
"""
Auto-Motion Camera Driver for ORB-SLAM3
Automatically generates synthetic camera motion from static frames
This simulates camera movement without physically moving the camera

Usage:
    python3 auto_motion_camera.py --camera_id 0 --motion_pattern circular
"""

import sys
import time
import numpy as np
import cv2
import argparse
import math
from pathlib import Path
from threading import Thread, Lock
from collections import deque

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class AutoMotionCamera(Node):
    def __init__(self, node_name="auto_motion_camera", motion_pattern="horizontal"):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter("settings_name", "Webcam_Sensitive")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("fps", 20)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("motion_amplitude", 40)  # pixels
        self.declare_parameter("motion_speed", 1.0)  # cycles per second
        
        # Get parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.camera_id = int(self.get_parameter('camera_id').value)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.motion_amplitude = int(self.get_parameter('motion_amplitude').value)
        self.motion_speed = float(self.get_parameter('motion_speed').value)
        
        # Motion pattern
        self.motion_pattern = motion_pattern
        
        # State variables
        self.send_config = True
        self.camera_ready = False
        self.slam_initialized = False
        self.current_pose = None
        self.trajectory = []
        self.lock = Lock()
        
        # Motion generation
        self.motion_time = 0.0
        self.base_frame = None
        self.motion_phase = 0
        
        # Performance monitoring
        self.frame_times = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.actual_fps = 0.0
        
        # CV Bridge
        self.br = CvBridge()
        
        # Initialize camera
        self.get_logger().info(f"Opening camera {self.camera_id}...")
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera {self.camera_id}")
            sys.exit(1)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        self.get_logger().info(f"Camera initialized: {self.width}x{self.height}")
        
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
        self.current_display_frame = None
        self.current_gray = None
        
        # Visualization
        self.viz_running = True
        self.viz_thread = Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Auto-Motion Camera Driver initialized")
        self.get_logger().info(f"Motion pattern: {self.motion_pattern}")
        self.get_logger().info(f"Motion amplitude: {self.motion_amplitude}px")
        self.get_logger().info(f"Motion speed: {self.motion_speed} cycles/sec")
        self.get_logger().info("=" * 60)
    
    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.send_config = False
            self.camera_ready = True
            self.get_logger().info("✓ Handshake complete! Starting AUTO-MOTION...")
    
    def pose_callback(self, msg):
        with self.lock:
            position = msg.pose.position
            self.current_pose = {
                'x': position.x,
                'y': position.y,
                'z': position.z,
            }
            self.trajectory.append([position.x, position.y, position.z])
            if len(self.trajectory) > 1000:
                self.trajectory.pop(0)
            
            if not self.slam_initialized and len(self.trajectory) > 5:
                self.slam_initialized = True
                self.get_logger().info("🎉 SLAM INITIALIZED with auto-motion!")
    
    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    
    def generate_motion_transform(self, t):
        """
        Generate transformation matrix for synthetic camera motion
        t: time parameter (increases each frame)
        """
        if self.motion_pattern == "horizontal":
            # Simple horizontal oscillation
            dx = self.motion_amplitude * math.sin(2 * math.pi * self.motion_speed * t)
            dy = 0
            
        elif self.motion_pattern == "vertical":
            # Vertical oscillation
            dx = 0
            dy = self.motion_amplitude * math.sin(2 * math.pi * self.motion_speed * t)
            
        elif self.motion_pattern == "circular":
            # Circular motion
            angle = 2 * math.pi * self.motion_speed * t
            dx = self.motion_amplitude * math.cos(angle)
            dy = self.motion_amplitude * math.sin(angle)
            
        elif self.motion_pattern == "figure8":
            # Figure-8 pattern
            angle = 2 * math.pi * self.motion_speed * t
            dx = self.motion_amplitude * math.sin(angle)
            dy = self.motion_amplitude * math.sin(2 * angle) / 2
            
        elif self.motion_pattern == "random":
            # Random walk (for variety)
            self.motion_phase += 0.1
            dx = self.motion_amplitude * math.sin(self.motion_phase * 0.7)
            dy = self.motion_amplitude * math.cos(self.motion_phase * 0.5)
            
        else:  # default to horizontal
            dx = self.motion_amplitude * math.sin(2 * math.pi * self.motion_speed * t)
            dy = 0
        
        # Create affine transformation matrix
        M = np.float32([[1, 0, dx], [0, 1, dy]])
        
        return M, dx, dy
    
    def apply_synthetic_motion(self, frame):
        """
        Apply synthetic camera motion to a frame
        """
        h, w = frame.shape[:2]
        
        # Get transformation matrix
        M, dx, dy = self.generate_motion_transform(self.motion_time)
        
        # Apply transformation (simulates camera motion)
        # Add padding to avoid black borders
        pad = self.motion_amplitude + 10
        frame_padded = cv2.copyMakeBorder(
            frame, pad, pad, pad, pad, 
            cv2.BORDER_REPLICATE
        )
        
        # Apply shift
        shifted = cv2.warpAffine(
            frame_padded, M, 
            (frame_padded.shape[1], frame_padded.shape[0]),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE
        )
        
        # Crop back to original size (centered)
        result = shifted[pad:pad+h, pad:pad+w]
        
        return result, dx, dy
    
    def capture_base_frame(self):
        """Capture a new base frame from camera"""
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        
        return frame
    
    def send_frame(self):
        """Generate and send a frame with synthetic motion"""
        # Capture new base frame every N frames to keep content fresh
        if self.base_frame is None or self.frame_id % 100 == 0:
            self.base_frame = self.capture_base_frame()
            if self.base_frame is None:
                self.get_logger().warn("Failed to capture base frame")
                return False
        
        # Apply synthetic motion to base frame
        frame_with_motion, dx, dy = self.apply_synthetic_motion(self.base_frame)
        
        # Store for visualization
        self.current_display_frame = frame_with_motion.copy()
        
        # Convert to grayscale
        gray_frame = cv2.cvtColor(frame_with_motion, cv2.COLOR_BGR2GRAY)
        self.current_gray = gray_frame.copy()
        
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
            self.motion_time += 1.0 / self.fps  # Increment motion time
            
            # Calculate FPS
            current_time = time.time()
            frame_time = current_time - self.last_frame_time
            self.frame_times.append(frame_time)
            self.last_frame_time = current_time
            
            if len(self.frame_times) > 0:
                self.actual_fps = 1.0 / (sum(self.frame_times) / len(self.frame_times))
            
            # Log progress
            if self.frame_id % 100 == 0:
                self.get_logger().info(
                    f"Frames: {self.frame_id} | FPS: {self.actual_fps:.1f} | "
                    f"Motion: ({dx:.1f}, {dy:.1f}) | Trajectory: {len(self.trajectory)}"
                )
            
            return True
            
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return False
    
    def draw_camera_view(self):
        """Draw camera feed with motion visualization"""
        if self.current_display_frame is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        display = self.current_display_frame.copy()
        h, w = display.shape[:2]
        
        # Draw motion indicator
        M, dx, dy = self.generate_motion_transform(self.motion_time)
        
        # Status overlay
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, display, 0.4, 0, display)
        
        # Status
        if self.slam_initialized:
            status_color = (0, 255, 0)
            status_text = "TRACKING"
        elif self.camera_ready:
            status_color = (0, 255, 255)
            status_text = "INITIALIZING (AUTO-MOTION)"
        else:
            status_color = (255, 0, 0)
            status_text = "WAITING"
        
        cv2.putText(display, status_text, (10, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Info
        info_lines = [
            f"Frame: {self.frame_id} | FPS: {self.actual_fps:.1f}",
            f"Motion: {self.motion_pattern} ({dx:.1f}, {dy:.1f})px",
            f"Trajectory: {len(self.trajectory)} points"
        ]
        
        y_offset = 65
        for line in info_lines:
            cv2.putText(display, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25
        
        # Draw motion path indicator
        center_x, center_y = w // 2, 80
        path_points = []
        for i in range(20):
            t = self.motion_time + i * 0.05
            _, px, py = self.generate_motion_transform(t)
            path_points.append((int(center_x + px), int(center_y + py)))
        
        # Draw path
        for i in range(len(path_points) - 1):
            cv2.line(display, path_points[i], path_points[i+1], (0, 255, 255), 2)
        
        # Current position
        cv2.circle(display, path_points[0], 6, (0, 255, 0), -1)
        
        # Grayscale preview
        if self.current_gray is not None:
            gray_small = cv2.resize(self.current_gray, (160, 120))
            gray_bgr = cv2.cvtColor(gray_small, cv2.COLOR_GRAY2BGR)
            display[h-125:h-5, w-165:w-5] = gray_bgr
            cv2.rectangle(display, (w-165, h-125), (w-5, h-5), (255, 255, 255), 2)
            cv2.putText(display, "SENT", (w-150, h-15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return display
    
    def draw_trajectory_2d(self, width=800, height=600, scale=100):
        """Draw 2D trajectory map"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        with self.lock:
            if len(self.trajectory) < 2:
                # Grid
                for i in range(0, width, 50):
                    cv2.line(img, (i, 0), (i, height), (30, 30, 30), 1)
                for i in range(0, height, 50):
                    cv2.line(img, (0, i), (width, i), (30, 30, 30), 1)
                
                msg = "Waiting for initialization with AUTO-MOTION..."
                text_size = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                x_pos = (width - text_size[0]) // 2
                cv2.putText(img, msg, (x_pos, height // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                return img
            
            trajectory = np.array(self.trajectory)
        
        center_x, center_y = width // 2, height // 2
        
        # Grid
        for i in range(0, width, 50):
            cv2.line(img, (i, 0), (i, height), (30, 30, 30), 1)
        for i in range(0, height, 50):
            cv2.line(img, (0, i), (width, i), (30, 30, 30), 1)
        
        # Trajectory
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
        
        # Info overlay
        cv2.rectangle(img, (5, 5), (300, 100), (0, 0, 0), -1)
        info = [
            f"Trajectory: {len(self.trajectory)} points",
            f"Frame: {self.frame_id}",
            f"Pattern: {self.motion_pattern}",
        ]
        y = 25
        for text in info:
            cv2.putText(img, text, (10, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y += 25
        
        return img
    
    def visualization_loop(self):
        """Visualization thread"""
        cv2.namedWindow('Auto-Motion Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Auto-Motion Camera', 640, 480)
        cv2.namedWindow('Trajectory', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Trajectory', 800, 600)
        
        while self.viz_running and rclpy.ok():
            camera_img = self.draw_camera_view()
            cv2.imshow('Auto-Motion Camera', camera_img)
            
            traj_img = self.draw_trajectory_2d()
            cv2.imshow('Trajectory', traj_img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.viz_running = False
                break
            elif key == ord('r'):
                with self.lock:
                    self.trajectory = []
                    self.slam_initialized = False
                self.get_logger().info("Reset trajectory")
        
        cv2.destroyAllWindows()
    
    def cleanup(self):
        self.viz_running = False
        if self.viz_thread.is_alive():
            self.viz_thread.join(timeout=2)
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
    
    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_id', type=int, default=0)
    parser.add_argument('--motion_pattern', type=str, default='horizontal',
                       choices=['horizontal', 'vertical', 'circular', 'figure8', 'random'])
    parser.add_argument('--motion_amplitude', type=int, default=40,
                       help='Motion amplitude in pixels')
    parser.add_argument('--motion_speed', type=float, default=0.5,
                       help='Motion speed in cycles per second')
    
    parsed_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    
    try:
        node = AutoMotionCamera(
            "auto_motion_camera",
            motion_pattern=parsed_args.motion_pattern
        )
        
        # Override parameters from command line
        node.motion_amplitude = parsed_args.motion_amplitude
        node.motion_speed = parsed_args.motion_speed
        
    except Exception as e:
        print(f"Failed to initialize: {e}")
        rclpy.shutdown()
        return
    
    rate = node.create_rate(node.fps)
    
    # Handshake
    node.get_logger().info("Starting handshake...")
    timeout = 60
    start = time.time()
    
    while node.send_config and rclpy.ok():
        if time.time() - start > timeout:
            node.get_logger().error("Handshake timeout!")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        node.handshake_with_cpp_node()
        rclpy.spin_once(node, timeout_sec=0.01)
        if not node.send_config:
            break
    
    if not node.camera_ready:
        node.destroy_node()
        rclpy.shutdown()
        return
    
    node.get_logger().info("=" * 60)
    node.get_logger().info("✓ AUTO-MOTION STARTED!")
    node.get_logger().info("Camera will move automatically - NO MANUAL MOVEMENT NEEDED")
    node.get_logger().info("=" * 60)
    
    try:
        while rclpy.ok() and node.viz_running:
            if not node.send_frame():
                time.sleep(0.1)
                continue
            rclpy.spin_once(node, timeout_sec=0.001)
            rate.sleep()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

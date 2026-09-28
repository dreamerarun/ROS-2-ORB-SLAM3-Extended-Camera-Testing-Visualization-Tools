#!/usr/bin/env python3
"""
Fixed Live Camera Feed Driver for ORB-SLAM3 with Real-time Visualization
Compatible with ros2_orb_slam3 mono_node_cpp

Key Fixes:
- Added camera frame preview
- Proper resolution handling
- Better error handling and debugging
- FPS monitoring
- Camera calibration warnings

Author: Based on mono_driver_node.py by Azmyin Md. Kamal
Modified for live camera feed and real-time visualization
"""

import sys
import time
import numpy as np
import cv2
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


class LiveMonoCamera(Node):
    def __init__(self, node_name="live_mono_camera"):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("fps", 20)  # Changed to 20 to match EuRoC config
        self.declare_parameter("width", 752)  # Match EuRoC config
        self.declare_parameter("height", 480)  # Match EuRoC config
        self.declare_parameter("save_frames", False)  # Option to save frames
        self.declare_parameter("show_camera", True)  # Show camera feed
        
        # Get parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.camera_id = int(self.get_parameter('camera_id').value)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.save_frames = bool(self.get_parameter('save_frames').value)
        self.show_camera = bool(self.get_parameter('show_camera').value)
        
        # State variables
        self.send_config = True
        self.camera_ready = False
        self.current_pose = None
        self.trajectory = []
        self.lock = Lock()
        
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
            self.get_logger().error("Please check:")
            self.get_logger().error("  1. Camera is connected")
            self.get_logger().error("  2. No other application is using the camera")
            self.get_logger().error("  3. Try different camera_id (0, 1, 2...)")
            sys.exit(1)
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Get actual camera properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps} FPS")
        
        # Warning if resolution mismatch
        if actual_width != self.width or actual_height != self.height:
            self.get_logger().warn(f"Camera resolution mismatch!")
            self.get_logger().warn(f"  Requested: {self.width}x{self.height}")
            self.get_logger().warn(f"  Actual:    {actual_width}x{actual_height}")
            self.get_logger().warn("Images will be resized, which may affect SLAM performance")
        
        # Create output directory for saved frames if needed
        if self.save_frames:
            from datetime import datetime
            self.output_dir = Path(f"./camera_frames_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Saving frames to: {self.output_dir}")
        
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
        
        # Current frame storage for visualization
        self.current_frame = None
        self.current_gray = None
        
        # Visualization
        self.viz_running = True
        self.viz_thread = Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Live Camera Driver initialized")
        self.get_logger().info(f"Configuration: {self.exp_config_msg}")
        self.get_logger().warn("IMPORTANT: Make sure your camera is calibrated!")
        self.get_logger().warn(f"Currently using: {self.settings_name}.yaml")
        self.get_logger().warn("This may not match your camera's intrinsics")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Waiting for handshake with C++ node...")
    
    def ack_callback(self, msg):
        """Handle acknowledgment from C++ node"""
        self.get_logger().info(f"Received: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False
            self.camera_ready = True
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
        
        # Store original frame for visualization
        self.current_frame = frame.copy()
        
        # Resize if needed
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        
        # Convert to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.current_gray = gray_frame.copy()
        
        # Save frame if enabled
        if self.save_frames:
            frame_path = self.output_dir / f"frame_{self.frame_id:06d}.png"
            cv2.imwrite(str(frame_path), gray_frame)
        
        # Create timestamp (nanoseconds since epoch converted to seconds)
        timestamp = time.time_ns() / 1e9
        
        try:
            # Create ROS messages
            img_msg = self.br.cv2_to_imgmsg(gray_frame, encoding="mono8")
            timestep_msg = Float64()
            timestep_msg.data = timestamp
            
            # Publish (timestep first, then image - ORDER MATTERS)
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
            
            # Log every 100 frames
            if self.frame_id % 100 == 0:
                self.get_logger().info(
                    f"Frames sent: {self.frame_id} | "
                    f"FPS: {self.actual_fps:.1f} | "
                    f"Trajectory points: {len(self.trajectory)}"
                )
            
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
        
        # Create semi-transparent overlay for text background
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, display, 0.5, 0, display)
        
        # Status indicator
        status_color = (0, 255, 0) if self.camera_ready else (0, 0, 255)
        status_text = "TRACKING" if len(self.trajectory) > 0 else "INITIALIZING"
        cv2.putText(display, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Frame info
        info_lines = [
            f"Frame: {self.frame_id}",
            f"FPS: {self.actual_fps:.1f}",
            f"Trajectory: {len(self.trajectory)} pts",
        ]
        
        y_offset = 60
        for line in info_lines:
            cv2.putText(display, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25
        
        # Draw grayscale preview in corner
        if self.current_gray is not None:
            gray_small = cv2.resize(self.current_gray, (160, 120))
            gray_bgr = cv2.cvtColor(gray_small, cv2.COLOR_GRAY2BGR)
            display[h-125:h-5, w-165:w-5] = gray_bgr
            cv2.rectangle(display, (w-165, h-125), (w-5, h-5), (255, 255, 255), 2)
            cv2.putText(display, "SENT", (w-150, h-10),
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
                msg_lines = [
                    "Waiting for SLAM initialization...",
                    "",
                    "Move camera slowly with good features",
                    "Ensure good lighting conditions"
                ]
                y_pos = height // 2 - 40
                for line in msg_lines:
                    text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    x_pos = (width - text_size[0]) // 2
                    cv2.putText(img, line, (x_pos, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    y_pos += 35
                
                return img
            
            trajectory = np.array(self.trajectory)
        
        center_x = width // 2
        center_y = height // 2
        
        # Draw grid
        for i in range(0, width, 50):
            cv2.line(img, (i, 0), (i, height), (30, 30, 30), 1)
        for i in range(0, height, 50):
            cv2.line(img, (0, i), (width, i), (30, 30, 30), 1)
        
        # Draw center cross
        cv2.line(img, (center_x - 10, center_y), (center_x + 10, center_y), (100, 100, 100), 1)
        cv2.line(img, (center_x, center_y - 10), (center_x, center_y + 10), (100, 100, 100), 1)
        
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
            
            # Color gradient from blue to green
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
            
            # Draw direction arrow (simplified)
            cv2.arrowedLine(img, curr_pt, 
                          (curr_pt[0], curr_pt[1] - 20),
                          (0, 255, 0), 2, tipLength=0.3)
        
        # Info overlay with dark background
        info = [
            f"Points: {len(self.trajectory)}",
            f"Scale: {scale}x",
            f"Frame: {self.frame_id}",
            f"FPS: {self.actual_fps:.1f}",
        ]
        
        if self.current_pose:
            info.extend([
                "",
                f"X: {self.current_pose['x']:7.3f} m",
                f"Y: {self.current_pose['y']:7.3f} m",
                f"Z: {self.current_pose['z']:7.3f} m"
            ])
        
        # Draw dark background for text
        cv2.rectangle(img, (5, 5), (250, 35 + len(info) * 28), (0, 0, 0), -1)
        cv2.rectangle(img, (5, 5), (250, 35 + len(info) * 28), (100, 100, 100), 1)
        
        y_offset = 25
        for text in info:
            cv2.putText(img, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 28
        
        # Instructions at bottom
        instructions = [
            "Q: Quit | R: Reset | S: Save | +/-: Zoom"
        ]
        y_pos = height - 15
        for inst in instructions:
            cv2.putText(img, inst, (10, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_pos += 20
        
        return img
    
    def visualization_loop(self):
        """Real-time visualization thread"""
        if self.show_camera:
            cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Camera Feed', 640, 480)
        
        cv2.namedWindow('ORB-SLAM3: Trajectory', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ORB-SLAM3: Trajectory', 800, 600)
        
        scale = 100  # Trajectory scale
        
        while self.viz_running and rclpy.ok():
            # Draw camera view
            if self.show_camera:
                camera_img = self.draw_camera_view()
                cv2.imshow('Camera Feed', camera_img)
            
            # Draw trajectory
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
                self.get_logger().info("Trajectory reset")
            elif key == ord('s'):
                self.save_trajectory()
            elif key == ord('+') or key == ord('='):
                scale = min(scale + 10, 500)
                self.get_logger().info(f"Scale: {scale}x")
            elif key == ord('-') or key == ord('_'):
                scale = max(scale - 10, 10)
                self.get_logger().info(f"Scale: {scale}x")
        
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
        self.get_logger().info(f"✓ Trajectory saved to {filename}")
        self.get_logger().info(f"  {len(trajectory)} points saved")
    
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
        node = LiveMonoCamera("live_mono_camera")
    except Exception as e:
        print(f"Failed to initialize node: {e}")
        rclpy.shutdown()
        return
    
    # Create rate controller
    rate = node.create_rate(node.fps)
    
    # Handshake loop
    node.get_logger().info("=" * 60)
    node.get_logger().info("Starting handshake with C++ node...")
    node.get_logger().info("Make sure mono_node_cpp is running!")
    node.get_logger().info("=" * 60)
    
    handshake_timeout = 60  # seconds
    handshake_start = time.time()
    
    while node.send_config and rclpy.ok():
        if time.time() - handshake_start > handshake_timeout:
            node.get_logger().error("Handshake timeout! C++ node not responding")
            node.get_logger().error("Please check if mono_node_cpp is running")
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
    
    # Main camera loop
    node.get_logger().info("=" * 60)
    node.get_logger().info("✓ System ready! Starting camera feed...")
    node.get_logger().info("Controls:")
    node.get_logger().info("  Q: Quit")
    node.get_logger().info("  R: Reset trajectory")
    node.get_logger().info("  S: Save trajectory")
    node.get_logger().info("  +/-: Zoom trajectory view")
    node.get_logger().info("=" * 60)
    
    try:
        while rclpy.ok() and node.viz_running:
            # Send frame
            if not node.send_frame():
                node.get_logger().error("Failed to send frame, retrying...")
                time.sleep(0.1)
                continue
            
            # Process callbacks
            rclpy.spin_once(node, timeout_sec=0.001)
            
            # Maintain FPS
            rate.sleep()
            
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl+C)")
    except Exception as e:
        node.get_logger().error(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Simple Live Camera Driver for ORB-SLAM3
Just captures and sends frames - no motion simulation needed
ORB-SLAM3 will initialize from natural camera/scene movement

Usage:
    python3 simple_live_camera.py
"""

import sys
import time
import numpy as np
import cv2
from threading import Thread, Lock
from collections import deque

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class SimpleLiveCamera(Node):
    def __init__(self):
        super().__init__("simple_live_camera")
        
        # Parameters
        self.declare_parameter("settings_name", "Webcam_Sensitive")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("fps", 20)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.camera_id = int(self.get_parameter('camera_id').value)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        
        # State
        self.send_config = True
        self.camera_ready = False
        self.slam_tracking = False
        self.current_pose = None
        self.trajectory = []
        self.lock = Lock()
        
        # Monitoring
        self.frame_times = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.actual_fps = 0.0
        self.frame_id = 0
        
        # CV Bridge
        self.br = CvBridge()
        
        # Open camera
        self.get_logger().info(f"Opening camera {self.camera_id}...")
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_id}")
            sys.exit(1)
        
        # Set properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize lag
        
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f"✓ Camera ready: {actual_w}x{actual_h}")
        
        # Current frame
        self.current_frame = None
        self.current_gray = None
        
        # ROS2 setup
        self.pub_config = self.create_publisher(String, "/mono_py_driver/experiment_settings", 1)
        self.pub_img = self.create_publisher(Image, "/mono_py_driver/img_msg", 1)
        self.pub_time = self.create_publisher(Float64, "/mono_py_driver/timestep_msg", 1)
        
        self.sub_ack = self.create_subscription(String, "/mono_py_driver/exp_settings_ack", self.ack_callback, 10)
        self.sub_pose = self.create_subscription(PoseStamped, "/orb_slam3/camera_pose", self.pose_callback, 10)
        
        self.exp_config_msg = self.settings_name
        
        # Visualization thread
        self.viz_running = True
        self.viz_thread = Thread(target=self.viz_loop, daemon=True)
        self.viz_thread.start()
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Simple Live Camera initialized")
        self.get_logger().info(f"Config: {self.settings_name}")
        self.get_logger().info("=" * 70)
    
    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.send_config = False
            self.camera_ready = True
            self.get_logger().info("✓ Connected to ORB-SLAM3")
    
    def pose_callback(self, msg):
        with self.lock:
            pos = msg.pose.position
            self.current_pose = {'x': pos.x, 'y': pos.y, 'z': pos.z}
            self.trajectory.append([pos.x, pos.y, pos.z])
            
            if len(self.trajectory) > 1000:
                self.trajectory.pop(0)
            
            if not self.slam_tracking:
                self.slam_tracking = True
                self.get_logger().info("🎉 SLAM is now TRACKING!")
    
    def handshake(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.pub_config.publish(msg)
    
    def send_frame(self):
        """Capture and send one frame"""
        ret, frame = self.cap.read()
        if not ret:
            return False
        
        # Store for display
        self.current_frame = frame.copy()
        
        # Resize if needed
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.current_gray = gray.copy()
        
        # Create messages
        timestamp = time.time_ns() / 1e9
        
        try:
            img_msg = self.br.cv2_to_imgmsg(gray, encoding="mono8")
            time_msg = Float64()
            time_msg.data = timestamp
            
            # Publish
            self.pub_time.publish(time_msg)
            self.pub_img.publish(img_msg)
            
            self.frame_id += 1
            
            # FPS calculation
            now = time.time()
            dt = now - self.last_frame_time
            self.frame_times.append(dt)
            self.last_frame_time = now
            
            if len(self.frame_times) > 0:
                self.actual_fps = 1.0 / (sum(self.frame_times) / len(self.frame_times))
            
            # Log every 100 frames
            if self.frame_id % 100 == 0:
                status = "TRACKING" if self.slam_tracking else "INITIALIZING"
                self.get_logger().info(
                    f"[{status}] Frame: {self.frame_id} | "
                    f"FPS: {self.actual_fps:.1f} | "
                    f"Trajectory: {len(self.trajectory)} points"
                )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            return False
    
    def draw_display(self):
        """Create display with info overlay"""
        if self.current_frame is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        disp = self.current_frame.copy()
        h, w = disp.shape[:2]
        
        # Dark overlay for text
        overlay = disp.copy()
        cv2.rectangle(overlay, (0, 0), (w, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, disp, 0.4, 0, disp)
        
        # Status
        if self.slam_tracking:
            status = "TRACKING"
            color = (0, 255, 0)
        elif self.camera_ready:
            status = "INITIALIZING"
            color = (0, 255, 255)
        else:
            status = "WAITING"
            color = (0, 0, 255)
        
        cv2.putText(disp, status, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        # Info
        cv2.putText(disp, f"Frame: {self.frame_id} | FPS: {self.actual_fps:.1f}", 
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(disp, f"Trajectory: {len(self.trajectory)} points",
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Grayscale preview
        if self.current_gray is not None:
            gray_preview = cv2.resize(self.current_gray, (160, 120))
            gray_bgr = cv2.cvtColor(gray_preview, cv2.COLOR_GRAY2BGR)
            disp[h-125:h-5, w-165:w-5] = gray_bgr
            cv2.rectangle(disp, (w-165, h-125), (w-5, h-5), (255, 255, 255), 2)
            cv2.putText(disp, "SENT", (w-145, h-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return disp
    
    def draw_trajectory(self, width=800, height=600, scale=100):
        """Draw trajectory map"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Grid
        for i in range(0, width, 50):
            cv2.line(img, (i, 0), (i, height), (30, 30, 30), 1)
        for i in range(0, height, 50):
            cv2.line(img, (0, i), (width, i), (30, 30, 30), 1)
        
        with self.lock:
            if len(self.trajectory) < 2:
                msg = "Waiting for SLAM initialization..."
                size = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                x = (width - size[0]) // 2
                cv2.putText(img, msg, (x, height // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                return img
            
            traj = np.array(self.trajectory)
        
        cx, cy = width // 2, height // 2
        
        # Draw trajectory
        for i in range(1, len(traj)):
            pt1 = (int(cx + traj[i-1][0] * scale), int(cy - traj[i-1][2] * scale))
            pt2 = (int(cx + traj[i][0] * scale), int(cy - traj[i][2] * scale))
            
            ratio = i / len(traj)
            color = (0, int(255 * ratio), int(255 * (1 - ratio)))
            cv2.line(img, pt1, pt2, color, 2)
        
        # Current position
        if self.current_pose:
            pt = (int(cx + self.current_pose['x'] * scale),
                  int(cy - self.current_pose['z'] * scale))
            cv2.circle(img, pt, 8, (0, 255, 0), -1)
        
        # Info box
        cv2.rectangle(img, (5, 5), (250, 90), (0, 0, 0), -1)
        cv2.rectangle(img, (5, 5), (250, 90), (100, 100, 100), 1)
        
        info = [
            f"Points: {len(self.trajectory)}",
            f"Frame: {self.frame_id}",
            f"FPS: {self.actual_fps:.1f}"
        ]
        
        y = 25
        for text in info:
            cv2.putText(img, text, (10, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y += 25
        
        return img
    
    def viz_loop(self):
        """Visualization thread"""
        cv2.namedWindow('Live Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Live Camera', 640, 480)
        
        cv2.namedWindow('Trajectory', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Trajectory', 800, 600)
        
        scale = 100
        
        while self.viz_running and rclpy.ok():
            # Camera view
            cam_img = self.draw_display()
            cv2.imshow('Live Camera', cam_img)
            
            # Trajectory
            traj_img = self.draw_trajectory(scale=scale)
            cv2.imshow('Trajectory', traj_img)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Quit requested")
                self.viz_running = False
                break
            elif key == ord('r'):
                with self.lock:
                    self.trajectory = []
                    self.slam_tracking = False
                self.get_logger().info("Trajectory reset")
            elif key == ord('+') or key == ord('='):
                scale = min(scale + 10, 500)
            elif key == ord('-'):
                scale = max(scale - 10, 10)
        
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
    rclpy.init(args=args)
    
    try:
        node = SimpleLiveCamera()
    except Exception as e:
        print(f"Init failed: {e}")
        rclpy.shutdown()
        return
    
    rate = node.create_rate(node.fps)
    
    # Handshake
    node.get_logger().info("Connecting to ORB-SLAM3...")
    timeout = 60
    start = time.time()
    
    while node.send_config and rclpy.ok():
        if time.time() - start > timeout:
            node.get_logger().error("Connection timeout!")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        node.handshake()
        rclpy.spin_once(node, timeout_sec=0.01)
        
        if not node.send_config:
            break
    
    if not node.camera_ready:
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Main loop
    node.get_logger().info("=" * 70)
    node.get_logger().info("✓ CAMERA STREAMING")
    node.get_logger().info("Move camera slowly to initialize SLAM")
    node.get_logger().info("Press 'Q' in any window to quit")
    node.get_logger().info("=" * 70)
    
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
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

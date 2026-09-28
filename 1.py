#!/usr/bin/env python3
"""
Real-time ORB-SLAM3 Python Interface with Map Viewer
Compatible with ros2_orb_slam3 package
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Thread, Lock
import time

class OrbSlam3Visualizer(Node):
    def __init__(self):
        super().__init__('orbslam3_visualizer')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('pose_topic', '/orb_slam3/camera_pose')
        self.declare_parameter('visualization', True)
        
        camera_topic = self.get_parameter('camera_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        
        # CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        
        # State variables
        self.current_frame = None
        self.current_pose = None
        self.trajectory = []
        self.map_points = []
        self.lock = Lock()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            10
        )
        
        # Visualization thread
        self.running = True
        self.viz_thread = Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info('ORB-SLAM3 Visualizer initialized')
        self.get_logger().info(f'Listening to: {camera_topic}')
        self.get_logger().info(f'Pose topic: {pose_topic}')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.current_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def pose_callback(self, msg):
        """Process camera pose updates"""
        with self.lock:
            position = msg.pose.position
            orientation = msg.pose.orientation
            
            self.current_pose = {
                'x': position.x,
                'y': position.y,
                'z': position.z,
                'qx': orientation.x,
                'qy': orientation.y,
                'qz': orientation.z,
                'qw': orientation.w
            }
            
            # Add to trajectory
            self.trajectory.append([position.x, position.y, position.z])
            
            # Keep only last 1000 poses
            if len(self.trajectory) > 1000:
                self.trajectory.pop(0)
    
    def draw_trajectory_2d(self, width=800, height=600, scale=50):
        """Draw 2D top-down view of trajectory"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        with self.lock:
            if len(self.trajectory) < 2:
                return img
            
            trajectory = np.array(self.trajectory)
        
        # Center and scale trajectory
        center_x = width // 2
        center_y = height // 2
        
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
            
            # Color gradient from red to green
            color_ratio = i / len(trajectory)
            color = (0, int(255 * color_ratio), int(255 * (1 - color_ratio)))
            
            cv2.line(img, pt1, pt2, color, 2)
        
        # Draw current position
        if self.current_pose:
            curr_pt = (
                int(center_x + self.current_pose['x'] * scale),
                int(center_y - self.current_pose['z'] * scale)
            )
            cv2.circle(img, curr_pt, 5, (0, 255, 0), -1)
        
        # Add grid
        for i in range(0, width, 50):
            cv2.line(img, (i, 0), (i, height), (50, 50, 50), 1)
        for i in range(0, height, 50):
            cv2.line(img, (0, i), (width, i), (50, 50, 50), 1)
        
        # Add info text
        info_text = [
            f"Trajectory points: {len(self.trajectory)}",
            f"Scale: {scale}",
        ]
        
        if self.current_pose:
            info_text.extend([
                f"X: {self.current_pose['x']:.2f}m",
                f"Y: {self.current_pose['y']:.2f}m",
                f"Z: {self.current_pose['z']:.2f}m"
            ])
        
        for i, text in enumerate(info_text):
            cv2.putText(img, text, (10, 30 + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return img
    
    def visualization_loop(self):
        """Main visualization loop"""
        cv2.namedWindow('ORB-SLAM3: Current Frame', cv2.WINDOW_NORMAL)
        cv2.namedWindow('ORB-SLAM3: Trajectory Map', cv2.WINDOW_NORMAL)
        
        while self.running and rclpy.ok():
            # Display current frame
            with self.lock:
                if self.current_frame is not None:
                    display_frame = self.current_frame.copy()
                    
                    # Add pose info overlay
                    if self.current_pose:
                        pose_text = f"Pose: ({self.current_pose['x']:.2f}, " \
                                   f"{self.current_pose['y']:.2f}, " \
                                   f"{self.current_pose['z']:.2f})"
                        cv2.putText(display_frame, pose_text, (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    cv2.imshow('ORB-SLAM3: Current Frame', display_frame)
            
            # Display trajectory map
            trajectory_img = self.draw_trajectory_2d()
            cv2.imshow('ORB-SLAM3: Trajectory Map', trajectory_img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit requested')
                self.running = False
                break
            elif key == ord('r'):
                with self.lock:
                    self.trajectory = []
                    self.get_logger().info('Trajectory reset')
            elif key == ord('s'):
                # Save trajectory
                self.save_trajectory()
        
        cv2.destroyAllWindows()
    
    def save_trajectory(self):
        """Save trajectory to file"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{timestamp}.txt"
        
        with self.lock:
            if len(self.trajectory) == 0:
                self.get_logger().warn('No trajectory to save')
                return
            
            trajectory = np.array(self.trajectory)
        
        np.savetxt(filename, trajectory, fmt='%.6f', 
                  header='x y z', comments='')
        self.get_logger().info(f'Trajectory saved to {filename}')
    
    def destroy_node(self):
        """Cleanup"""
        self.running = False
        if self.viz_thread.is_alive():
            self.viz_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    visualizer = OrbSlam3Visualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

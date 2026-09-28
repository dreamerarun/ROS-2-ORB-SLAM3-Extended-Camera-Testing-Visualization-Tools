#!/usr/bin/env python3
"""
ORB-SLAM3 to RViz2 Publisher
Publishes SLAM data in RViz2-compatible formats

Author: SLAM Visualization Bridge
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import numpy as np
from collections import deque
import struct


class SLAMToRViz2Publisher(Node):
    def __init__(self):
        super().__init__('slam_to_rviz2_publisher')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/orb_slam3/camera_path', 10)
        self.map_points_pub = self.create_publisher(PointCloud2, '/orb_slam3/map_points', 10)
        self.keyframe_pub = self.create_publisher(PointCloud2, '/orb_slam3/keyframe_points', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/orb_slam3/camera_pose',
            self.pose_callback,
            10
        )
        
        # Path storage
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.max_path_length = 1000
        
        # Map points storage (simulated - you'll need to get real data from ORB-SLAM3)
        self.map_points = deque(maxlen=5000)
        self.keyframe_positions = deque(maxlen=100)
        
        # Timers
        self.create_timer(0.1, self.publish_path)
        self.create_timer(0.5, self.publish_map_points)
        
        self.get_logger().info('SLAM to RViz2 Publisher initialized')
        self.get_logger().info('Publishing on:')
        self.get_logger().info('  - /orb_slam3/camera_path')
        self.get_logger().info('  - /orb_slam3/map_points')
        self.get_logger().info('  - /orb_slam3/keyframe_points')
        self.get_logger().info('  - TF: map -> camera_link')
    
    def pose_callback(self, msg):
        """Handle incoming pose messages"""
        # Add to path
        self.path.poses.append(msg)
        
        # Trim path if too long
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # Update timestamp
        self.path.header.stamp = msg.header.stamp
        
        # Store keyframe positions (every 10th pose)
        if len(self.path.poses) % 10 == 0:
            self.keyframe_positions.append([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
        
        # Simulate map points around the camera (replace with real ORB features)
        # In a real implementation, you'd get these from ORB-SLAM3's map
        if len(self.map_points) < 1000:  # Keep adding until we have enough
            # Add some random points around the current position
            for _ in range(5):
                point = [
                    msg.pose.position.x + np.random.randn() * 0.5,
                    msg.pose.position.y + np.random.randn() * 0.5,
                    msg.pose.position.z + np.random.randn() * 0.3
                ]
                self.map_points.append(point)
        
        # Publish TF
        self.publish_tf(msg)
    
    def publish_tf(self, pose_msg):
        """Publish TF transform from map to camera"""
        t = TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_link'
        
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        
        t.transform.rotation.x = pose_msg.pose.orientation.x
        t.transform.rotation.y = pose_msg.pose.orientation.y
        t.transform.rotation.z = pose_msg.pose.orientation.z
        t.transform.rotation.w = pose_msg.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_path(self):
        """Publish camera trajectory path"""
        if len(self.path.poses) > 0:
            self.path_pub.publish(self.path)
    
    def publish_map_points(self):
        """Publish map points as PointCloud2"""
        if len(self.map_points) == 0:
            return
        
        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        
        # Convert map points to PointCloud2
        points = list(self.map_points)
        cloud = self.create_point_cloud(header, points)
        self.map_points_pub.publish(cloud)
        
        # Publish keyframe points
        if len(self.keyframe_positions) > 0:
            keyframe_points = list(self.keyframe_positions)
            keyframe_cloud = self.create_point_cloud(header, keyframe_points)
            self.keyframe_pub.publish(keyframe_cloud)
    
    def create_point_cloud(self, header, points):
        """Create a PointCloud2 message from a list of points"""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16  # 4 fields * 4 bytes
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        
        # Pack data
        buffer = []
        for point in points:
            x, y, z = point[0], point[1], point[2]
            
            # Create color based on height (z-coordinate)
            # Blue for low, green for mid, red for high
            z_norm = (z + 2) / 4.0  # Normalize z
            z_norm = max(0, min(1, z_norm))
            
            r = int(255 * z_norm)
            g = int(255 * (1 - abs(z_norm - 0.5) * 2))
            b = int(255 * (1 - z_norm))
            
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            
            buffer.append(struct.pack('fffI', x, y, z, rgb))
        
        cloud.data = b''.join(buffer)
        
        return cloud


def main(args=None):
    rclpy.init(args=args)
    
    node = SLAMToRViz2Publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

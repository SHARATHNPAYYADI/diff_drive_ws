#!/usr/bin/env python3
"""

ROS2 node: Localize robot in a fixed occupancy map using laser scans.
- Subscribes: /map (nav_msgs/OccupancyGrid), /scan (sensor_msgs/LaserScan)
- Publishes:  /scan_pose (geometry_msgs/PoseStamped)
"""

import math
import numpy as np
np.float = float  # Fix for older transforms3d

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler


def laser_to_points(scan, max_range=10.0, min_range=0.02):
    """Convert LaserScan to Nx2 numpy array (scanner frame)."""
    angles = np.arange(scan.angle_min,
                       scan.angle_max,
                       scan.angle_increment)
    ranges = np.array(scan.ranges[: angles.shape[0]])
    valid = np.isfinite(ranges) & (ranges <= max_range) & (ranges >= min_range)
    ranges = ranges[valid]
    angles = angles[valid]
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    return np.stack([xs, ys], axis=-1)


class ScanMapLocalizer(Node):
    def __init__(self):
        super().__init__("scan_map_localizer")

        # Subscribers
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, 1)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, "/scan_pose", 10)

        # Internal state
        self.map = None
        self.map_info = None
        self.pose = np.array([0.0, 0.0, 0.0])  # Initial guess (x, y, theta)

        # Parameters
        self.resolution = 0.05  # meters per cell
        self.search_window = 1.0  # meters
        self.search_step = 0.1    # meters
        self.angle_step = math.radians(5)  # radians

        self.get_logger().info("scan_map_localizer started")

    def map_cb(self, msg: OccupancyGrid):
        """Store map in numpy array."""
        w, h = msg.info.width, msg.info.height
        self.map = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.map_info = msg.info
        self.resolution = msg.info.resolution
        self.get_logger().info(f"Map received: {w}x{h}, res={self.resolution}")

    def scan_cb(self, scan: LaserScan):
        if self.map is None:
            self.get_logger().warn("Map not received yet, cannot localize.")
            return

        pts = laser_to_points(scan, max_range=scan.range_max)
        self.get_logger().info(f"Received {pts.shape[0]} valid scan points.")

        if pts.shape[0] < 5:
            self.get_logger().warn("Not enough valid scan points to localize.")
            self.publish_pose()  # Still publish last known pose
            return

        best_pose, best_score = None, -1e9
        x0, y0, th0 = self.pose

        xs = np.arange(x0 - self.search_window, x0 + self.search_window, self.search_step)
        ys = np.arange(y0 - self.search_window, y0 + self.search_window, self.search_step)
        ths = np.arange(th0 - math.radians(15), th0 + math.radians(15), self.angle_step)

        for x in xs:
            for y in ys:
                for th in ths:
                    score = self.score_pose(x, y, th, pts)
                    if score > best_score:
                        best_score = score
                        best_pose = (x, y, th)

        if best_pose is not None:
            self.pose = np.array(best_pose)
            self.get_logger().info(f"Best match: x={best_pose[0]:.2f}, y={best_pose[1]:.2f}, "
                                   f"theta={math.degrees(best_pose[2]):.1f}°, score={best_score}")
        else:
            self.get_logger().warn("No good match found. Publishing last known pose.")

        self.publish_pose()

    def score_pose(self, x, y, th, scan_pts):
        """Project scan points into map and count occupied cells."""
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        world_pts = (R @ scan_pts.T).T + np.array([x, y])

        if self.map is None:
            return -1e9

        hits = 0
        for px, py in world_pts:
            mx = int((px - self.map_info.origin.position.x) / self.resolution)
            my = int((py - self.map_info.origin.position.y) / self.resolution)
            if 0 <= mx < self.map.shape[1] and 0 <= my < self.map.shape[0]:
                if self.map[my, mx] > 50:  # occupied
                    hits += 1
            else:
                self.get_logger().debug(f"Point outside map: px={px:.2f}, py={py:.2f}")
        return hits

    def publish_pose(self):
        x, y, th = self.pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        q = quaternion_from_euler(0.0, 0.0, th)
        pose_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMapLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


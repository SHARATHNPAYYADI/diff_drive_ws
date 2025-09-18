#!/usr/bin/env python3
"""
trajectory_recorder_ekf.py
Records:
  - /odom (wheel odometry)
  - /ekf_pose or scan_pose(estimated pose from EKF)
  - Ground truth via TF (map → base_link)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder_ekf')

        self.odom_data = []
        self.gt_data = []
        self.ekf_data = []

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.ekf_cb, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to record TF at 10 Hz
        self.create_timer(0.1, self.record_tf)

    def odom_cb(self, msg: Odometry):
        self.odom_data.append([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def ekf_cb(self, msg: PoseWithCovarianceStamped):
        self.ekf_data.append([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def record_tf(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            ts = self.get_clock().now().nanoseconds * 1e-9
            x = t.transform.translation.x
            y = t.transform.translation.y
            self.gt_data.append([ts, x, y])
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass  # TF not ready yet

    def save_csv(self):
        def save(filename, data, label):
            with open(filename, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['time', 'x', 'y'])
                writer.writerows(data)
            self.get_logger().info(f"Saved {label} → {filename}")

        save('odom.csv', self.odom_data, "Odometry")
        save('ground_truth.csv', self.gt_data, "Ground Truth")
        save('ekf_pose.csv', self.ekf_data, "EKF Pose")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_csv()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


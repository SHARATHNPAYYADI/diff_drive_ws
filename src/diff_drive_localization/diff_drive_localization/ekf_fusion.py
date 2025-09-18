import rclpy
from rclpy.node import Node
import numpy as np
np.float = float
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class EKFFusion(Node):
    def __init__(self):
        super().__init__('ekf_fusion')

        # Declare parameters with defaults (can be overridden by launch)
        self.declare_parameter('process_noise', [0.01, 0.01, 0.01])
        self.declare_parameter('scan_pose_covariance', [0.5, 0.5, 0.1])

        # Load parameters
        q_flat = self.get_parameter('process_noise').value
        r_flat = self.get_parameter('scan_pose_covariance').value
        self.Q = np.diag(q_flat)
        self.R = np.diag(r_flat)

        # State [x, y, theta]
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/scan_pose', self.scan_callback, 10)

        # Publisher
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)

        self.last_time = None

    def odom_callback(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z

        theta = self.x[2, 0]
        dx = vx * np.cos(theta) * dt - vy * np.sin(theta) * dt
        dy = vx * np.sin(theta) * dt + vy * np.cos(theta) * dt
        dtheta = omega * dt

        self.x += np.array([[dx], [dy], [dtheta]])
        self.P += self.Q

        self.publish_estimate(msg.header)

    #Estimating based on EKF filter
    def scan_callback(self, msg: PoseStamped):
        quat = (msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)
        _, _, yaw = euler_from_quaternion(quat)

        z = np.array([[msg.pose.position.x],
                      [msg.pose.position.y],
                      [yaw]])

        H = np.eye(3)
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        self.publish_estimate(msg.header)

    def publish_estimate(self, header):
        msg = PoseWithCovarianceStamped()
        msg.header = header
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])

        q = quaternion_from_euler(0, 0, float(self.x[2]))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.pose.covariance[0] = self.P[0, 0]
        msg.pose.covariance[7] = self.P[1, 1]
        msg.pose.covariance[35] = self.P[2, 2]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""localization/odometry (nav_msgs/Odometry) -> px4_msgs/VehicleOdometry for EKF2 external vision.

Whatever backend runs remaps its output to localization/odometry, so this is the only node that
knows PX4. It is also the only /fmu/in writer outside mullet_core, and writes one topic.

publish_to_px4 false (default, shadow mode): the converted message goes to
localization/vehicle_odometry, so a bag holds exactly what PX4 would have fused.
publish_to_px4 true: it goes to px4_topic. EKF2_EV_CTRL decides whether PX4 fuses it.
"""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from mullet_localization.frames import to_px4, variances

# mullet_core's px4Qos(): best effort, transient local, keep last 1.
PX4_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _f32(v) -> np.ndarray:
    return np.asarray(v, dtype=np.float32)


class Px4EvBridge(Node):

    def __init__(self) -> None:
        super().__init__('px4_ev_bridge')
        publish_to_px4 = self.declare_parameter('publish_to_px4', False).value
        px4_topic = self.declare_parameter('px4_topic', '/fmu/in/vehicle_visual_odometry').value
        self._t = _f32(self.declare_parameter('body_t_base', [0.0, 0.0, 0.0]).value)
        self._q = np.asarray(self.declare_parameter('body_q_base', [1.0, 0.0, 0.0, 0.0]).value)
        self._q /= np.linalg.norm(self._q)

        topic = px4_topic if publish_to_px4 else 'localization/vehicle_odometry'
        self._pub = self.create_publisher(VehicleOdometry, topic, PX4_QOS)
        self.create_subscription(
            Odometry, 'localization/odometry', self._on_odometry, qos_profile_sensor_data)
        self.get_logger().info(
            f'localization/odometry -> {self.resolve_topic_name(topic)}'
            + ('' if publish_to_px4 else ' (shadow mode, PX4 gets nothing)'))

    def _on_odometry(self, msg: Odometry) -> None:
        pose, twist = msg.pose.pose, msg.twist.twist
        p, q, v, w = to_px4(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z],
            [twist.linear.x, twist.linear.y, twist.linear.z],
            [twist.angular.x, twist.angular.y, twist.angular.z],
            self._t, self._q)
        pos_var, att_var, vel_var = variances(msg.pose.covariance, msg.twist.covariance)

        out = VehicleOdometry()
        # Agent (ROS) time; PX4 converts it with UXRCE_DDS_SYNCT=1, the default.
        out.timestamp = self.get_clock().now().nanoseconds // 1000
        out.timestamp_sample = Time.from_msg(msg.header.stamp).nanoseconds // 1000
        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.position = _f32(p)
        out.q = _f32(q)
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD
        out.velocity = _f32(v)
        out.angular_velocity = _f32(w)
        out.position_variance = _f32(pos_var)
        out.orientation_variance = _f32(att_var)
        out.velocity_variance = _f32(vel_var)
        # ponytail: never signals estimator resets; count them once a backend exposes its resets.
        out.reset_counter = 0
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    rclpy.spin(Px4EvBridge())


if __name__ == '__main__':
    main()

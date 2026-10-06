#!/usr/bin/env python3
"""Persistent, display-only publisher. No command/action/driver interfaces."""
import fcntl
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from live_view_data import snapshot


def main():
    root = Path(__file__).resolve().parent
    lock = (root / '.live_view.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return
    rclpy.init()
    node = rclpy.create_node('s_curve_live_view')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    target_pub = node.create_publisher(RosPath, '/s_curve/target_path', qos)
    actual_pub = node.create_publisher(RosPath, '/s_curve/actual_path', qos)
    status_pub = node.create_publisher(Marker, '/s_curve/status', qos)
    previous = None
    def refresh():
        nonlocal previous
        try:
            data = snapshot(root)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            data = ('S-curve', [], [], 'Waiting for readable run data: '+str(exc))
        if data == previous:
            return
        previous = data
        name, target, actual, status = data
        stamp = node.get_clock().now().to_msg()
        for publisher, points in ((target_pub, target), (actual_pub, actual)):
            msg = RosPath()
            msg.header.frame_id = 's_curve_view'
            msg.header.stamp = stamp
            for x, y in points:
                p = PoseStamped()
                p.header = msg.header
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.w = 1.0
                msg.poses.append(p)
            publisher.publish(msg)
        label = Marker()
        label.header.frame_id = 's_curve_view'
        label.header.stamp = stamp
        label.ns = 'run_status'
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(x=2.4, y=-1.2, z=0.1)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = name+'\n'+status+' | poses: '+str(len(actual))+'\nBlue: target / Orange: odom trajectory'
        status_pub.publish(label)
    node.create_timer(0.2, refresh)
    refresh()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()

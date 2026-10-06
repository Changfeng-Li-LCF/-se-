"""Publish a reference Path and record localization; NEVER send motion commands.

Run on the car after sourcing ROS2. No action client or velocity publisher exists
in this tool. A future driving test needs a separate, explicitly started client.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

from route import load_route, nearest_error, relative_pose, transform_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', default=str(Path(__file__).with_name('reference.json')))
    parser.add_argument('--frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--output', default='s_path_recordings')
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be positive')
    route = load_route(args.route)

    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Path as RosPath
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException

    rclpy.init()
    node = rclpy.create_node('s_path_preview_recorder')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(RosPath, '/s_test/reference_path', qos)
    command = {'v': None, 'omega': None, 'received': None}

    def on_command(message):
        command.update(v=message.linear.x, omega=message.angular.z, received=time.monotonic())

    subscription = node.create_subscription(Twist, '/car_cmd_vel', on_command, 10)

    def read_pose():
        tf = buffer.lookup_transform(args.frame, args.base_frame, Time())
        stamp_ns = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not -0.2 <= age <= 0.5:
            raise RuntimeError(f'TF timestamp is stale or clock differs: age={age:.3f}s')
        q, p = tf.transform.rotation, tf.transform.translation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        return (p.x, p.y, yaw), stamp_ns

    log = None
    try:
        end = time.monotonic() + 10
        anchor = None
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                anchor, _ = read_pose()
                break
            except (TransformException, RuntimeError):
                pass
        if anchor is None:
            raise RuntimeError('No fresh map-to-car pose; check localization before previewing')
        destination = Path(args.output) / (time.strftime('%Y%m%d-%H%M%S') + '-' + str(time.time_ns()%1_000_000))
        destination.mkdir(parents=True, exist_ok=False)
        metadata = dict(frame=args.frame, base_frame=args.base_frame, anchor=list(anchor),
                        route=route, motion_commands_sent=False)
        (destination / 'reference_and_anchor.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        log = (destination / 'actual.csv').open('w', newline='', encoding='utf-8')
        fields = ['elapsed_s', 'tf_stamp_ns', 'x_m', 'y_m', 'yaw_rad', 'cross_track_m',
                  'progress_m', 'command_v_mps', 'command_omega_rps', 'command_age_s']
        writer = csv.DictWriter(log, fieldnames=fields)
        writer.writeheader()
        message = RosPath()
        message.header.frame_id = args.frame
        for pose in route['poses']:
            x, y, yaw = transform_pose(pose, anchor)
            p = PoseStamped()
            p.header.frame_id = args.frame
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            message.poses.append(p)
        print('Preview only: no movement commands will be sent.', flush=True)
        print(f'RViz Path topic: /s_test/reference_path; recordings: {destination}', flush=True)
        start = time.monotonic()
        last_publish = last_record = -math.inf
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_publish >= 1:
                message.header.stamp = node.get_clock().now().to_msg()
                for p in message.poses:
                    p.header.stamp = message.header.stamp
                publisher.publish(message)
                last_publish = now
            if now - last_record < 0.1:
                continue
            last_record = now
            try:
                position, stamp_ns = read_pose()
            except (TransformException, RuntimeError):
                continue  # Never record an old transform as a current position.
            x, y, yaw = relative_pose(*position, anchor)
            error, progress = nearest_error(x, y, route['poses'])
            writer.writerow(dict(elapsed_s=now-start, tf_stamp_ns=stamp_ns, x_m=x, y_m=y,
                                 yaw_rad=yaw, cross_track_m=error, progress_m=progress,
                                 command_v_mps=command['v'], command_omega_rps=command['omega'],
                                 command_age_s=None if command['received'] is None else now-command['received']))
            log.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if log is not None:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

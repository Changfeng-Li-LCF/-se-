"""Read-only ROS 2 data reception check; never publishes vehicle commands."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

rclpy.init()
node = Node('racecar_display_reception_check')
stats = {}
start = time.monotonic()

def receive(topic, msg):
    now = time.monotonic()
    entry = stats.setdefault(topic, {'count': 0, 'first_s': now-start, 'last_s': 0, 'max_gap_s': 0})
    if entry['count']:
        entry['max_gap_s'] = max(entry['max_gap_s'], now-start-entry['last_s'])
    entry['count'] += 1
    entry['last_s'] = now-start
    if topic == '/scan':
        entry['frame_id'] = msg.header.frame_id
        entry['points'] = len(msg.ranges)
    elif topic == '/map':
        entry['frame_id'] = msg.header.frame_id
        entry['width'] = msg.info.width
        entry['height'] = msg.info.height
        entry['resolution'] = msg.info.resolution
    else:
        frames = set(entry.get('transforms', []))
        frames.update(t.header.frame_id + ' -> ' + t.child_frame_id for t in msg.transforms)
        entry['transforms'] = sorted(frames)

map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subs = [node.create_subscription(cls, topic, lambda msg, topic=topic: receive(topic, msg), qos)
        for cls, topic, qos in [(LaserScan, '/scan', qos_profile_sensor_data),
                                (OccupancyGrid, '/map', map_qos),
                                (TFMessage, '/tf', qos_profile_sensor_data),
                                (TFMessage, '/tf_static', static_qos)]]
try:
    while time.monotonic()-start < 30:
        rclpy.spin_once(node, timeout_sec=0.2)
    publishers = {}
    for topic in ['/scan', '/map', '/tf', '/tf_static']:
        publishers[topic] = [{'node': info.node_name, 'namespace': info.node_namespace}
                             for info in node.get_publishers_info_by_topic(topic)]
    for entry in stats.values():
        interval = entry['last_s']-entry['first_s']
        entry['observed_hz'] = (entry['count']-1)/interval if interval > 0 else None
    report = {'duration_s': time.monotonic()-start, 'received': stats, 'publishers': publishers}
    text = json.dumps(report, ensure_ascii=False, indent=2)
    Path('/mnt/d/RacecarWork/verification/rviz-data-reception.json').write_text(text, encoding='utf-8')
    print(text)
finally:
    node.destroy_node()
    rclpy.shutdown()

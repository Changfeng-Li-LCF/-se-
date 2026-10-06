"""Passive timestamp comparison; never publishes or calls vehicle services."""
import collections
import json
import statistics
import time
import rclpy
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node('latency_observer_readonly')
rows = collections.defaultdict(list)
state = {}
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
start = time.monotonic()

def record(key, header):
    mono = time.monotonic()
    if mono-start < 2: return
    stamp = header.stamp.sec + header.stamp.nanosec*1e-9
    now = node.get_clock().now().nanoseconds*1e-9
    gid = "observed"
    rows[key+'|'+gid].append([mono-start, (now-stamp)*1000, stamp])

def tf_callback(msg):
    for t in msg.transforms:
        if t.child_frame_id in ('odom','base_footprint'):
            record(t.header.frame_id+'->'+t.child_frame_id, t.header)

def state_callback(msg):
    try: state.update(json.loads(msg.data))
    except ValueError: pass

subs=[node.create_subscription(TFMessage,'/tf',tf_callback,qos),
      node.create_subscription(String,'/racecar_driver/closed_loop_state',state_callback,qos)]
for topic, cls in [('/scan',LaserScan),('/IMU_data',Imu),('/odom',Odometry),
                   ('/scan_matched_points2',PointCloud2)]:

    # rclpy detects arity: keep a two-argument callable.
    def factory(name):
        def receive(msg): record(name,msg.header)
        return receive
    subs.append(node.create_subscription(cls,topic,factory(topic),qos))

while time.monotonic()-start < 10:
    rclpy.spin_once(node, timeout_sec=.05)

out={'observed_s': time.monotonic()-start, 'streams':{}, 'driver_state':state}
for key, values in rows.items():
    ages = sorted(v[1] for v in values)
    out['streams'][key]={'count':len(values),
        'hz': (len(values)-1)/(values[-1][0]-values[0][0]) if len(values)>1 else None,
        'age_ms_min':min(ages),'age_ms_median':statistics.median(ages),
        'age_ms_p95':ages[int(.95*(len(ages)-1))], 'age_ms_max':max(ages),
        'last_stamp':values[-1][2]}
out['publishers']={topic:[{'node':p.node_name,'namespace':p.node_namespace,
                           'gid':bytes(p.endpoint_gid).hex()} for p in node.get_publishers_info_by_topic(topic)]
                   for topic in ['/scan','/IMU_data','/tf','/odom']}
print(json.dumps(out,indent=2))
node.destroy_node()
rclpy.shutdown()

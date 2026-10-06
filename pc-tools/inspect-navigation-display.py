import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path

rclpy.init()
n = Node('inspect_navigation_display')
results = {}
def markers(msg):
    results['trajectory_markers'] = [{'ns': m.ns, 'id': m.id, 'type': m.type,
        'rgba': [m.color.r, m.color.g, m.color.b, m.color.a],
        'points': len(m.points),
        'first': [m.points[0].x,m.points[0].y] if m.points else None,
        'last': [m.points[-1].x,m.points[-1].y] if m.points else None}
        for m in msg.markers if m.points][:8]
def path(topic, msg):
    points = [[p.pose.position.x,p.pose.position.y] for p in msg.poses]
    results[topic] = {'frame': msg.header.frame_id, 'poses': len(points),
        'points_sample': points[::max(1,len(points)//12)], 'last': points[-1:]}
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
subs = [n.create_subscription(MarkerArray, '/trajectory_node_list', markers, q)]
for topic in ['/plan','/received_global_plan','/global_plan','/local_plan']:
    subs.append(n.create_subscription(Path, topic, lambda msg,t=topic: path(t,msg), q))
end = time.monotonic()+10
while time.monotonic()<end:
    rclpy.spin_once(n, timeout_sec=0.2)
results['publishers'] = {t: [p.node_name for p in n.get_publishers_info_by_topic(t)]
    for t in ['/trajectory_node_list','/plan','/received_global_plan','/global_plan','/local_plan']}
print(json.dumps(results, indent=2))
n.destroy_node()
rclpy.shutdown()

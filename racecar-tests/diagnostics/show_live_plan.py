#!/usr/bin/env python3
"""Visualize recorded live-map plan in existing RViz; exits with its SLAM launch."""
import json,pathlib,sys,time
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile,DurabilityPolicy
filename=pathlib.Path(sys.argv[1]);owner=int(sys.argv[2]);proc=pathlib.Path('/proc')/str(owner)/'stat';identity=proc.read_text().split()[21]
data=json.loads(filename.read_text());rclpy.init();node=rclpy.create_node('live_slam_path_preview_only')
pub=node.create_publisher(Path,'/received_global_plan',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def publish():
 try:
  if proc.read_text().split()[21]!=identity:raise RuntimeError('Launch ended')
 except (OSError,RuntimeError):raise KeyboardInterrupt()
 msg=Path();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
 for p in data['path']:
  pose=PoseStamped();pose.header=msg.header;pose.pose.position.x=p['x'];pose.pose.position.y=p['y']
  for k in 'xyzw':setattr(pose.pose.orientation,k,p['q'+k])
  msg.poses.append(pose)
 pub.publish(msg)
timer=node.create_timer(1.,publish)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.shutdown()

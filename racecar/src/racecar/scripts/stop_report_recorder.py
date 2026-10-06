#!/usr/bin/env python3
"""Read-only stop reporting. No motion, stop, reset, or parameter commands."""
import json,math,os,queue,threading,time
from pathlib import Path
from collections import deque
from datetime import datetime

REASONS={
 'ZERO_SPEED_COMMAND':'收到零速停车指令（是否到达终点需结合导航结果）',
 'COMMAND_TIMEOUT':'驾驶指令超过时限未更新',
 'EXPLICIT_EMERGENCY_SERVICE':'调用了急停服务',
 'EXTERNAL_EMERGENCY_TOPIC':'收到外部急停话题',
 'control_blocked':'独立看门狗检测到控制线程阻塞',
 'serial_blocked':'串口写入超时',
 'serial_failed':'串口写入失败',
}
def clean(value):
 if isinstance(value,float) and not math.isfinite(value):return str(value)
 if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
 if isinstance(value,list):return [clean(v) for v in value]
 return value
def description(reason):
 for key,value in REASONS.items():
  if key in reason:return value
 for key,value in [('MAP_BOUNDARY','边界不允许通行，或保护心跳过期'),('VELOCITY_FEEDBACK','有效速度反馈过期'),('IMU_FEEDBACK','有效陀螺仪反馈过期'),('CONTROL_LOOP_BLOCKED','控制循环超过允许间隔')]:
  if key in reason:return value
 return reason
def render(r):
 e=r['event'];b=e.get('before',{});a=e.get('after',{})
 lines=['# 小车停车报告','',f"- 事件编号：{e['id']}",f"- 时间：{datetime.fromtimestamp(e['stamp']).astimezone().isoformat()}",f"- 驱动首次原因：{e['reason']}",f"- 说明：{description(e['reason'])}",f"- 类型：{e.get('kind')}",f"- 采集状态：{r['capture_status']}",'', '| 停车前指标 | 数值 |','|---|---|']
 for k in ['target_v','measured_v','target_w','measured_w','base_steering_deg','yaw_correction_deg','last_written_motor_pwm','last_written_servo_pwm','velocity_feedback_age_s','imu_feedback_age_s','command_age_s','boundary_guard_clear','boundary_guard_age_s']:
  lines.append(f"| {k} | {b.get(k,'无数据')} |")
 lines+=['',f"停车指令写入后 PWM：{a.get('last_written_motor_pwm','无数据')}；急停锁定：{a.get('emergency_stopped','无数据')}。",
 '', '## 边界与导航上下文','', '以下是同时段收到的上下文，不替代驱动确认的首次原因。']
 for x in r.get('context',[]):
  if x['topic'] in ['boundary','rosout','navigation_status']:
   lines.append('- '+json.dumps(x,ensure_ascii=False))
 scan=[x for x in r.get('context',[]) if x['topic']=='scan_motion' and x['received_ros_s']<=e['stamp']]
 if scan:
  latest=scan[-1]['data']
  lines+=['','## 最近一次独立扫描速度估计','']
  lines.append('- '+json.dumps({k:latest.get(k) for k in ['stamp','valid','reason','source_age_ms','source_age_at_callback_ms','callback_to_compute_ms','processing_ms','scans_coalesced','imu_coverage_reason','scan_start_stamp','scan_end_stamp','imu_history_first_stamp','imu_history_last_stamp','imu_history_size','imu_history_generation','imu_start_gap_ms','imu_end_gap_ms','imu_samples_rejected','imu_history_resets','imu_callback_age_at_compute_ms']},ensure_ascii=False))
  lines.append('该项是最近收到的估计；不能把它当成停车瞬间尚未完成计算的那一帧。')
 lines+=['','完整 JSON 包含停车前后驱动状态、里程计与上下文；前后各段只保存有限时间窗口。',
         '驱动节点关闭、进程被强杀或掉电且来不及发出事件时，不能保证生成对应报告。','']
 return '\n'.join(lines)
class ReportStore:
 def __init__(self,root):
  self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True);self.latest_stamp=-math.inf
 def save(self,r):
  r=clean(r)
  event=r['event'];name=event['id']
  if not name or any(c not in '0123456789-_' for c in name):raise ValueError('Invalid stop id')
  day=datetime.fromtimestamp(event['stamp']).strftime('%Y%m%d');folder=self.root/day;folder.mkdir(exist_ok=True)
  data=dict(r,report_json=str(folder/(name+'.json')),report_markdown=str(folder/(name+'.md')))
  for suffix,text in [('.json',json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)),('.md',render(data))]:
   p=folder/(name+suffix);self.atomic(p,text)
   if event['stamp']>=self.latest_stamp:self.atomic(self.root/('latest_stop'+suffix),text)
  self.latest_stamp=max(self.latest_stamp,event['stamp'])
  return data['report_json']
 @staticmethod
 def atomic(p,text):
  temp=p.with_suffix(p.suffix+'.tmp');temp.write_text(text,encoding='utf8');os.replace(temp,p)
class AsyncStore:
 def __init__(self,root,on_error):
  self.store=ReportStore(root);self.items=queue.Queue(128);self.on_error=on_error
  self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
 def submit(self,report):
  try:self.items.put_nowait(report)
  except queue.Full:self.on_error('Stop report queue full: report could not be saved')
 def run(self):
  while True:
   item=self.items.get()
   try:
    if item is None:return
    self.store.save(item)
   except Exception as e:self.on_error('Stop report write failed: '+str(e))
   finally:self.items.task_done()
 def close(self):
  self.items.put(None);self.thread.join(timeout=5)

def main():
 import rclpy
 from rclpy.node import Node
 from rclpy.executors import ExternalShutdownException
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 from std_msgs.msg import String
 from nav_msgs.msg import Odometry
 from rcl_interfaces.msg import Log
 from action_msgs.msg import GoalStatusArray
 class Recorder(Node):
  def __init__(self):
   super().__init__('stop_report_recorder')
   root=self.declare_parameter('output_directory',str(Path.home()/'racecar-tests/stop-reports')).value
   self.history=deque(maxlen=600);self.pending={};self.seen=set();self.output_root=Path(root);self.rate_warnings={}
   self.writer=AsyncStore(root,lambda s:self.get_logger().error(s))
   qos=QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
   self.events=self.create_subscription(String,'/racecar_driver/stop_event',self.event,qos)
   self.state=self.create_subscription(String,'/racecar_driver/closed_loop_state',lambda m:self.structured('driver',m),20)
   self.scan_motion=self.create_subscription(String,'/scan_motion/status',lambda m:self.structured('scan_motion',m),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
   self.boundary=self.create_subscription(String,'/map_boundary_guard/state',lambda m:self.structured('boundary',m),10)
   self.odom=self.create_subscription(Odometry,'/odom',self.odometry,QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT))
   self.logs=self.create_subscription(Log,'/rosout',self.log,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
   self.status=[self.create_subscription(GoalStatusArray,t,lambda m,t=t:self.add('navigation_status',{'topic':t,'status':[x.status for x in m.status_list]}),qos) for t in ['/follow_path/_action/status','/navigate_to_pose/_action/status']]
   self.timer=self.create_timer(.1,self.tick)
   self.get_logger().info('Stop reports enabled: '+root)
  def add(self,topic,data):
   item={'received_ros_s':self.get_clock().now().nanoseconds*1e-9,'topic':topic,'data':data}
   self.history.append((time.monotonic(),item))
  def structured(self,topic,msg):
   try:self.add(topic,json.loads(msg.data))
   except (ValueError,TypeError):pass
  def odometry(self,m):
   p=m.pose.pose.position;q=m.pose.pose.orientation
   self.add('odom',{'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame':m.header.frame_id,'child':m.child_frame_id,'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'vx':m.twist.twist.linear.x,'vy':m.twist.twist.linear.y})
  def log(self,m):
   if m.name=='stop_report_recorder':return
   if m.level>=30 or any(s in m.msg.lower() for s in ['stop','abort','goal reached','succeed','no forward','timeout']):
    skipped=0
    # Keep first/unrelated warnings; rate warnings must not evict sensor context from the ring.
    category=next((x for x in ['control loop missed its desired rate','planner loop missed its desired rate','behavior tree tick rate'] if x in m.msg.lower()),None)
    if category is not None:
     key=(m.name,category);now=time.monotonic();last,skipped=self.rate_warnings.get(key,(-math.inf,0))
     if now-last<1.0:
      self.rate_warnings[key]=(last,skipped+1);return
     self.rate_warnings[key]=(now,0)
    self.add('rosout',{'stamp':m.stamp.sec+m.stamp.nanosec*1e-9,'node':m.name,'level':m.level,'message':m.msg[:2000],'repeated_rate_warnings_skipped':skipped})
  def event(self,m):
   try:
    e=json.loads(m.data);ident=e['id'];stamp=float(e['stamp'])
    if ident in self.seen:return
    if not ident or any(c not in '0123456789-_' for c in ident):raise ValueError('Invalid stop id')
    existing=self.output_root/datetime.fromtimestamp(stamp).strftime('%Y%m%d')/(ident+'.json')
    if existing.exists():
     self.seen.add(ident);return
    self.seen.add(ident)
    if len(self.seen)>2000:self.seen=set(self.pending)|{ident}
    now=time.monotonic();entry={'event':e,'received':now,'pre':[item for t,item in self.history if t>=now-3 and abs(item['received_ros_s']-stamp)<=3.5]}
    self.pending[ident]=entry
    self.writer.submit(self.report(entry,'collecting_after_stop'))
   except (ValueError,KeyError,TypeError) as e:self.get_logger().error('Invalid stop event: '+str(e))
  def report(self,entry,status):
   e=entry['event'];post=[item for t,item in self.history if t>entry['received'] and abs(item['received_ros_s']-e['stamp'])<=3.5]
   return {'event':e,'capture_status':status,'context':entry['pre']+post,'generated_at':datetime.now().astimezone().isoformat()}
  def tick(self):
   now=time.monotonic()
   for ident,entry in list(self.pending.items()):
    if now-entry['received']>=2:
     self.writer.submit(self.report(entry,'complete'));del self.pending[ident]
  def finish(self):
   for entry in self.pending.values():self.writer.submit(self.report(entry,'recorder_shutdown_before_post_window'))
   self.pending.clear();self.writer.close()
 rclpy.init();node=Recorder()
 try:rclpy.spin(node)
 except (KeyboardInterrupt,ExternalShutdownException):pass
 finally:
  node.finish();node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()

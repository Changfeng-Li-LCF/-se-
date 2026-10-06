"""Local report math. Map-frame tracks and progress-constrained error estimates."""
import math
from bisect import bisect_left, bisect_right
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'motion_dashboard'))
from model import build_model as control_model, number, truth, clean


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    a = [number(q.get(k)) for k in ('x', 'y', 'z', 'w')]
    if None in a:
        return None
    norm = math.sqrt(sum(v*v for v in a))
    if norm < 1e-8:
        return None
    x, y, z, w = [v/norm for v in a]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def nearest(times, t, tolerance):
    i = bisect_left(times, t)
    candidates = [j for j in (i-1, i) if 0 <= j < len(times)]
    if not candidates:
        return None
    j = min(candidates, key=lambda j: abs(times[j]-t))
    return j if abs(times[j]-t) <= tolerance else None


def map_track(raw, warnings):
    direct = [p for p in raw.get('map_poses', []) if all(number(p.get(k)) is not None for k in ('stamp','x','y','yaw'))]
    independent = bool(raw.get('map_transform_source'))
    if direct and not independent:
        warnings.append('地图轨迹来自 map_trajectory 记录，时间为采集时刻；位置是定位估计，不是独立实测真值。')
        return sorted(direct, key=lambda r:r['stamp']), '直接记录的 map 坐标轨迹'
    tfs = []
    for r in raw.get('transforms', []):
        a = yaw(r.get('rotation', {})); pos = r.get('translation', {})
        if a is not None and all(number(pos.get(k)) is not None for k in ('x','y')):
            tfs.append((float(r['stamp']), float(pos['x']), float(pos['y']), a))
    tfs = sorted({r[0]:r for r in tfs}.values())
    times = [r[0] for r in tfs]; result = []; skipped = 0
    first_active = min((float(r['stamp']) for r in raw.get('states',[])
                        if truth(r.get('active')) and number(r.get('stamp')) is not None),default=None)
    direct_at = {round(float(r['stamp']),6):r for r in direct}
    for p in raw.get('poses', []):
        if (not independent and p.get('quality') != 'ok') or any(number(p.get(k)) is None for k in ('stamp','x','y','yaw')):
            continue
        t, x, y, a = [float(p[k]) for k in ('stamp','x','y','yaw')]
        if independent and first_active is not None and t < first_active:continue
        original = direct_at.get(round(t,6)) if independent else None
        if original is not None:
            result.append(original);continue
        i = bisect_left(times,t); tf = None
        if i < len(tfs) and times[i] == t:
            tf=tfs[i][1:]
        elif 0 < i < len(tfs) and times[i]-times[i-1] <= .5:
            l,r=tfs[i-1],tfs[i]; f=(t-l[0])/(r[0]-l[0])
            tf=(l[1]+f*(r[1]-l[1]), l[2]+f*(r[2]-l[2]), l[3]+f*wrap(r[3]-l[3]))
        elif not independent:
            j=nearest(times,t,.12)
            if j is not None: tf=tfs[j][1:]
        if tf is None:
            skipped += 1
            continue
        tx,ty,ta=tf; c,s=math.cos(ta),math.sin(ta)
        result.append(dict(stamp=t,x=tx+c*x-s*y,y=ty+s*x+c*y,yaw=wrap(ta+a),
                           quality=p.get('quality'),pose_tf=dict(mode='source_time_recorded_tf')))
    if result:
        if independent:
            warnings.append('轨迹使用独立记录器的位置样本，按源时间匹配地图TF；只插值坐标变换，不补造位置样本。定位跳变仍保留，这不是独立实测真值。')
            if skipped:warnings.append(f'{skipped} 个定位样本缺少同时刻坐标变换，已留空。')
            return result,'独立记录器按源时间匹配的 map 轨迹'
        warnings.append('旧记录缺少直接地图轨迹：已按源时间戳，将 odom 与 map→odom 变换配对重建；不能将它当作实车真值。')
        if skipped: warnings.append(f'{skipped} 个定位样本缺少同时刻坐标变换，已留空。')
        return result,'按时间匹配 TF 重建的 map 轨迹'
    sparse=[]
    for r in raw.get('tracking', []):
        p=r.get('pose_in_plan_frame')
        if p and len(p)>=3:
            sparse.append(dict(stamp=r['stamp'],x=p[0],y=p[1],yaw=p[2]))
    if sparse:
        warnings.append('仅有稀疏的地图坐标采样；轨迹可展示，速度差分和连续偏差可能缺失。')
    return sparse,'稀疏的 map 位置记录' if sparse else '没有运动轨迹记录'


class OrderedProjection:
    """Monotone progress window avoids choosing the other branch at a crossing."""
    def __init__(self, points):
        self.segments=[]; self.ends=[]; self.progress=0.; self.last=None
        total=0.
        for a,b in zip(points,points[1:]):
            ax,ay=a[:2]; dx,dy=b[0]-ax,b[1]-ay; length=math.hypot(dx,dy)
            if length>1e-8:
                self.segments.append((ax,ay,dx,dy,length,total)); total+=length; self.ends.append(total)

    def project(self,x,y):
        travel=math.dist((x,y),self.last) if self.last else 0.
        lo=max(0.,self.progress-.25); hi=self.progress+max(1.,2*travel+.5)
        if self.last is None: hi=3.
        best=None
        for i in range(bisect_left(self.ends,lo),len(self.segments)):
            ax,ay,dx,dy,length,s=self.segments[i]
            if s>hi:break
            f=max(0.,min(1.,((x-ax)*dx+(y-ay)*dy)/(length*length)))
            q=s+f*length
            if not lo<=q<=hi:continue
            rx,ry=ax+f*dx,ay+f*dy; dist=math.hypot(x-rx,y-ry)
            if best is None or dist<best[0]:
                sign=1 if dx*(y-ry)-dy*(x-rx)>=0 else -1
                best=(dist,sign*dist,q,math.atan2(dy,dx))
        self.last=(x,y)
        if best:self.progress=max(self.progress,best[2])
        return best


def build(raw):
    # Reuse only the mature control/IMU statistics, not its old coordinate/error logic.
    warnings=[]; session=raw['session']; states=sorted([s for s in raw['states'] if number(s.get('stamp')) is not None],key=lambda r:float(r['stamp']))
    active=[s for s in states if truth(s.get('active'))]
    track,track_source=map_track(raw,warnings)
    base=control_model(dict(raw,poses=[dict(r,quality='ok') for r in track] if not states else [],reference={}))
    origin=float((active or states)[0]['stamp']) if states else float(track[0]['stamp']) if track else None
    if origin is None: origin=0.
    # control_model chose exactly this origin when states exist. IMU-only data has no motion origin.
    plans=[]
    for i,p in enumerate(raw.get('plans', [])):
        if p.get('frame')!='map':continue
        t=number(p.get('received_ros_s')) or number(p.get('stamp'))
        if t is None:continue
        points=[[q['x'],q['y']] for q in p.get('points',[]) if number(q.get('x')) is not None and number(q.get('y')) is not None]
        if len(points)>1:plans.append(dict(t=t-origin,sequence=p.get('sequence',i+1),points=points))
    plans.sort(key=lambda p:p['t'])
    initial=raw.get('live_plan',{}).get('path_map',[])
    ptimes=[p['t'] for p in plans]
    state_times=[float(s['stamp']) for s in states]
    imu=base['imu']; imu_times=[t for t in imu['t'] if t is not None]
    imu_samples=[v for t,v in zip(imu['t'],imu['yaw_deg']) if t is not None]
    # Establish a one-time relative yaw offset, not a repeated fit to the driven path.
    alignment=None; path=[]; history=[]; projectors={}; last_plan=None
    for r in sorted(track,key=lambda r:r['stamp']):
        t=r['stamp']-origin; x,y,a=r['x'],r['y'],r['yaw']; j=nearest(state_times,r['stamp'],.3)
        is_active=truth(states[j].get('active')) if j is not None else False
        k=bisect_right(ptimes,t)-1
        points=plans[k]['points'] if k>=0 else initial
        # If no motion samples exist before a plan publication, do not use a future plan.
        key=k
        if key not in projectors: projectors[key]=OrderedProjection(points)
        proj=projectors[key].project(x,y) if points else None
        diff=heading_error=None
        if proj:
            diff=proj[1]; heading_error=math.degrees(wrap(a-proj[3]))
        m=nearest(imu_times,t,.1); raw_imu=imu_samples[m] if m is not None else None
        if alignment is None and raw_imu is not None:
            alignment=dict(t=t,offset_deg=math.degrees(wrap(a-math.radians(raw_imu))))
        aligned=math.degrees(wrap(math.radians(raw_imu+alignment['offset_deg']))) if alignment and raw_imu is not None else None
        motion=speed=None
        if history and r['stamp']-history[-1]['stamp']>.3:history=[]
        history.append(r)
        while len(history)>2 and history[1]['stamp']<=r['stamp']-.2:history.pop(0)
        if len(history)>1 and history[0]['stamp']<=r['stamp']-.2<=history[1]['stamp']:
            l,h=history[:2];dt=h['stamp']-l['stamp']
            if 0<dt<=.3:
                f=(r['stamp']-.2-l['stamp'])/dt;dx=x-(l['x']+f*(h['x']-l['x']));dy=y-(l['y']+f*(h['y']-l['y']))
                speed=math.hypot(dx,dy)/.2
                if math.hypot(dx,dy)>=.02:motion=math.degrees(math.atan2(dy,dx))
        path.append(dict(t=t,x=x,y=y,yaw_deg=math.degrees(a),imu_raw_deg=raw_imu,imu_aligned_deg=aligned,
                         motion_deg=motion,position_speed=speed,active=is_active,error=diff,heading_error=heading_error,
                         plan_sequence=plans[k]['sequence'] if k>=0 else '初始参考'))
    errors=[abs(p['error']) for p in path if p['active'] and p['error'] is not None]
    base['stats'].update(max_error=max(errors) if errors else None,mean_error=statistics.mean(errors) if errors else None,
                         pose_samples=len(path),plan_count=len(plans))
    for message,available in [('没有闭环诊断记录，速度与 PWM 图留空。',states),('没有有效 IMU 记录。',raw['imu']),
                              ('本轮未记录实际运动轨迹。',path),('本轮没有成功保存规划路径。',initial or plans)]:
        if not available:warnings.append(message)
    if raw.get('skipped_running'):warnings.append('正在运行的记录暂未读取：'+', '.join(raw['skipped_running']))
    if not session.get('goal_sent'):warnings.insert(0,'本轮未下发行驶目标；下方显示启动失败记录，不会冒充上次成功试跑。')
    writer=raw.get('metadata',{}).get('raw_writer',{})
    if writer.get('dropped'):warnings.append(f"记录器报告丢弃了 {writer['dropped']} 条原始消息；未补造缺失数据。")
    if writer.get('incomplete'):warnings.append('原始记录尚不完整；可稍后重新点击入口读取。')
    # Keep independent session time in event table: do not mix it with first-active time.
    events=[]
    for e in raw.get('events',[]):
        events.append(dict(t=e.get('session_elapsed_s'),stage=e.get('stage'),detail=e.get('reason') or e.get('note') or
            ('规划编号 '+str(e['current_plan_sequence']) if e.get('current_plan_sequence') else '')))
    return clean(dict(base, path=path, plans=plans, initial=initial, map=raw.get('snapshot'),map_kind=raw.get('map_kind'),
        route=raw.get('route',{}),events=events,stop_summary=raw.get('stop_summary',{}),stop_report=raw.get('stop_report',''),
        stack_excerpt=raw.get('stack_excerpt',[]),warnings=warnings,track_source=track_source,
        alignment=alignment,goal_sent=bool(session.get('goal_sent')),origin=origin,
        reason=session.get('reason') if session.get('outcome')!='succeeded' else '导航任务返回成功；制动与清理事件见停车记录。',
        notes=['所有地图轨迹和目标使用 map 坐标系；轨迹是定位估计，不是独立真值。',
               '时间轴零点为首个闭环 active 样本；启动事件表使用独立的“启动后秒数”。',
               '蓝色规划路径按发布时间切换；/plan 记录不构成控制器已采纳该路径的确认。',
               '路径误差为报告按时间选取已发布路径、沿有限前进窗口计算的顺序估计，避免八字交叉处跳到另一支；不是控制器内部进度。',
               '新规划从路径起始段重新匹配；大幅定位跳变或日志缺失时，误差估计可能失真，应结合路径图查看。',
               'IMU 对齐只使用首个时间匹配样本的固定偏置；保留后续漂移和差异，不宣称获得绝对航向标定。',
               '位置差分速度与移动方向取前 0.2 秒 map 位移；它也包含定位修正，不能作为独立测速真值。',
               'PWM、基础舵角和修正角均为软件指令；车辆没有在这些日志中提供实测前轮角度。',
               '终止原因引用本轮日志；不会把启动失败推断成电机、转向或 IMU 故障。']))

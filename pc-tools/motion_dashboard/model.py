"""Convert saved diagnostics to chart data without changing vehicle files."""
import math
import statistics
from bisect import bisect_left
from collections import deque


def number(value):
    try:
        result=float(value)
        return result if math.isfinite(result) else None
    except (ValueError,TypeError):
        return None


def truth(value):
    return value is True or str(value).lower() in ('true','1')


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


def nearest_error(x,y,segments):
    best=None
    for ax,ay,dx,dy,length2 in segments:
        f=max(0.,min(1.,((x-ax)*dx+(y-ay)*dy)/length2))
        rx,ry=ax+f*dx,ay+f*dy
        dist2=(x-rx)**2+(y-ry)**2
        if best is None or dist2<best[0]:
            sign=1 if dx*(y-ry)-dy*(x-rx)>=0 else -1
            best=(dist2,sign*math.sqrt(dist2))
    return best[1] if best else None


def imu_series(rows, origin):
    result={key:[] for key in ('t','wx','wy','wz','yaw_deg')}
    valid={number(r.get('stamp')):r for r in rows if number(r.get('stamp')) is not None}
    last=None
    for stamp,row in sorted(valid.items()):
        if last is not None and stamp-last>.6:
            for values in result.values():values.append(None)
        gyro=row.get('angular_velocity') or {}
        q=row.get('orientation') or {}
        components=[number(q.get(k)) for k in ('x','y','z','w')]
        yaw=None
        if row.get('orientation_available',True) and all(v is not None for v in components):
            norm=math.sqrt(sum(v*v for v in components))
            if norm>1e-6:
                x,y,z,w=[v/norm for v in components]
                yaw=math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
        result['t'].append(stamp-origin)
        for key,axis in [('wx','x'),('wy','y'),('wz','z')]:
            result[key].append(number(gyro.get(axis)) if row.get('gyro_available',True) else None)
        result['yaw_deg'].append(yaw)
        last=stamp
    return result


def add_path_angles(path, imu):
    """Match IMU within 100 ms; displacement bearing uses the preceding 200 ms."""
    samples=[(t,yaw) for t,yaw in zip(imu['t'],imu['yaw_deg']) if t is not None]
    times=[item[0] for item in samples]
    history=deque()
    path['imu_yaw_deg']=[];path['imu_dt_s']=[];path['motion_deg']=[]
    for t,point in zip(path['t'],path['actual']):
        imu_yaw=offset=motion=None
        if t is None or any(v is None for v in point):
            history.clear()
        else:
            index=bisect_left(times,t)
            candidates=samples[max(0,index-1):index+1]
            if candidates:
                match=min(candidates,key=lambda item:abs(item[0]-t))
                if abs(match[0]-t)<=.1 and match[1] is not None:
                    imu_yaw=match[1];offset=match[0]-t
            x,y=point
            history.append((t,x,y))
            start=t-.2
            while len(history)>2 and history[1][0]<=start:
                history.popleft()
            if len(history)>=2:
                a,b=history[0],history[1]
                if a[0]<=start<=b[0] and 0<b[0]-a[0]<=.3:
                    f=(start-a[0])/(b[0]-a[0])
                    dx=x-(a[1]+f*(b[1]-a[1]));dy=y-(a[2]+f*(b[2]-a[2]))
                    if math.hypot(dx,dy)>=.02:
                        motion=math.degrees(math.atan2(dy,dx))
        path['imu_yaw_deg'].append(imu_yaw)
        path['imu_dt_s'].append(offset)
        path['motion_deg'].append(motion)


def build_model(raw):
    warnings=[]
    states=[]
    for row in raw['states']:
        stamp=number(row.get('stamp'))
        if stamp is not None:states.append(dict(row,_stamp=stamp))
    states.sort(key=lambda r:r['_stamp'])
    # Avoid duplicated time samples distorting averages.
    states=list({r['_stamp']:r for r in states}.values())
    poses=[p for p in raw['poses'] if p.get('quality')=='ok' and
           all(number(p.get(k)) is not None for k in ('stamp','x','y'))]
    poses.sort(key=lambda p:float(p['stamp']))
    active=[r for r in states if truth(r.get('active'))]
    origin=(active[0]['_stamp'] if active else states[0]['_stamp'] if states else
            float(poses[0]['stamp']) if poses else 0.)
    if not active:warnings.append('没有闭环激活样本，速度统计留空。')
    if not states:warnings.append('缺少闭环诊断记录，无法显示速度和 PWM。')
    gaps=[b['_stamp']-a['_stamp'] for a,b in zip(states,states[1:]) if b['_stamp']>a['_stamp']]
    period=statistics.median(gaps) if gaps else None
    max_gap=min(1.0,max(.6,(period or .2)*3))
    valid=lambda r:truth(r.get('active')) and truth(r.get('feedback_fresh')) and number(r.get('measured_v')) is not None
    good=[r for r in states if valid(r)]
    peak=max(good,key=lambda r:float(r['measured_v'])) if good else None
    integral=covered=0.
    for a,b in zip(states,states[1:]):
        dt=b['_stamp']-a['_stamp']
        if valid(a) and valid(b) and 0<dt<=max_gap:
            integral+=.5*(float(a['measured_v'])+float(b['measured_v']))*dt
            covered+=dt
    series={key:[] for key in ['t','target_v','measured_v','target_w','measured_w',
            'servo_pwm','motor_pwm','base_angle','correction','command_angle','active','speed_valid']}
    for row in states:
        if series['t'] and row['_stamp']-origin-series['t'][-1]>max_gap:
            # Break rendered lines across missing diagnostic intervals.
            for key in series:series[key].append(None)
        a=number(row.get('base_steering_deg'));b=number(row.get('yaw_correction_deg'))
        yaw_fresh=truth(row.get('imu_feedback_fresh')) if row.get('yaw_feedback_source')=='imu' else truth(row.get('feedback_fresh'))
        values=dict(t=row['_stamp']-origin,target_v=number(row.get('target_v')),
            measured_v=number(row.get('measured_v')) if truth(row.get('feedback_fresh')) else None,
            target_w=number(row.get('target_w')),measured_w=number(row.get('measured_w')) if yaw_fresh else None,
            servo_pwm=number(row.get('last_written_servo_pwm')),motor_pwm=number(row.get('last_written_motor_pwm')),
            base_angle=a,correction=b,command_angle=a+b if a is not None and b is not None else None,
            active=truth(row.get('active')),speed_valid=valid(row))
        for key in series:series[key].append(values[key])
    reference=raw.get('reference',{})
    points=[p for p in reference.get('points',[]) if all(number(p.get(k)) is not None for k in ('x','y'))]
    anchor=reference.get('anchor',points[0] if points else {})
    ax,ay,angle=[number(anchor.get(k)) or 0. for k in ('x','y','yaw')]
    c,s=math.cos(angle),math.sin(angle)
    def local(x,y):return [c*(x-ax)+s*(y-ay),-s*(x-ax)+c*(y-ay)]
    segments=[]
    for a,b in zip(points,points[1:]):
        x,y=float(a['x']),float(a['y']);dx,dy=float(b['x'])-x,float(b['y'])-y
        length2=dx*dx+dy*dy
        if length2>1e-15:segments.append((x,y,dx,dy,length2))
    path=dict(target=[local(float(p['x']),float(p['y'])) for p in points],actual=[],t=[],error=[],headings=[],heading_deg=[])
    last_arrow=None
    arrow_period=max(1., (float(poses[-1]['stamp'])-float(poses[0]['stamp']))/40) if poses else 1.
    errors=[]
    active_end=active[-1]['_stamp'] if active else None
    # Keep jump/invalid samples as gaps in the path instead of joining over them.
    last_stamp=None
    rejected_between=False
    for p in sorted(raw['poses'],key=lambda p:number(p.get('stamp')) or 0.):
        stamp=number(p.get('stamp'));x=number(p.get('x'));y=number(p.get('y'))
        if p.get('quality')!='ok' or any(v is None for v in (stamp,x,y)):
            rejected_between=True;continue
        if last_stamp is not None and (rejected_between or stamp-last_stamp>.6):
            path['actual'].append([None,None]);path['t'].append(None);path['error'].append(None)
            path['heading_deg'].append(None)
        error=nearest_error(x,y,segments)
        path['actual'].append(local(x,y));path['t'].append(stamp-origin);path['error'].append(error)
        yaw=number(p.get('yaw'))
        heading=math.degrees(math.atan2(math.sin(yaw-angle),math.cos(yaw-angle))) if yaw is not None else None
        path['heading_deg'].append(heading)
        if yaw is not None and (last_arrow is None or stamp-last_arrow>=arrow_period):
            hx,hy=local(x,y)
            path['headings'].append(dict(x=hx,y=hy,t=stamp-origin,yaw_deg=heading,path_index=len(path['t'])-1))
            last_arrow=stamp
        if active_end is not None and origin<=stamp<=active_end and error is not None:errors.append(abs(error))
        last_stamp=stamp;rejected_between=False
    if raw.get('skipped_newer'):warnings.append('已跳过 '+str(len(raw['skipped_newer']))+' 个未完成或未行驶的更新记录，当前显示最近一次已完成运行。')
    if not poses:warnings.append('没有有效定位点。')
    if len(good)<len(active):warnings.append(f'速度统计排除了 {len(active)-len(good)} 个无效反馈样本。')
    session=raw.get('session',{})
    span=active[-1]['_stamp']-origin if len(active)>1 else None
    driver=session.get('effective_driver') or raw.get('metadata',{}).get('parameters',{}).get('/racecar_driver',{})
    imu=imu_series(raw.get('imu',[]),origin)
    add_path_angles(path,imu)
    if not imu['t']:warnings.append('这份缓存没有原始 IMU 数据；重新读取该次车端记录可补充。')
    return clean(dict(run_id=raw['run_id'],source=raw['source'],finished=session.get('finished'),
        outcome=session.get('outcome','unknown'),reason=session.get('reason') if session.get('outcome')!='succeeded' else None,
        series=series,path=path,imu=imu,driver=driver,warnings=warnings,
        stats=dict(max_speed=number(peak['measured_v']) if peak else None,
            peak_t=peak['_stamp']-origin if peak else None,mean_speed=integral/covered if covered else None,
            speed_coverage_s=covered,active_span_s=span,max_error=max(errors) if errors else None,
            action_duration_s=number(session.get('elapsed_s')),
            samples=len(states),valid_speed_samples=len(good),pose_samples=len(poses),
            diagnostic_hz=1/period if period else None),
        notes=['速度为里程计反馈估计；最高值是诊断采样峰值，不是独立测速仪测量。',
               '均速按有效且连续的激活反馈时段做时间加权；无效和缺失反馈不按零计算。',
               '舵机 PWM 为驱动成功发送记录，没有真实舵角反馈；基础角和修正角是控制指令。',
               '蓝色箭头来自同一定位样本的 yaw，并按路径显示坐标旋转；等长箭头只表示估计车头方向，不表示速度或目标点方向。',
               '定位轨迹悬停显示车头角、IMU 航向角及移动方向角。车头角与移动方向使用图中坐标系（+X 为 0°，逆时针为正）；IMU 保留传感器原始参考系，不能直接当作同一绝对航向。IMU 匹配最近源时间戳，容差 100 ms，悬停显示匹配时间差。',
               '移动方向用该点之前 0.2 秒的位置差计算；窗口起点在相邻有效定位点间插值。位移不足 2 cm、窗口不足或跨无效定位/记录缺口时留空；它是轨迹位移方向，不是实测轮胎方向。',
               '原始 IMU 曲线使用 /IMU_data 的测量时间戳，未套用驱动滤波；航向角来自 IMU 输出姿态，采用传感器参考系，±180°处会折返，不能与前轮转角直接比较。',
               '两条路径共用起点坐标变换，不重新拟合。偏差为到最近路径线段的有符号距离，左正右负；八字交叉处不代表控制器顺序进度。',
               '时间零点为首个闭环激活样本；曲线包含起步前与停车后的记录。任务时长来自运行日志。']))

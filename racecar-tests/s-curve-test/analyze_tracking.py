#!/usr/bin/env python3
"""Offline reference/trajectory comparison; never changes vehicle parameters."""
import argparse
import csv
import json
import math
from pathlib import Path
from tracking_core import project, stats, inferred_pwm


def read_csv(path):
    if not path.exists(): return []
    rows = list(csv.DictReader(path.open(encoding='utf-8')))
    for r in rows:
        for k in r:
            if k not in ('quality','topic'): r[k] = float(r[k])
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run', help='Output directory of record_tracking.py')
    ap.add_argument('--driver-topic', default='/car_cmd_vel')
    ap.add_argument('--no-plot', action='store_true', help='Write metrics/report without matplotlib')
    args = ap.parse_args()
    out = Path(args.run)
    metadata = json.loads((out/'metadata.json').read_text(encoding='utf-8'))
    anchored = (out/'reference_world.json').exists()
    path = json.loads((out/('reference_world.json' if anchored else 'reference_local.json')).read_text(encoding='utf-8'))
    rows = read_csv(out/'poses.csv'); commands = read_csv(out/'commands.csv')
    if rows and not anchored: raise ValueError('Pose data requires fixed world reference; refusing automatic best-fit alignment')
    result = stats(rows,path)
    result['data_kind'] = metadata.get('data_kind','vehicle_observation')
    result['frame'] = path['frame']
    result['quality_events'] = metadata.get('quality_events',{})
    valid = [r for r in rows if r.get('quality') == 'ok']
    errors = [dict(r,**project(path,r['x'],r['y'],r['yaw'])) for r in valid]
    if errors:
        with (out/'errors.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=list(errors[0]));w.writeheader();w.writerows(errors)
    params = metadata.get('parameters',{})
    driver = params.get('/racecar_driver',{})
    loop_states = []
    raw_path = out/'raw.jsonl'
    if raw_path.exists():
        with raw_path.open(encoding='utf-8') as raw:
            for line in raw:
                try:
                    entry=json.loads(line)
                    if entry.get('topic')=='/racecar_driver/closed_loop_state':
                        loop_states.append(dict(receipt_t_s=entry['t'],**json.loads(entry['message']['data'])))
                except (ValueError,KeyError,TypeError):
                    continue
    closed_loop = (driver.get('speed_feedback_enabled') is True or driver.get('yaw_rate_feedback_enabled') is True or
                   any(r.get('speed_closed_loop') or r.get('yaw_rate_closed_loop') for r in loop_states))
    if loop_states:
        with (out/'closed_loop_state.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=sorted(set().union(*(r.keys() for r in loop_states))))
            w.writeheader();w.writerows(loop_states)
    result['closed_loop_enabled'] = closed_loop
    result['closed_loop_state_samples'] = len(loop_states)
    # Inferred output is calculated ONLY when effective driver parameters were read.
    inferred = []; required = ['speed_epsilon_mps','motor_neutral_pwm','servo_center_pwm','wheelbase_m',
        'left_angle_deg','right_angle_deg','servo_left_pwm','servo_right_pwm','max_speed_mps',
        'forward_pwm_per_mps','reverse_pwm_per_mps','motor_max_pwm','motor_min_pwm']
    if not closed_loop and all(isinstance(driver.get(k),(int,float)) for k in required):
        for c in commands:
            if c['topic'] != args.driver_topic: continue
            motor,servo,angle,saturated = inferred_pwm(c['v'],c['w'],driver)
            inferred.append(dict(c,motor_pwm_estimate=motor,servo_pwm_estimate=servo,
                                 requested_angle_deg=angle,steering_saturated=int(saturated)))
        if inferred:
            with (out/'pwm_estimates.csv').open('w',newline='',encoding='utf-8') as f:
                w=csv.DictWriter(f,fieldnames=list(inferred[0]));w.writeheader();w.writerows(inferred)
    result['inferred_pwm_samples'] = len(inferred)
    result['pwm_note'] = 'Calculated from commands + effective parameters, NOT measured PWM/angle/speed'
    result['steering_saturation_samples'] = sum(r['steering_saturated'] for r in inferred)
    if closed_loop:
        result['pwm_note'] = 'Closed-loop PWM cannot be inferred from Twist alone; use closed_loop_state.csv last_written PWM (software serial-write report, not actuator feedback)'
        result['steering_saturation_samples'] = None
    result['keyboard_messages'] = sum(c['topic']=='/teleop_cmd_vel' for c in commands)
    smoother = params.get('/velocity_smoother',{})
    bounds = None
    if len(smoother.get('max_velocity',[])) == 3 and len(smoother.get('min_velocity',[])) == 3:
        bounds = (float(smoother['min_velocity'][2]),float(smoother['max_velocity'][2]))
    before = [c for c in commands if c['topic']=='/cmd_vel_nav']
    after = [c for c in commands if c['topic']==args.driver_topic]
    clipped = paired = 0
    j = 0
    for c in after:
        while j+1 < len(before) and before[j+1]['t'] <= c['t']: j+=1
        if not before or not 0 <= c['t']-before[j]['t'] <= .15: continue
        paired += 1
        if bounds:
            lo,hi = bounds
            clipped += int((math.isfinite(hi) and before[j]['w']>hi+.005 and abs(c['w']-hi)<.001) or
                           (math.isfinite(lo) and before[j]['w']<lo-.005 and abs(c['w']-lo)<.001))
    result['command_comparison'] = {'paired_samples':paired,'angular_clamp_evidence_samples':clipped,
                                     'note':'Transient pre/post differences alone are not proof of speed clipping'}
    (out/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    if not args.no_plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
        a=axes[0,0]; p=path['points']
        a.plot([r['x'] for r in p],[r['y'] for r in p],label='Fixed reference',color='#007f91')
        if rows: a.plot([r['x'] for r in rows],[r['y'] for r in rows],label='Estimated vehicle pose',color='#d35c29',lw=1)
        a.set_aspect('equal');a.set(xlabel='x (m)',ylabel='y (m)',title='Reference vs localization estimate' if rows else 'Reference only: no vehicle data');a.legend()
        if errors:
            axes[0,1].plot([r['t'] for r in errors],[r['cross_track_m'] for r in errors],color='#d35c29')
            axes[1,0].plot([r['t'] for r in errors],[r['heading_error_deg'] for r in errors],color='#8b4fb5')
        axes[0,1].set(xlabel='Time (s)',ylabel='Signed cross-track error (m)',title='Positive = left of reference')
        axes[1,0].set(xlabel='Time (s)',ylabel='Heading error (deg)',title='Wrapped heading difference')
        for topic,color in [('/cmd_vel_nav','#007f91'),(args.driver_topic,'#d35c29')]:
            c=[r for r in commands if r['topic']==topic]
            if c: axes[1,1].plot([r['t'] for r in c],[r['w'] for r in c],label=topic,color=color)
        axes[1,1].set(xlabel='Time (s)',ylabel='Command angular speed (rad/s)',title='Before / after velocity smoother')
        if before or after: axes[1,1].legend()
        for a in axes.flat: a.grid(alpha=.25)
        fig.suptitle(('SYNTHETIC VALIDATION - NOT A CAR RUN | ' if result['data_kind']=='synthetic_validation' else '')+'Tracking status: '+result['status'])
        fig.savefig(out/'comparison.png',dpi=160);plt.close(fig)
    msg=['# 路径偏差分析', '', '状态：`'+result['status']+'`。', '',
         '轨迹来自定位估计，不是外部测量的真实轨迹。参考路径固定，不做事后平移旋转来缩小误差。', '']
    if result['data_kind']=='synthetic_validation':
        msg += ['**本目录只有合成测试数据，用来验证软件，不能代表实车效果。**','']
    if closed_loop:
        msg += ['本次使用车速/车身转弯角速度闭环。目标值、反馈值和驱动最后成功写出的 PWM 见 closed_loop_state.csv；这些是软件发送记录，不是电调或舵机反馈。不能再用 Twist 的开环公式推算实际输出。','']
    metric=result.get('tracking_metrics')
    if metric:
        m=metric['all']
        msg += [f"横向 RMSE：{m['rmse_m']:.3f} m；95% 绝对误差：{m['p95_abs_m']:.3f} m；最大绝对误差：{m['max_abs_m']:.3f} m。",
                f"航向 RMSE：{m['heading_rmse_deg']:.2f}°；覆盖比例：{result['progress_fraction']:.1%}；终点距离：{result['endpoint_gap_m']:.3f} m。", '',
                '统计按有效采样点计算，包含停车等待段；左右弯分项见 summary.json。partial 只能代表已走过的部分，不能用于宣称全程调优成功。','']
    else:
        msg += ['**目前没有足够的行驶数据，不能给出实车跟踪误差，也不能判断参数调整是否有效。**','']
    if result['keyboard_messages']: msg += ['检测到键盘指令；先确认是否混入自动控制，再解释误差。','']
    if clipped: msg += [f'发现 {clipped} 个接近角速度硬限幅的样本；核对运行时参数是否仍是旧值。','']
    if result['steering_saturation_samples']: msg += ['存在按驱动模型推算的舵机饱和；先查实际车轮角度标定和路径曲率。','']
    if closed_loop:
        msg += ['闭环调参顺序：先比较 target_v/measured_v，再比较 target_w/measured_w；一次只调整一个环的增益。当前反馈来自定位估计。角加减速度限幅已解除，勿继续沿用旧版加速度调参建议。','',
                '参数未自动修改；增益效果需依据用户实测数据判断。','']
    else:
        msg += ['最少调参顺序：', '',
            '1. 本次已经取消角速度幅值限制。之后先用当前其余参数跑同一路径，建立基线。',
            '2. 若左右连续摆动，先只试增大 FollowPath.lookahead_dist；若切弯且无舵机饱和，先只试减小它。每次约 0.05–0.10 m，比较同一路径指标。',
            '3. 若平滑器输出转向指令持续滞后，再检查 max_accel/max_decel 的角分量；不把瞬态差异直接当作限幅。',
            '4. 若左右偏差明显不对称或直线持续偏向一边，先核对舵机中值和实际转角，避免同时改多组参数。', '',
            '不自动修改参数。运行中参数变更记录在 raw.jsonl 的 /parameter_events；若中途改参，应拆分试验重新记录。', '',
            ('图表将在电脑端生成。' if args.no_plot else '![对比图](comparison.png)'), '']
    (out/'analysis.md').write_text('\n'.join(msg),encoding='utf-8')
    print(json.dumps(result,indent=2,ensure_ascii=False))

if __name__ == '__main__':
    main()

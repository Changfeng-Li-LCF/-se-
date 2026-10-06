# S 弯参考路径与最少调参方案

日期：2026-09-27。当前阶段：准备完成，未进行实车行驶测试。

已按用户要求取消速度平滑器的角速度幅值限制；其他控制参数本次未调整。小车未启动，没有发送导航目标或运动指令。

## 1. 已准备的参考路径

| 项目 | 数值与含义 |
|---|---|
| 路径形状 | 直线进入 → 左弯 → 右弯 → 直线退出，曲率连续 |
| 长度 | 5.386 m |
| 中心线范围 | 向前 3.963 m，向左 2.486 m |
| 最小曲率半径 | 1.000 m |
| 最大航向变化 | 相对起始车头 80°，结束恢复起始方向 |
| 前后直线 | 各 0.5 m |
| 点间距离 | 不超过 0.02 m |
| 坐标 | start_local，起点为原点，x 向前，y 向左 |

文件：`reference_local.json`、`reference_local.csv`、`reference_preview.png`、`reference_preview.svg`。

![参考路径，尚无实车轨迹](reference_preview.png)

这些尺寸是中心线范围，布置到 6×6 m 场地时，还要留出车身和偏离路径的空间。当前没有实时定位与障碍数据，所以没有把它绑定到旧地图坐标，也没有宣称已经验证场地可通行。

此路径用于之后的 **固定路径 FollowPath 控制器测试**，届时应使用记录工具生成的 `reference_world.json` 作为同一条控制目标。当前全局规划器的 `minimum_turning_radius=1.5 m`，不会主动规划这条最小半径 1 m 的曲线。发送终点给 NavigateToPose 会重新规划，不能当作固定 S 弯跟踪测试。本次未修改全局规划器，也没有执行 FollowPath。

## 2. 本次取消角速度限幅

建图导航当前配置：

```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [0.20, 0.0, .inf]
    min_velocity: [0.0, 0.0, -.inf]
```

数组依次为 x 线速度、y 线速度、z 角速度。`.inf` / `-.inf` 表示不额外裁剪有限的角速度指令。不能直接删除这两个参数：Nav2 会使用自身默认限幅。

已更新车端源码与安装目录中的 `nav_carto.yaml` 和 `nav.yaml`。普通导航原先没有显式 smoother 速度配置，因此新增了该段，保留 Humble 默认线速度分量 ±0.5，仅取消角速度边界。建图导航仍保持前进上限 0.20 m/s、禁止倒车。底盘在所有入口仍使用统一的 `driver_calibration.yaml`。

配置位置：

```text
/home/bianbu/racecar/src/racecar/config/nav_carto.yaml
/home/bianbu/racecar/src/racecar/config/nav.yaml
/home/bianbu/racecar/install/racecar/share/racecar/config/nav_carto.yaml
/home/bianbu/racecar/install/racecar/share/racecar/config/nav.yaml
```

备份：`/home/bianbu/racecar/calibration-backups/angular-limit-20260926-235515/`。

车端实际安装的 Nav2 速度平滑器在隔离的 ROS_DOMAIN_ID=197、仅 localhost 环境中成功加载了两份配置；仅执行 configure，没有 activate。验证结果见 `angular-limit-validation.json`。正常车端导航仍停止，**这些配置在下次启动时生效**。

保留现有角加速度约束 `max_accel[2]=0.20`、`max_decel[2]=-0.30 rad/s²`，以及驱动的舵机端点。取消角速度数值上限不代表车辆可以无限快转向。Nav2 在速度平滑器中分别裁剪三个速度分量，再处理加速度约束；`scale_velocities=true` 不保证被角速度硬裁剪后的曲率保持不变。[Humble 实现](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_velocity_smoother/src/velocity_smoother.cpp)

## 3. 当前底盘参数与未确定项

| 参数 | 当前保存值 |
|---|---:|
| 轴距 wheelbase_m | 0.248 m |
| 电机中值、下限、上限 | 1500 / 1500 / 1600 |
| 舵机中值、左端点、右端点 | 1489 / 1679 / 1299 |
| 左、右标称最大转角 | 各 30°，实际等效前轮转角尚未测量 |
| 速度换算系数 | 250 PWM / (m/s)，实际速度尚未标定 |
| 底盘最大速度指令 | 0.20 m/s |
| RPP 期望速度 / 弯道最低调节速度 | 0.20 / 0.10 m/s |
| RPP 前视距离 | 0.60 m，未启用随速度变化 |

用户测出的“中值左右各 190 接近最大偏角”确定了 PWM 端点，**没有确定这个端点等于真实 30°**。按现有模型，半径 1 m 的弯需要 `atan(0.248/1) ≈ 13.93°`，对应中值左右约 88 PWM；这只是模型换算值，不是已测到的前轮角度。

按现有速度映射，0.10 m/s 对应电机 PWM 1525，0.20 m/s 对应 1550。1600 是上限，不是导航始终输出的值。如果低速指令下车不走，应先测启动死区/实际速度，单纯提高最大 PWM 不会改变这些换算结果。

## 4. 怎样记录偏差

代码在车端 `/home/bianbu/racecar-tests/s-curve-test/`，电脑源文件在 `D:\RacecarWork\code\s-curve-test\`。

- `record_tracking.py`：只订阅与读取参数，不启动其他节点，不发布控制指令，不发送 action，不打开串口。
- `tracking_core.py`：几何投影、误差统计、按当前驱动公式推算 PWM。
- `analyze_tracking.py`：离线生成误差 CSV、统计 JSON、对比图和分析说明。
- `make_reference.py`：离线生成路径 JSON、CSV 和预览图。
- `test_tracking.py`：几何、误差符号、角度跨 ±π、静止/无数据、端点限幅等验证。

记录采用新鲜的 `odom → base_footprint` TF。首次收到新鲜位姿时，只将局部路径平移旋转一次，并保存 `reference_world.json`。终端出现 `Reference locked` 表示路径坐标已经固定；将来的试跑必须在这之后使用这份路径。之后不做事后拟合对齐。重复同一路径实验时使用同一份 world reference；如果重启定位导致 odom 原点变化，必须重新建立一致坐标，不能直接沿用旧坐标。

记录内容：

| 文件 | 内容 |
|---|---|
| poses.csv | 位姿、TF 时间、接收时间、数据质量 |
| commands.csv | /cmd_vel_nav、/car_cmd_vel、/cmd_vel、/teleop_cmd_vel 原始指令 |
| raw.jsonl | TF、里程计、激光、常见 IMU 话题、ROS 日志、路径、action 状态与参数变更事件；不是 rosbag |
| metadata.json | 运行中参数、节点、话题发布者、缺失数据、记录状态 |
| source_*.yaml / installed_*.yaml | 源配置与安装配置快照，用于区分保存值和运行值 |
| errors.csv | 每个位姿投影到最近路径线段后的横向误差、航向误差、路径进度 |
| summary.json | RMSE、95% 误差、最大误差、左右弯统计、终点距离、路径覆盖比例 |
| pwm_estimates.csv | 有运行时驱动参数时，根据指令推算 PWM；不是下位机反馈 |
| comparison.png / analysis.md | 对比图和自动整理的结果 |

横向误差向路径左侧为正；航向误差限制在 ±180° 内。投影到线段，不仅寻找离散点，避免路径采样间距带来额外误差。过期 TF 不计入轨迹，异常跳变单独标记；定位误差仍会影响指标，不能将 SLAM 轨迹当作外部测量真值。

**无位姿、几乎没走、只走部分路径、完整走完，分别标记。** 未动不能被解释为“误差为零”。统计按采样点计算，停车等待会影响均值，因此比较试验时应保持记录窗口一致；中途改参数需要拆分重新记录。

只启动记录的 MobaXterm 命令如下。它不会让小车跑；目前定位进程停止，因此若仍未启动定位，会在 15 秒后报告没有新鲜位姿并退出。

```bash
source /opt/ros/humble/setup.bash
source ~/racecar/install/setup.bash
cd ~/racecar-tests/s-curve-test
python3 record_tracking.py \
  --output "runs/record-$(date +%Y%m%d-%H%M%S)" \
  --duration 120
```

电脑已经有绘图库，可把某次记录目录复制回电脑后执行：

```powershell
& 'D:\RacecarTools\envs\yolov5\python.exe' `
  'D:\RacecarWork\code\s-curve-test\analyze_tracking.py' `
  'D:\RacecarWork\reports\s-curve-preparation\某次记录目录'
```

之后真正执行固定路径测试时，还需在用户允许行驶后，将该次固定路径传给控制器；记录脚本本身不承担发车功能。

## 5. 尽量少改参数的顺序

**本次已经改了角速度限幅这一组，先保留其余值，之后用一次同路径试验建立新基线。现在没有证据支持再同时改多组。**

原来半径 1 m、速度指令 0.1 m/s 时，理想角速度约为 0.1 rad/s，超过原上限 0.07；在该速度不变的条件下，后者对应约 1.43 m 半径。这说明旧限幅可能妨碍转弯，不代表已经测得它是全部误差来源。此限制现已移除，下一次日志仍应确认运行值。

| 之后日志和轨迹呈现的现象 | 优先核查 | 最先只动的参数 |
|---|---|---|
| 左右来回摆动，指令频繁反向，无明显端点饱和 | 定位是否跳变，指令是否混入键盘 | `FollowPath.lookahead_dist`，先从 0.60 增到 0.65 或 0.70 m 试一次 |
| 稳定地切弯/转向偏迟，定位正常，指令无明显延迟/饱和 | 参考路径与控制器收到的路径是否一致 | 同一个前视距离，从 0.60 降到 0.55 或 0.50 m 试一次；过小可能增加摆动 |
| 平滑前已转向，平滑后明显迟到 | 对比角速度变化率，排除正常起步过渡 | 确认受限后再改角加减速度对应项；不预先一起改 |
| 直线持续向一侧偏，左右误差不对称 | 机械回中、左右实际轮角、定位偏差 | 确认后只改中值，或分别校准左右角度映射 |
| 指令发出但低速不走/走走停停 | PWM 1525 附近是否足以启动、实际速度 | 先标定死区和速度映射，不先改前视距离 |
| 长时间推算舵机已到端点仍跟不上 | 实际最大轮角、指令曲率、机械情况 | 先验证角度标定与路径可达性，不能再加大已测端点 |

这里的候选数值是单变量试验起点，不是已经验证有效的最终参数。前视距离变动后，比较横向 RMSE/P95、最大误差、航向误差、完整覆盖率、终点距离和是否出现振荡；不能只挑某个指标下降就宣布更好。

由于 `use_velocity_scaled_lookahead_dist=false`，当前首先试的只有 `lookahead_dist`，无需为了固定前视距离同步改 min/max_lookahead_dist。底盘参数如需调整，继续修改统一的 `driver_calibration.yaml`，让所有场景一致；导航调整则核对所有在用入口的配置。

## 6. 本次已经获得的数据与验证

- `preparation-no-motion-20260927/`：车端只读观察约 5 秒，0 个位姿、0 条运动指令，结果为 `no_data`。没有实车偏差数值。
- `synthetic-aligned-validation-20260927/`：隔离 ROS 域里的合成 TF 验证数据，529 个有效位姿，验证记录到离线分析的链路。未发布运动话题，不能当作实车表现。另保留 `synthetic-validation-20260927/` 的启动错位样本：它显示提前移动再固定参考路径会产生偏差，分析器没有把这种错位自动消除。
- 7 项离线单元验证通过，覆盖几何、参考坐标变换、误差计算和无数据判定。
- `historical_idle_observation/`：之前约 10.23 秒旧观测，没有收到控制指令，定位 x/y 范围变化约 2.16/2.49 cm。它早于当前舵机端点、电机上限和角速度配置，只用于提醒定位本身也会变化；不是当前 S 弯实测，也不是精确定位噪声标定。

因此，当前可交付的是固定参考路径、被动记录工具、分析工具、参数快照与验证记录。**真实跟踪偏差及最有效的参数调整，需要之后允许行驶并采到数据才能确定。**

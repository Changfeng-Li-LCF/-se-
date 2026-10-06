# gmapping.sh 建图调用流程说明

## 文件说明

**文件路径**: `/home/zk/mnt/bianbu/zk/racecar/gmapping.sh`

## 调用链

### 1. 启动主程序
```bash
bash gmapping.sh
```

### 2. 执行流程

#### 步骤 1: 启动基础驱动和传感器 (Run_car.launch.py)
```bash
ros2 launch racecar Run_car.launch.py &
```

**包含的节点**:
- 激光雷达驱动 (lslidar_driver_node)
- IMU 发布器 (publisher_imu_node)
- EKF 融合 (ekf_node)
- rf2o_laser_odometry (rf2o_laser_odometry_node) ← 已修复
- 其他传感器和 TF 发布器

**等待时间**: 5 秒 (让驱动完全初始化)

#### 步骤 2: 启动 SLAM 建图 (slam_gmapping)
```bash
ros2 launch slam_gmapping slam_gmapping.launch.py &
```

**包含的节点**:
- slam_gmapping - 基于激光扫描的 SLAM 建图节点

**参数**:
- `use_sim_time: false` (使用实时时间，不是仿真时间)

### 3. 节点依赖关系

```
gmapping.sh (主脚本)
    │
    ├─→ Run_car.launch.py (基础驱动)
    │    ├─→ lslidar_driver_node (激光雷达)
    │    ├─→ publisher_imu_node (IMU)
    │    ├─→ ekf_node (融合定位)
    │    ├─→ rf2o_laser_odometry_node (激光里程计)
    │    └─→ racecar_driver_node (底盘驱动)
    │
    └─→ slam_gmapping.launch.py (SLAM 建图)
         └─→ slam_gmapping (建图节点)
              ├─ 输入: /scan (激光扫描)
              ├─ 输入: /odom_rf2o (里程计)
              └─ 输出: /map (地图)
```

### 4. 关键话题

| 话题 | 来源 | 用途 | 说明 |
|------|------|------|------|
| `/scan` | lslidar_driver_node | 激光数据 | 原始激光扫描 |
| `/odom_rf2o` | rf2o_laser_odometry_node | 里程计 | 基于激光的里程计 |
| `/odom_combined` | ekf_node | 融合里程计 | IMU + rf2o 融合 |
| `/map` | slam_gmapping | 建图输出 | 建立的栅格地图 |
| `/tf` | 多个节点 | 变换树 | 坐标系转换 |

## 使用方法

### 方式 1: 直接运行建图脚本
```bash
cd ~/zk/racecar
bash gmapping.sh
```

### 方式 2: 在另一个终端监视建图进度
```bash
# 查看话题
ros2 topic list

# 监视地图
ros2 topic echo /map --once | head -20

# 查看节点
ros2 node list

# 可视化（在有 X11 显示的情况下）
rviz2 -d racecar/rviz/rviz2_gmapping.rviz
```

### 方式 3: 手动启动（分步骤）
```bash
# 终端 1: 启动基础驱动
cd ~/zk/racecar
source install/setup.bash
ros2 launch racecar Run_car.launch.py

# 终端 2: 启动建图（在驱动完全启动后）
source ~/zk/racecar/install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

## 结束建图

### 方式 1: 按 Ctrl+C
```
^C
```
脚本会自动终止所有后台进程

### 方式 2: 在另一个终端手动停止
```bash
pkill -f slam_gmapping
pkill -f "ros2 launch racecar"
```

## 保存地图

建图完成后，保存地图：

```bash
# 1. 启动地图服务器（如果还在运行）
ros2 run nav2_map_server map_saver_cli -f ~/racecar_map

# 2. 地图文件位置
# - ~/racecar_map.pgm (栅格地图)
# - ~/racecar_map.yaml (地图元数据)
```

## 常见问题

### 1. slam_gmapping 没有生成地图
- **检查**: `/scan` 话题是否在发布
  ```bash
  ros2 topic echo /scan --once
  ```
- **检查**: 激光雷达是否正常工作
  ```bash
  ros2 topic list | grep scan
  ```

### 2. 地图质量差
- **原因**: 可能需要更多的运动或更好的里程计
- **改进**: 调节 slam_gmapping 参数（见下文）

### 3. 建图速度慢
- **原因**: 正常，取决于环境大小和运动速度

## slam_gmapping 配置

如需调整 slam_gmapping 参数，编辑：
`/home/zk/mnt/bianbu/zk/racecar/src/slam_gmapping/launch/slam_gmapping.launch.py`

常见参数:
```python
parameters=[{
    'use_sim_time': False,           # 使用实时时间
    'inverted_laser': False,         # 激光反向
    'throttle_scans': 1,            # 扫描节流
    'base_frame': 'base_footprint', # 基础框架
    'map_frame': 'map',             # 地图框架
    'odom_frame': 'odom',           # 里程计框架
    # ... 其他参数
}]
```

## 工作流建议

1. **启动驱动**: `bash gmapping.sh`
2. **驱动机器人**: 在环境中移动，让激光扫描覆盖全区域
3. **观察地图**: `ros2 topic echo /map`
4. **完成后保存**: 按 Ctrl+C 停止，然后保存地图

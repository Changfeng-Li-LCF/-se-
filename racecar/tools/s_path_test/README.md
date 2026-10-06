# S 弯路径：当前只做准备与预览

当前没有发送导航目标、FollowPath 动作或电机指令，也没有修改底盘及导航参数。

## 已准备的路线

- 相对测试开始时的小车位置和朝向生成：x 向前，y 向左。
- 向前 4 m，左、右各偏移最多 0.25 m；总路程约 4.17 m。
- 起点和终点的切线都与初始车头一致，曲率从零开始、在终点回到零。
- 按轴距 0.248 m 计算，最小模型转弯半径约 0.785 m，最大模型转角约 17.5°。这些是几何估算，并非已校准的实际前轮角度或舵机 PWM。
- reference.json 和 reference.csv 是参考路径；s_path_preview.png 是预览图。

## 车端预览与记录（不会驱动车辆）

将这个目录放在车端 ~/racecar/tools/s_path_test 后，在已启动建图且定位正常的情况下运行：

```bash
source /opt/ros/humble/setup.bash
source ~/racecar/install/setup.bash
cd ~/racecar/tools/s_path_test
python3 preview_record.py --duration 120
```

在 RViz 中添加一个 Path 显示，话题选择 /s_test/reference_path，Fixed Frame 使用 map。

脚本只发布这一条显示用的 Path；它不包含动作客户端或速度发布器。每次运行都用当时的小车位姿确定路径的起点，因此摆正车头以后再预览。

记录保存在当前目录的 s_path_recordings 下，包含参考路线、起点位姿及 actual.csv。实际轨迹来自建图定位估计，不能作为独立的地面真值；也不能仅凭它分辨舵机、速度标定与定位误差。

actual.csv 记录横向误差、沿路径进度及收到的 /car_cmd_vel 指令。路径起止点之外的误差采用到最近端点的距离；command_age_s 用于判断记录的速度指令是否已经过期。出现定位数据中断时不伪造轨迹点。

## 后续实车跟踪方式

准备使用现有 Nav2 /follow_path 接口，让当前控制器跟踪指定路径，并对比参考路径与实际轨迹。只给终点会让规划器自行选路，因此不能保证进行这条 S 弯测试。

发车前需要现场可立即停车的人，并核对当时运行的车速限幅、左右角度限幅、定位和雷达数据。当前键盘程序持续发布的停车指令可能与导航争用底盘，测试时需要明确切换控制来源。现在没有切换或停止键盘程序。

这个目录目前没有自动发车功能。预览路径也不会触发自动跟踪。

## 本地重新生成与几何验证

```bash
python3 generate.py
python3 -m unittest -v test_route.py
```

路线生成不需要 ROS。生成图片需要 matplotlib；预览和记录需要车端 ROS2、tf2_ros 和消息包。没有写死任何电脑或小车 IP。

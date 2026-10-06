"""Menu descriptions only; importing this module never starts an operation."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Action:
    key: str
    group: str
    title: str
    description: str
    example: str


ACTIONS = [
    Action('explorer', '建图与遥控', '启动 Explorer 建图',
           '启动底盘、雷达、IMU、Cartographer 和 RViz。停止路径测试留下的后台后启动；若另一个 Explorer 正在运行，请先在原终端退出。此项不发送导航目标。',
           'bash ~/racecar/explorer.sh'),
    Action('keyboard', '建图与遥控', '打开键盘控制',
           '先启动 Explorer，然后在新标签页按 i 前进、u/o 转弯、空格或 k 停车。车速使用车端当前配置。Ctrl+C 退出键盘程序。',
           'ros2 run racecar racecar_teleop.py'),
    Action('save_map', '建图与遥控', '保存当前地图',
           '停车后保持 Explorer 运行。保存带时间戳的 YAML、PGM 到 ~/maps，并尝试保存 Cartographer 的 pbstream，不覆盖旧地图。',
           'ros2 run nav2_map_server map_saver_cli -f ~/maps/manual_loop_<时间戳>'),
    Action('view_map', '建图与遥控', '选择并查看已保存地图',
           '在终端列出地图，输入编号，直接回车选择最新地图。使用独立地图话题打开 RViz；Ctrl+C 结束此次查看。',
           '列出 ~/maps 和 ~/racecar/src/racecar/map 中的地图 → 选择 → RViz'),
    Action('s_curve', '自动路径', '运行 S 弯（会驱动车辆）',
           '使用 S 弯参考路径和当前车端参数，记录轨迹并打开路径显示。沿用原测试的 --allow-unknown-map；点击执行后会启动自动行驶。',
           '原 run_session.py + 本次 S 弯参考路径 + --allow-unknown-map'),
    Action('figure8', '自动路径', '运行 8 字（会驱动车辆）',
           '使用 8 字参考路径和当前车端参数，记录轨迹并打开路径显示。沿用原测试的 --allow-unknown-map；点击执行后会启动自动行驶。',
           '原 run_session.py + 本次 8 字参考路径 + --allow-unknown-map'),
    Action('rviz', '自动路径', '打开目标与实际路径 RViz',
           '打开现有路径测试的 RViz 配置。需要相应路径发布程序仍在运行，才能看到目标与实际轨迹。',
           'rviz2 -d ~/racecar-tests/s-curve-test/s_curve_live.rviz'),
    Action('stop', '停止与复位', '发送急停（锁定）',
           '调用驱动的急停服务。之后需要停止命令发布程序，再手动复位才能继续行驶；网络命令不能替代现场断电。',
           'ros2 service call /racecar_driver/emergency_stop std_srvs/srv/Trigger "{}"'),
    Action('reset', '停止与复位', '解除急停锁定',
           '先退出键盘程序和正在发车的任务，至少等待 0.5 秒。只有终端返回 success: true 才代表复位成功；复位本身不发运动指令。',
           'ros2 service call /racecar_driver/reset_emergency_stop std_srvs/srv/Trigger "{}"'),
    Action('stop_stack', '停止与复位', '停止路径测试后台',
           '停止 S 弯／8 字测试工具管理的后台。手动启动的 Explorer 需要在其原标签页按 Ctrl+C 退出。',
           'bash ~/racecar-tests/s-curve-test/run_s_curve.sh --stop-stack'),
    Action('params', '参数与代码', '查看已保存和运行中的参数',
           '先显示源文件中的底盘配置，再查询运行中驱动的参数。两者可能不同：配置修改后需要同步并重启。',
           'cat ~/racecar/src/racecar/config/driver_calibration.yaml\nros2 param dump /racecar_driver'),
    Action('sync', '参数与代码', '同步配置到安装目录',
           '将车端源码里的 YAML/Lua 配置同步到安装目录，并备份被替换的文件。不重启节点；下次启动对应模式生效。',
           '~/racecar/src/racecar/config → ~/racecar/install/racecar/share/racecar/config'),
    Action('build', '参数与代码', '编译并安装车端代码',
           '先退出底盘程序。编译现有 racecar、驱动、控制器及 IMU 包和依赖，再同步配置；不自动启动车辆。编译可能需要几分钟。',
           'colcon build --packages-up-to racecar racecar_driver <已有控制器/IMU包> --executor sequential'),
]
BY_KEY = {a.key: a for a in ACTIONS}

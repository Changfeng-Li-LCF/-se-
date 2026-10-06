# 小车 ROS 2 工程与配套工具

本目录对应 2026-10-06 从实车读取的源码快照，以及电脑上当前的配套工具。
仓库原有 Visual Studio 项目保留在根目录，小车相关内容放在下面的独立目录。

| 目录 | 内容 |
| --- | --- |
| `racecar/src` | 完整 ROS 2 源码：底盘驱动、传感器、定位桥接、规划器、控制器及配置 |
| `racecar` 根目录 | 小车启动入口、建图/导航脚本与使用说明 |
| `racecar-tests/s-curve-test` | S 弯、8 字、标定路线、未知区域探索的运行与记录/分析源码 |
| `racecar-tests/diagnostics` | 当前诊断入口源码 |
| `maps/explore_20260930_181005` | 当前标定路线引用的地图 |
| `pc-tools` | Windows 网络发现、SSH/VS Code 入口、RViz 转接、命令菜单、网页报告工具 |
| `docs/*-source-manifest.json` | 导出时的文件清单和 SHA-256 |

## 车端部署

源码来自 `/home/bianbu/racecar` 与 `/home/bianbu/racecar-tests`。
当前部分脚本使用这些绝对路径。部署到相同用户目录时，把仓库中的 `racecar`、
`racecar-tests`、`maps` 目录放回 `/home/bianbu/`；其他用户需调整相应路径。

车端环境为 ROS 2 Humble。先安装各包 `package.xml` 与 `CMakeLists.txt` 声明的依赖
（包括 Nav2、Cartographer ROS、OMPL、libpcap 与 Python 依赖），再编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/racecar
colcon build --symlink-install
source install/setup.bash
```

未知区域探索使用的独立本地库需从源码编译一次：

```bash
cd ~/racecar-tests/s-curve-test
g++ -std=c++17 -O3 -shared -fPIC exploration_fields.cpp -o libexploration_fields.so
```

标定路线运行入口：

```bash
bash ~/racecar-tests/s-curve-test/run_marked_route.sh --restart-stack
```

这份快照上传没有启动导航或发送运动命令，也没有改动车端参数。

## 电脑配套工具

保留现有工具的目录布局时，`pc-tools` 对应 `D:\RacecarWork\tools`。
复制 `racecar-connection.example.json` 为 `racecar-connection.json`，填写自己的小车 IP、
SSH 密钥路径及经核对的 `known_hosts` 文件路径；网络变化由统一配置和网络发现代码处理。
真实 SSH 私钥、密码及本机认证配置未上传。

网页工具依赖 Python，RViz 转接还需工具说明中对应的 WSL/ROS 2 环境。
`motion_dashboard` 和 `marked_route_report` 共用 `motion_dashboard/assets/plotly.min.js`，
部署时保留两目录的相对位置。菜单的本机 MobaXterm 路径可在 `command_menu/settings.json` 调整。

## 快照范围

包含当前代码、配置、必要地图和已有源码测试；不包含编译/安装产物、试跑日志、
历史部署备份、录屏、大型训练模型或系统 ROS 2 安装。Cartographer/Nav2 等系统依赖
通过系统安装获取；各源码包保留原有许可证与版权声明。
本次仅做源码快照完整性与 Git 提交检查，没有重跑编译、模拟测试或实车试跑。

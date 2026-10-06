# 标定路线桌面报告

双击桌面 **查看标定路线报告**，读取车端最近一次已结束的 `marked-route-*` 运行，在默认浏览器打开报告。启动失败也会被选中，不会悄悄显示上一次成功运行。尚在运行的记录暂时跳过，并在页面提示。

数据只读获取；使用共享连接配置和已固定的 SSH 主机身份，支持 IP 变化及地址发现，不改写连接配置。连接失败时显著提示并显示本地缓存。首次读取某轮后可离线浏览；重新双击入口才会获取车端新数据。

报告包括保存地图、标定点/辅助点、按发布时间切换的规划路径、定位轨迹、时间回放、速度、PWM、转向修正、角速度、对齐后的航向对比、路径偏差、停车报告和事件日志。缺失数据明确留空。

入口：`D:/RacecarWork/tools/marked_route_report/run_report.py`

输出：`D:/RacecarWork/reports/marked-route-dashboard/index.html`。每轮另存独立 HTML；历史下拉框列出最多 30 份电脑已下载的报告。

```powershell
& D:\RacecarTools\envs\yolov5\python.exe D:\RacecarWork\tools\marked_route_report\run_report.py
# 仅打开本机缓存
& D:\RacecarTools\envs\yolov5\python.exe D:\RacecarWork\tools\marked_route_report\run_report.py --offline
# 指定车端某次标定路线
& D:\RacecarTools\envs\yolov5\python.exe D:\RacecarWork\tools\marked_route_report\run_report.py --run marked-route-20260930-192905-242091
```

不安装车端节点或脚本，不发送导航目标或底盘命令。工具单独存放，复用原运动报告的控制统计、网络发现与本地图表库，不修改原运动报告。

## 数据口径

- map 轨迹优先使用本轮直接记录；旧记录用源时间戳对齐 odom 与 map→odom TF，缺少配对时留空。不用当前 TF 解释历史轨迹。
- 同一时间轴以首个闭环激活样本为零；启动事件表保留独立的启动后秒数。
- /plan 是规划器发布记录，不保证控制器已经采纳；报告对此明确标注。
- 路径偏差为有限前进窗口内的顺序匹配估计，不冒充控制器内部进度。定位大跳变、长缺口和大范围偏航时可能失真。
- IMU 使用首个匹配样本的固定角度偏置进行对齐，保留原始航向，可切换显示。此举不是 IMU 绝对标定。
- PWM 为软件指令；位置差分速度含定位修正；均非独立实测真值。
- 停车原因引用日志，不把未记录的信息猜成具体硬件故障。

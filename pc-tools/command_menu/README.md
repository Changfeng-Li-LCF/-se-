# 小车常用命令

双击桌面「小车常用命令」，在左侧选择操作，点击「发送并执行」。程序会在 MobaXterm 新标签页连接车端并执行；输出和交互都在该标签页中。菜单启动本身不连接、不启动底盘、不发运动指令。

## 常用流程

- 键盘建图：启动 Explorer → 打开键盘控制 → 在键盘标签页驾驶 → 停车 → 保存当前地图。保持 Explorer 运行直到保存成功。
- 查看地图：选择并查看已保存地图 → 在新终端选择编号，直接回车选最新地图。
- 自动试跑：选择 S 弯或 8 字 → 发送并执行。这两项会让小车自动行驶，沿用现有测试的参数、记录器、RViz 和 `--allow-unknown-map`。
- 参数更新：在车端 VS Code 修改配置 → 同步配置到安装目录 → 退出并重新启动对应模式。菜单不会自动重启运行中的节点。
- 代码更新：先退出底盘程序 → 编译并安装车端代码 → 等待编译完成后再启动模式。
- 急停：发送急停会锁定底盘；退出键盘等指令发布程序，等待后再选择解除急停。以终端返回的 `success` 字段为准。

长时间运行的操作使用独立标签页。要退出它，在相应标签页按 Ctrl+C；关闭菜单不会自动终止车端任务。程序不向已有终端模拟按键，避免把命令写进正在运行的程序。

## 文件与连接

- 桌面入口：`小车常用命令.lnk`。
- `menu.py`：菜单界面。
- `backend.py`：读取连接配置、安装命令工具、打开 MobaXterm 标签页。
- `catalog.py`：操作标题、说明和命令预览。
- `settings.json`：MobaXterm 路径；更换安装位置时修改这里。
- `payload/dispatch.py`：车端执行逻辑。
- `terminal_sessions/`：每次发送的独立脚本，方便复查。
- 统一连接配置：上一级的 `racecar-connection.json`。使用已有 `racecar_network.py` 自动寻找并验证小车；不依赖旧书签里的固定 IP。

只使用已有 SSH 密钥及固定主机身份，不保存密码或关闭主机校验。MobaXterm 的 SSH 连接使用明确参数，不读取它的全局 SSH 默认配置。

首次执行会把工具安装到车端 `~/.local/share/racecar-command-menu/versions/<版本>/`。安装步骤只写菜单自己的文件，不导入或执行车辆操作；相同版本重复安装只核对内容。不会覆盖 `~/racecar` 的源码或参数。只有用户选择同步配置或编译时，才执行对应写入。

## 验证记录

- Windows：`test_backend.py`，验证命令转义、安装幂等、路径限制、已有文件被修改时拒绝覆盖。
- Windows：`menu.py --smoke-test`，验证 GUI 加载、全部 13 项命令及搜索。
- WSL：`test_payload.py`，验证两条路径、内存内路径选择、配置备份和同步。
- MobaXterm：新标签经 SSH 输出 `MENU_TRANSPORT_OK`，退出码 0。
- 未通过菜单启动 Explorer、键盘、S 弯或 8 字，未做实车运动测试。

依据 MobaXterm 官方 `-newtab` 接口：[官方命令行说明](https://blog.mobatek.net/post/mobaxterm-command-lines/)。

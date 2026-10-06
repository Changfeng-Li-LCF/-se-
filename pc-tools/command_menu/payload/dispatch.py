"""User-invoked commands. Never import this module to start hardware."""
import ast
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
WORK = Path.home() / 'racecar'
TEST = Path.home() / 'racecar-tests/s-curve-test'


def run(argv, **kw):
    print('+ ' + ' '.join(map(str, argv)), flush=True)
    return subprocess.run(list(map(str, argv)), check=True, **kw)


def driver_pids():
    result = []
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = p.read_bytes().split(b'\0')[:2]
            if any(Path(os.fsdecode(a)).name in ('racecar_driver_node', 'racecar_driver_node_one') for a in args):
                result.append(p.parent.name)
        except (OSError, ValueError):
            pass
    return result


def require_idle():
    active = driver_pids()
    if active:
        raise RuntimeError('底盘仍在运行（PID ' + ', '.join(active) + '）。先停止测试后台；若是 Explorer，请在它的终端 Ctrl+C。')


def validate_reference(data):
    pts = data['points']
    if data['frame'] != 'start_local' or len(pts) < 3:
        raise ValueError('参考路径格式不正确')
    last = -1.0
    for p in pts:
        if not all(math.isfinite(p[k]) for k in ('s', 'x', 'y', 'yaw', 'curvature')) or p['s'] <= last:
            raise ValueError('参考路径含非有限数或逆序里程')
        last = p['s']


def reference_tree(source, reference):
    """Select only this process's template; leave the shared runner/files intact.

    The legacy runner has no --reference option. Replace its one template read
    in memory, preserving __file__, locks, logs, safety checks and live viewer.
    Fail before running any code if that interface has changed.
    """
    tree = ast.parse(source)
    matches = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'read_text' and isinstance(node.func.value, ast.BinOp)):
            target = node.func.value
            if (isinstance(target.op, ast.Div) and isinstance(target.left, ast.Name)
                    and target.left.id == 'root' and isinstance(target.right, ast.Constant)
                    and target.right.value == 'reference_local.json'):
                matches.append(node)
    if len(matches) != 1:
        raise RuntimeError('车端运行器的路径读取接口已改变，未发车。请更新菜单兼容适配。')
    matches[0].func.value = ast.Call(func=ast.Name(id='Path', ctx=ast.Load()),
                                    args=[ast.Constant(str(reference))], keywords=[])
    return ast.fix_missing_locations(tree)


def drive(mode):
    path = HERE / 'references' / (mode + '.json')
    data = json.loads(path.read_text())
    validate_reference(data)
    original = TEST / 'run_session.py'
    tree = reference_tree(original.read_text(), path)
    print(f'本次路径：{mode}，长度 {data["length_m"]:.2f} m，范围 {data["bounds_m"]}', flush=True)
    print('使用当前车端参数；沿用 --allow-unknown-map。任务会驱动车辆，Ctrl+C 停止。', flush=True)
    sys.path.insert(0, str(TEST))
    sys.argv = [str(original), '--allow-unknown-map']
    namespace = {'__name__': '__main__', '__file__': str(original), '__package__': None}
    exec(compile(tree, str(original), 'exec'), namespace)


def atomic_write(path, data, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
        temporary.chmod(mode)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def sync_config():
    import yaml
    source = WORK / 'src/racecar/config'
    dest = WORK / 'install/racecar/share/racecar/config'
    if not source.is_dir() or not dest.is_dir():
        raise RuntimeError('未找到 racecar 源码或安装配置目录')
    records = []
    for p in sorted(source.rglob('*')):
        if not p.is_file() or p.suffix not in ('.yaml', '.yml', '.lua'):
            continue
        data = p.read_bytes()
        if p.suffix in ('.yaml', '.yml'):
            yaml.safe_load(data)
        target = dest / p.relative_to(source)
        if target.resolve() == p.resolve():
            continue
        if not target.resolve().is_relative_to(dest.resolve()):
            raise RuntimeError('安装配置链接指向目录外，未覆盖：' + str(target))
        old = target.read_bytes() if target.exists() else None
        if old != data:
            records.append((p, target, data, old, target.stat().st_mode & 0o777 if target.exists() else 0o644))
    if not records:
        print('源码配置与安装配置一致，无需复制。运行中的参数仍以节点启动时为准。')
        return
    backup = WORK / 'config_backups' / ('command-menu-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    backup.mkdir(parents=True)
    for p, target, data, old, mode in records:
        if p.read_bytes() != data or (target.read_bytes() if target.exists() else None) != old:
            raise RuntimeError('配置在同步准备期间被修改，请重新操作')
        if old is not None:
            saved = backup / target.relative_to(dest)
            saved.parent.mkdir(parents=True, exist_ok=True)
            saved.write_bytes(old)
    applied = []
    try:
        for p, target, data, old, mode in records:
            if p.read_bytes() != data or (target.read_bytes() if target.exists() else None) != old:
                raise RuntimeError('同步时检测到并发修改：' + str(p))
            atomic_write(target, data, mode)
            applied.append((target, old, mode))
            print('已同步 ' + str(p.relative_to(source)))
    except BaseException:
        for target, old, mode in reversed(applied):
            if old is None:
                target.unlink(missing_ok=True)
            else:
                atomic_write(target, old, mode)
        raise
    print('备份：', backup)
    print('配置已保存到安装目录。没有重启节点；重新启动对应模式后生效。')


def build():
    require_idle()
    output = subprocess.check_output(['colcon', 'list', '--names-only'], cwd=WORK, text=True)
    available = set(output.split())
    packages = [p for p in ('racecar', 'racecar_driver', 'racecar_multipoint_controller', 'imu_get') if p in available]
    if not {'racecar', 'racecar_driver'} <= set(packages):
        raise RuntimeError('缺少 racecar / racecar_driver 源码包')
    run(['colcon', 'build', '--packages-up-to', *packages, '--executor', 'sequential'],
        cwd=WORK, env=dict(os.environ, CMAKE_BUILD_PARALLEL_LEVEL='2'))
    sync_config()
    print('编译完成。源码修改已安装；下一次启动模式时加载。')


def explorer():
    if (TEST / 'run_s_curve.sh').exists():
        run(['bash', TEST / 'run_s_curve.sh', '--stop-stack'])
    require_idle()
    run(['bash', WORK / 'explorer.sh'], cwd=WORK)


def keyboard():
    if not driver_pids():
        raise RuntimeError('请先在菜单启动 Explorer，再打开键盘控制。')
    print('接下来在本 MobaXterm 标签页按键驾驶；先看键盘程序显示的操作说明。', flush=True)
    run(['ros2', 'run', 'racecar', 'racecar_teleop.py'])


def save_map():
    folder = Path.home() / 'maps'
    folder.mkdir(exist_ok=True)
    base = folder / ('manual_loop_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base,
         '--ros-args', '-p', 'save_map_timeout:=10.0'])
    if not base.with_suffix('.yaml').is_file() or not base.with_suffix('.pgm').is_file():
        raise RuntimeError('保存命令结束，但未找到完整 YAML/PGM，请查看上方输出')
    print('地图已保存：', base.with_suffix('.yaml'), flush=True)
    try:
        run(['ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState',
             json.dumps({'filename': str(base.with_suffix('.pbstream')), 'include_unfinished_submaps': True})], timeout=25)
        if not base.with_suffix('.pbstream').is_file():
            print('未确认 pbstream 文件；YAML/PGM 地图已保留。')
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        print('Cartographer 状态未保存成功，YAML/PGM 地图已保留。')


def stop_child(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            p.wait(timeout=5)


def view_map():
    import yaml
    choices = []
    for folder in (Path.home() / 'maps', WORK / 'src/racecar/map'):
        for p in folder.glob('*.yaml'):
            try:
                data = yaml.safe_load(p.read_text())
                if isinstance(data, dict) and {'image', 'resolution', 'origin'} <= data.keys():
                    choices.append(p)
            except (ValueError, OSError, yaml.YAMLError):
                pass
    choices.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    if not choices:
        raise RuntimeError('~/maps 和 src/racecar/map 中还没有已保存的地图')
    for i, path in enumerate(choices, 1):
        print(f'{i}. {path}')
    value = input('输入地图编号（直接回车打开最新一张）：').strip() or '1'
    if not value.isdigit() or not 1 <= int(value) <= len(choices):
        raise ValueError('地图编号不正确')
    path = choices[int(value) - 1]
    node = 'menu_map_' + str(os.getpid())
    topic = '/' + node + '/map'
    p = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
                          '-r', '__node:=' + node, '-p', 'yaml_filename:=' + str(path), '-p', 'topic_name:=' + topic],
                         start_new_session=True)
    try:
        time.sleep(2)
        run(['ros2', 'lifecycle', 'set', '/' + node, 'configure'], timeout=25)
        run(['ros2', 'lifecycle', 'set', '/' + node, 'activate'], timeout=25)
        state = subprocess.check_output(['ros2', 'lifecycle', 'get', '/' + node], text=True, timeout=15)
        if 'active [3]' not in state:
            raise RuntimeError('地图节点未激活：' + state)
        cfg = 'Panels: []\nVisualization Manager:\n  Class: ""\n  Global Options:\n    Fixed Frame: map\n    Frame Rate: 15\n  Displays:\n    - Class: rviz_default_plugins/Map\n      Name: Saved map\n      Enabled: true\n      Topic:\n        Value: ' + topic + '\n        Durability Policy: Transient Local\n        Reliability Policy: Reliable\n      Alpha: 1\n  Views:\n    Current:\n      Class: rviz_default_plugins/TopDownOrtho\n      Scale: 50\n'
        with tempfile.TemporaryDirectory(prefix='racecar-map-view-') as tmp:
            config = Path(tmp) / 'saved_map.rviz'
            config.write_text(cfg)
            relay = Path.home() / '.local/lib/racecar-rviz-relay/rviz2'
            run([relay if relay.exists() else 'rviz2', '-d', config])
            print('地图服务保持运行；在本终端按 Ctrl+C 结束查看。', flush=True)
            while p.poll() is None:
                time.sleep(1)
    finally:
        stop_child(p)


def params():
    import yaml
    print('=== 源码中保存的底盘配置（不等于运行时参数）===')
    print((WORK / 'src/racecar/config/driver_calibration.yaml').read_text())
    print('=== 运行中的底盘参数 ===', flush=True)
    run(['ros2', 'param', 'dump', '/racecar_driver'], timeout=20)


def rviz():
    path = TEST / 's_curve_live.rviz'
    relay = Path.home() / '.local/lib/racecar-rviz-relay/rviz2'
    run([relay if relay.exists() else 'rviz2', '-d', path])


def main():
    action = sys.argv[1]
    operations = {
        'sync': sync_config, 'build': build, 's_curve': lambda: drive('s_curve'),
        'figure8': lambda: drive('figure8'), 'explorer': explorer, 'keyboard': keyboard,
        'save_map': save_map, 'view_map': view_map, 'params': params, 'rviz': rviz,
        'stop_stack': lambda: run(['bash', TEST / 'run_s_curve.sh', '--stop-stack']),
        'stop': lambda: run(['ros2', 'service', 'call', '/racecar_driver/emergency_stop', 'std_srvs/srv/Trigger', '{}'], timeout=10),
        'reset': lambda: run(['ros2', 'service', 'call', '/racecar_driver/reset_emergency_stop', 'std_srvs/srv/Trigger', '{}'], timeout=10),
    }
    # Serialize configuration/build operations invoked through this menu.
    if action in ('sync', 'build'):
        mutex = Path.home() / '.local/share/racecar-command-menu/update.lock'
        mutex.parent.mkdir(parents=True, exist_ok=True)
        with mutex.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            operations[action]()
    else:
        operations[action]()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n操作已中断。')
        raise SystemExit(130)
    except Exception as exc:
        print('\n操作失败：' + str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1)

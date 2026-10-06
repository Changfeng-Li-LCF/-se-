"""Own and reuse only the stack launched by this S-curve tool (no ROS imports)."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import threading


def process_info(pid):
    try:
        directory = Path('/proc') / str(pid)
        text = (directory / 'stat').read_text()
        fields = text[text.rindex(')') + 2:].split()
        return {'pid': int(pid), 'state': fields[0], 'ppid': int(fields[1]),
                'pgid': int(fields[2]), 'ticks': fields[19],
                'command': (directory / 'cmdline').read_bytes().split(b'\0')[:-1]}
    except (OSError, ValueError, IndexError):
        return None


def same_process(identity):
    current = process_info(identity['pid'])
    return bool(current and current['state'] != 'Z' and current['ticks'] == identity['ticks'])


def descendants(pid):
    rows = [process_info(p.name) for p in Path('/proc').iterdir() if p.name.isdigit()]
    rows = [r for r in rows if r]
    found = {pid}
    previous = set()
    while previous != found:
        previous = found.copy()
        found.update(r['pid'] for r in rows if r['ppid'] in found)
    return [r for r in rows if r['pid'] in found]


def atomic_json(path, data):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(data, indent=2))
    tmp.chmod(0o600)
    os.replace(tmp, path)


def configuration_fingerprint(workspace, saved_map=None):
    from planner_plugin_preflight import validate_planner_installation
    paths = validate_planner_installation(workspace)
    for relative in ('config', 'launch'):
        source = workspace / 'src/racecar' / relative
        installed = workspace / 'install/racecar/share/racecar' / relative
        names = ('driver_calibration.yaml', 'nav_carto.yaml', 'dynamic_turning_radius.yaml',
                 'navigate_live_replanning.xml', 'navigate_live_rolling.xml', 'navigate_through_poses_disabled.xml',
                 'navigate_marked_route.xml', 'saved_map_localization.yaml') if relative == 'config' else (
            'Run_explorer.launch.py', 'Run_saved_map.launch.py', 'navigation_follow_path.launch.py', 'navigation_no_smoother.launch.py', 'cartographer.launch.py')
        for name in names:
            a, b = source / name, installed / name
            if a.read_bytes() != b.read_bytes():
                raise RuntimeError('Source/install mismatch: ' + str(a))
            paths.extend((a, b))
    paths += sorted((workspace / 'install/racecar/share/racecar/config').rglob('*.lua'))
    paths += [workspace / 'install/racecar_driver/lib/racecar_driver/racecar_driver_node',
              workspace / 'install/racecar/lib/racecar/cartographer_odom_bridge.py']
    paths += [workspace / 'install/racecar/lib/libracecar_multipoint_controller.so',
              workspace / 'install/racecar/share/racecar_multipoint_controller/controller_plugin.xml']
    paths += [workspace / 'install/racecar/lib/racecar/stop_report_recorder.py',
              workspace / 'install/racecar/lib/racecar/map_boundary_guard.py',
              workspace / 'install/racecar/lib/racecar/speed_dependent_turning_radius.py']
    paths += sorted((workspace / 'install/racecar').rglob('planning_geometry.py'))
    paths += [workspace / 'install/racecar/lib/racecar/scan_motion_odometry.py',
              workspace / 'install/racecar/lib/libscan_motion_native.so']
    paths += sorted((workspace / 'install/racecar').rglob('scan_motion.py'))
    paths += [workspace / 'install/racecar_smac_planner/lib/libracecar_goal_preference_bt_node.so']
    if saved_map:
        import yaml
        map_path=Path(saved_map).expanduser().resolve()
        metadata=yaml.safe_load(map_path.read_text())
        image=Path(metadata['image'])
        if not image.is_absolute():image=map_path.parent/image
        paths.extend((map_path,image.resolve()))
    digest = hashlib.sha256()
    for path in paths:
        digest.update(str(path).encode()); digest.update(path.read_bytes())
    for name in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
                 'FASTRTPS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
        digest.update((name + '=' + os.environ.get(name, '')).encode())
    return digest.hexdigest()


class ManagedStack:
    def __init__(self, root, record, process=None):
        self.root, self.record, self.process = root, record, process

    def poll(self):
        if self.process is not None and self.process.poll() is not None:
            return self.process.returncode
        return None if same_process(self.record) else 1

    @property
    def log_path(self):
        return Path(self.record['log'])

    def mark_idle(self, idle):
        if idle:
            problem=self.health_reason()
            if problem:raise RuntimeError(problem)
        self.record['idle_verified'] = bool(idle)
        atomic_json(self.root / '.stack.json', self.record)

    def remember_ready_children(self):
        # Capture while all startup checks pass. A child lost before cleanup
        # must invalidate reuse, rather than disappear from a refreshed list.
        if not self.record.get('critical_processes'):
            self.record['critical_processes'] = [
                {key:row[key] for key in ('pid','ticks')}
                for row in descendants(self.record['pid']) if critical_process(row)]
        problem=self.health_reason()
        if problem:raise RuntimeError(problem)

    def health_reason(self):
        identities = self.record.get('critical_processes')
        if not identities:
            return 'reusable stack has no verified child-process identities'
        if any(not same_process(row) for row in identities):
            return 'a retained sensor, localization or navigation process exited'
        return None

    def stop(self):
        if self.poll() is None:
            # Track exact process identities before asking launch to shut down.
            owned = descendants(self.record['pid'])
            for sig, delay in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
                for row in owned:
                    if same_process(row):
                        try: os.kill(row['pid'], sig)
                        except ProcessLookupError: pass
                until = time.monotonic() + delay
                while any(same_process(row) for row in owned) and time.monotonic() < until:
                    if self.process: self.process.poll()
                    time.sleep(.05)
                if not any(same_process(row) for row in owned): break
            if any(same_process(row) for row in owned):
                raise RuntimeError('Owned stack processes did not stop; inspect stack log')
        if self.process: self.process.poll()
        registry = self.root / '.stack.json'
        if registry.exists() and json.loads(registry.read_text()).get('ticks') == self.record['ticks']:
            registry.unlink()


def load_stack(root):
    registry = root / '.stack.json'
    if not registry.exists(): return None
    record = json.loads(registry.read_text())
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if record.get('boot') != boot or not same_process(record):
        registry.unlink()
        return None
    current = process_info(record['pid'])
    # A stale or edited PID file never authorizes killing another application.
    expected_launch = b'Run_saved_map.launch.py' if record.get('saved_map') else b'Run_explorer.launch.py'
    if current['pgid'] != record['pid'] or expected_launch not in current['command']:
        raise RuntimeError('Stack ownership could not be verified; no processes changed')
    if not record.get('saved_map') and (b'follow_path_only:=true' if record.get('follow_path_only',True) else b'follow_path_only:=false') not in current['command']:
        raise RuntimeError('Recorded process is not the S-curve stack')
    if record.get('plan_and_follow_only') and b'plan_and_follow_only:=true' not in current['command']:
        raise RuntimeError('Recorded planner stack mode does not match its process')
    if record.get('saved_map') and ('map:='+record['saved_map']).encode() not in current['command']:
        raise RuntimeError('Recorded saved-map path does not match its process')
    return ManagedStack(root, record)


def start_stack(root, fingerprint, follow_path_only=True, saved_map=None):
    # Called with the session lock held, after any owned old stack has stopped.
    processes = subprocess.check_output(['ps', '-eo', 'comm='], text=True).splitlines()
    if any(p.strip() in ('racecar_driver_', 'controller_serv', 'cartographer_no',
                         'velocity_smooth') for p in processes):
        raise RuntimeError('Another vehicle stack is running outside this tool; stop it first')
    logs = root / 'stack_logs'; logs.mkdir(exist_ok=True)
    logfile = logs / ('stack-' + str(time.time_ns()) + '.log')
    command = (['ros2','launch','racecar','Run_saved_map.launch.py','map:='+str(saved_map),
                'plan_and_follow_only:=true','managed_startup:=true','no_rviz:=false','rviz_config:='+str(root/'marked_route_run.rviz')]
               if saved_map else
               ['ros2','launch','racecar','Run_explorer.launch.py',
                'navigation_autostart:=false',
                'no_rviz:='+str(follow_path_only).lower(),'follow_path_only:='+str(follow_path_only).lower()]
               + ([] if follow_path_only else ['plan_and_follow_only:=true','rviz_config:='+str(root/'live_slam_run.rviz')]))
    with logfile.open('ab') as stream:
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
            close_fds=True)
    info = process_info(process.pid)
    if info is None:
        raise RuntimeError('Stack process exited immediately; inspect ' + str(logfile))
    record = {k: info[k] for k in ('pid', 'ticks')}
    record.update(startup_schema=2,managed_startup=True,
                  boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                  fingerprint=fingerprint, log=str(logfile), idle_verified=False, follow_path_only=follow_path_only, plan_and_follow_only=not follow_path_only)
    if saved_map:record['saved_map']=str(saved_map)
    try: atomic_json(root / '.stack.json', record)
    except Exception:
        process.terminate(); process.wait(timeout=5)
        raise
    return ManagedStack(root, record, process)


def critical_process(row):
    names = {b'racecar_driver_node', b'cartographer_node', b'cartographer_odom_bridge.py', b'scan_motion_odometry.py',
             b'lslidar_driver_node', b'publisher_imu_node', b'controller_server',
             b'planner_server', b'velocity_smoother', b'bt_navigator', b'map_server',
             b'amcl', b'lifecycle_manager', b'speed_dependent_turning_radius.py'}
    return any(token.rsplit(b'/', 1)[-1] in names for token in row['command'])


def reopen_rviz(root, config):
    """One bounded relay request; desktop availability never gates navigation."""
    relay = Path.home()/'.local/lib/racecar-rviz-relay/rviz2'
    if not relay.is_file():
        raise FileNotFoundError('PC RViz relay not installed: ' + str(relay))
    # Reuse an in-flight request if commands are invoked again before PC reply.
    registry = root/'.rviz-open.json'
    if registry.exists():
        try:
            row = json.loads(registry.read_text())
            current = process_info(row['pid'])
            if (same_process(row) and current and
                    str(config).encode() in current['command']):
                return
        except (OSError, ValueError, KeyError):
            pass
    with (root/'rviz_open.log').open('ab') as stream:
        child = subprocess.Popen([str(relay), '-d', str(config)],
            stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
            close_fds=True, start_new_session=True)
    # relay.py exits after acknowledgement or its own bounded connection wait.
    threading.Thread(target=child.wait, daemon=True, name='rviz-open-reaper').start()
    info = process_info(child.pid)
    if info is not None:
        atomic_json(registry, {key:info[key] for key in ('pid','ticks')})

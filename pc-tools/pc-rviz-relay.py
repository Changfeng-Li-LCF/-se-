"""Windows login agent: SSH-only, no listening network port, no remote shell commands."""
import json
import msvcrt
import os
from pathlib import Path
import subprocess
import threading
import time
from racecar_network import resolve, ssh_options
from rviz_reuse import request_spec, MANAGED
from rviz_windows_focus import focus_titles, WindowsWindowNotReady

root = Path('D:/RacecarWork')
state = root / 'verification/rviz-relay'
state.mkdir(parents=True, exist_ok=True)
lock = (state / 'agent.lock').open('a+b')
lock.seek(0)
lock.write(b'0')
lock.flush()
lock.seek(0)
try:
    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
except OSError:
    raise SystemExit(0)
hidden = subprocess.CREATE_NO_WINDOW
ssh = str(Path(os.environ['WINDIR']) / 'System32/OpenSSH/ssh.exe')
wsl = str(Path(os.environ['WINDIR']) / 'System32/wsl.exe')
status_lock = threading.RLock()
active_rviz = None
active_request = None
active_viewers = {}

def check_dialog(pids, activate=False):
    command = [wsl, '-d', 'RacecarUbuntu2204', '--', 'python3',
               '/mnt/d/RacecarWork/tools/probe-rviz-window.py']
    for pid in pids:
        command += ['--pid', str(pid)]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding='utf-8', timeout=12, creationflags=hidden)
    if result.returncode != 0:
        raise RuntimeError('Cannot verify/activate the requested RViz window: ' + result.stderr.strip())
    snapshot = json.loads(result.stdout)
    if activate and snapshot['windows']:
        # Linux X11 focus is insufficient for WSLg's Windows RemoteApp window.
        # Also expose an owned modal dialog instead of leaving it behind the app.
        snapshot.update(focus_titles([item['title'] for item in snapshot['windows']]))
    if snapshot['dialogs']:
        raise RuntimeError('RViz 正等待弹窗确认。请处理此窗口的弹窗后重试。')
    return snapshot


def status(**data):
    with status_lock:
        data['updated_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
        temporary = state / 'status.tmp'
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(temporary, state / 'status.json')

def watch_rviz(proc, ident):
    code = proc.wait()
    with status_lock:
        data = json.loads((state / 'status.json').read_text(encoding='utf-8'))
        if data.get('windows_pid') == proc.pid:
            data.update(state='exited', exit_code=code,
                        message='RViz has exited. A new request can open it again.')
            status(**data)

def refresh_network(spec, inspect_only=False):
    command = [wsl, '-d', 'RacecarUbuntu2204', '--', 'python3',
               '/mnt/d/RacecarWork/tools/refresh-rviz-network.py', spec['kind'],
               str(spec['domain']), spec['fingerprint']]
    if inspect_only:
        command.append('--inspect-only')
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding='utf-8', timeout=20, creationflags=hidden)
    if result.returncode:
        raise RuntimeError('Cannot prepare the requested RViz instance: ' + result.stderr.strip())
    return json.loads(result.stdout)


def start_rviz(spec):
    ident = spec['id']
    args = list(spec['args'])
    directory = state / ident
    directory.mkdir(exist_ok=True)
    for index, content in spec['files'].items():
        suffix = '.rviz' if args[index-1] in ('-d', '--display-config') else '.data'
        destination = directory / (str(index) + suffix)
        destination.write_bytes(content)
        args[index] = '/mnt/d/' + destination.as_posix()[3:]
    log = (directory / 'rviz.log').open('wb')
    command = [wsl, '-d', 'RacecarUbuntu2204', '--', 'env',
               'RACECAR_ROS_DOMAIN_ID='+str(spec['domain']),
               'RACECAR_RVIZ_INSTANCE='+spec['kind'],
               'RACECAR_RVIZ_MANAGED='+MANAGED,
               'RACECAR_RVIZ_FINGERPRINT='+spec['fingerprint'],
               'RACECAR_RVIZ_REQUEST='+ident,
               'bash', '/mnt/d/RacecarWork/tools/run-rviz.sh', *args]
    proc = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                            stderr=log, creationflags=hidden)
    log.close()
    return proc


def ready_window(spec, proc=None):
    deadline = time.monotonic() + 12
    last = None
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            raise RuntimeError('Local RViz launcher exited; see ' + str(state/spec['id']/'rviz.log'))
        network = refresh_network(spec, inspect_only=True)
        if network['pids']:
            try:
                last = check_dialog(network['pids'], activate=True)
            except WindowsWindowNotReady:
                # The X11 window can exist before its Windows RemoteApp host.
                time.sleep(.2)
                continue
            if any(window.get('viewable') for window in last['windows']):
                if not last.get('windows_focus_confirmed'):
                    raise RuntimeError('RViz window exists, but bringing it to the foreground was not confirmed.')
                return network, last
        time.sleep(.3)
    raise RuntimeError('RViz process/window was not ready within 12 seconds; rendering success has not been confirmed.')


completed_requests = {}
while True:
    client = None
    heartbeat_stop = threading.Event()
    try:
        config = resolve()
        host = config['host']
        command = [ssh, '-T', *ssh_options(config),
                   '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
                   config['username']+'@'+host,
                   'python3 /home/bianbu/.local/lib/racecar-rviz-relay/relay.py --listen']
        status(connected=False, state='connecting', host=host)
        with (state / 'ssh.log').open('wb') as sshlog:
            client = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=sshlog, text=True, encoding='utf-8', creationflags=hidden)
            send_lock = threading.Lock()
            def send(payload, connection=client, mutex=send_lock):
                with mutex:
                    connection.stdin.write(json.dumps(payload)+'\n')
                    connection.stdin.flush()
            def heartbeat(connection=client, stop=heartbeat_stop, emit=send):
                while not stop.is_set():
                    try:
                        emit({'heartbeat': True})
                    except (OSError, ValueError):
                        return
                    stop.wait(5)
            threading.Thread(target=heartbeat, daemon=True).start()
            for line in client.stdout:
                request = json.loads(line)
                if request.get('ready'):
                    status(connected=True, state='waiting', host=host)
                    continue
                ident = request.get('id', '')
                if ident in completed_requests:
                    send(completed_requests[ident])
                    continue
                try:
                    spec = request_spec(request, (root/'tools/rviz-light.rviz').read_bytes())
                    kind = spec['kind']
                    active_rviz, active_request = active_viewers.get(kind, (None, None))
                    network = refresh_network(spec)
                    reused = bool(network['pids'])
                    replaced = network['refreshed']
                    if not reused:
                        proc = start_rviz(spec)
                        active_rviz, active_request = proc, ident
                        active_viewers[kind] = (proc, ident)
                    else:
                        proc = None
                    network, window = ready_window(spec, proc)
                    status(connected=True, state='window_ready', request=ident,
                           windows_pid=active_rviz.pid if active_rviz and active_rviz.poll() is None else None,
                           linux_pids=network['pids'], bind_ip=network['ip'],
                           viewer_kind=kind, fingerprint=spec['fingerprint'], reused=reused,
                           replaced_pids=replaced, window=window,
                           message='Requested config matched; Windows foreground confirmed. ROS data and rendering health are not verified.')
                    if proc is not None:
                        threading.Thread(target=watch_rviz, args=(proc, ident), daemon=True).start()
                    ack = {'id': ident, 'ok': True, 'state': 'window_ready',
                           'reused': reused, 'viewer_kind': kind,
                           'fingerprint': spec['fingerprint'],
                           'windows_focus_confirmed': window['windows_focus_confirmed'],
                           'rendering_verified': False}
                except Exception as error:
                    ack = {'id': ident, 'ok': False, 'error': str(error)}
                    if active_rviz is not None and active_rviz.poll() is None:
                        status(connected=True, state='attention_required', error=str(error),
                               request=active_request, windows_pid=active_rviz.pid)
                    else:
                        status(connected=True, state='launch_error', error=str(error))
                completed_requests[ident] = ack
                if len(completed_requests) > 256:
                    del completed_requests[next(iter(completed_requests))]
                send(ack)
            raise RuntimeError('SSH connection ended')
    except Exception as error:
        heartbeat_stop.set()
        status(connected=False, state='reconnecting', error=str(error))
        if client is not None:
            client.terminate()
            try:
                client.wait(timeout=5)
            except subprocess.TimeoutExpired:
                client.kill()
        time.sleep(10)

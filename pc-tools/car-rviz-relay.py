#!/usr/bin/env python3
"""Forward RViz launch requests over an authenticated SSH stdio connection."""
import base64
import fcntl
import json
import os
from pathlib import Path
import re
import select
import stat
import sys
import threading
import time
import uuid

state = Path.home() / '.local/state/racecar-rviz-relay'
state.mkdir(parents=True, exist_ok=True, mode=0o700)
fifo = state / 'requests.fifo'
valid_id = re.compile(r'^[a-f0-9]{32}$')

def listen():
    lock = (state / 'listener.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if not fifo.exists():
        os.mkfifo(fifo, 0o600)
    if not stat.S_ISFIFO(fifo.stat().st_mode):
        raise RuntimeError('Invalid request FIFO')
    fd = os.open(fifo, os.O_RDWR)
    def acknowledgements():
        for line in sys.stdin:
            try:
                ack = json.loads(line)
                if valid_id.fullmatch(ack.get('id', '')):
                    (state / (ack['id'] + '.ack')).write_text(json.dumps(ack), encoding='utf-8')
            except (ValueError, OSError):
                pass
        os._exit(0)
    threading.Thread(target=acknowledgements, daemon=True).start()
    print(json.dumps({'ready': True}), flush=True)
    with os.fdopen(fd, 'r') as pipe:
        for line in pipe:
            ident = line.strip()
            if not valid_id.fullmatch(ident):
                continue
            path = state / (ident + '.json')
            try:
                if path.stat().st_size > 8_000_000:
                    raise ValueError('Request too large')
                payload = json.loads(path.read_text(encoding='utf-8'))
                print(json.dumps(payload), flush=True)
            finally:
                path.unlink(missing_ok=True)

def request():
    ident = uuid.uuid4().hex
    args = sys.argv[1:]
    files = {}
    for index, arg in enumerate(args[:-1]):
        if arg in ('-d', '--display-config', '--params-file', '-s', '--splash-screen'):
            path = Path(args[index+1]).expanduser()
            if not path.is_file() or path.stat().st_size > 2_000_000:
                raise RuntimeError('Cannot transfer RViz file: ' + str(path))
            files[str(index+1)] = {'name': path.name, 'data': base64.b64encode(path.read_bytes()).decode()}
    payload = {'id': ident, 'args': args, 'files': files, 'domain': os.environ.get('ROS_DOMAIN_ID', '0')}
    path = state / (ident + '.json')
    path.write_text(json.dumps(payload), encoding='utf-8')
    try:
        fd = os.open(fifo, os.O_WRONLY | os.O_NONBLOCK)
        try:
            os.write(fd, (ident + '\n').encode())
        finally:
            os.close(fd)
    except OSError:
        path.unlink(missing_ok=True)
        raise RuntimeError('电脑 RViz 转接服务未连接，请先启动电脑“比赛工具”中的 08_RViz_Relay；不会回退到小车渲染。')
    ack_path = state / (ident + '.ack')
    deadline = time.monotonic()+45
    while time.monotonic() < deadline:
        if ack_path.exists():
            try:
                ack = json.loads(ack_path.read_text())
            except ValueError:
                time.sleep(0.1)
                continue
            ack_path.unlink(missing_ok=True)
            if ack.get('ok'):
                print('RViz 已交给电脑运行，配置文件已同步；小车不启动图形渲染。')
                return
            raise RuntimeError(ack.get('error', 'PC launch failed'))
        time.sleep(0.25)
    path.unlink(missing_ok=True)
    raise RuntimeError('电脑未确认 RViz 启动，请检查电脑转接服务。')

try:
    listen() if sys.argv[1:] == ['--listen'] else request()
except Exception as error:
    print(str(error), file=sys.stderr)
    sys.exit(1)

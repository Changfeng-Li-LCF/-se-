#!/usr/bin/env python3
"""Refresh only the passive display after network changes; never drives the car."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def start(root):
    with (root / 'rviz_open.log').open('a') as log:
        subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker'],
                         stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                         start_new_session=True, close_fds=True)


def identity(proc):
    return (proc / 'stat').read_text().rsplit(')', 1)[1].split()[19]


def worker(root):
    with (root / '.live_view_start.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        target = str(root / 'live_view.py')
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                args = (proc / 'cmdline').read_bytes().split(b'\0')
                if len(args) < 2 or args[1].decode() != target:
                    continue
                if proc.stat().st_uid != os.getuid():
                    continue
                birth = identity(proc)
                if identity(proc) == birth:
                    os.kill(int(proc.name), signal.SIGTERM)
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and proc.exists():
                    if identity(proc) != birth:
                        break
                    time.sleep(0.05)
            except (OSError, IndexError, UnicodeError):
                continue
        with (root / 'live_view.log').open('a') as log:
            subprocess.Popen([sys.executable, target], stdin=subprocess.DEVNULL,
                             stdout=log, stderr=log, start_new_session=True, close_fds=True)
    result = subprocess.run([str(Path.home() / '.local/lib/racecar-rviz-relay/rviz2'),
                             '-d', str(root / 's_curve_live.rviz')], stdin=subprocess.DEVNULL)
    return result.returncode


if __name__ == '__main__':
    root = Path(__file__).resolve().parent
    if sys.argv[1:] == ['--worker']:
        raise SystemExit(worker(root))
    start(root)

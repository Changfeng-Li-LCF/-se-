"""Explicit, user-invoked local RViz recovery. No car nodes are stopped."""
import json
import os
from pathlib import Path
import subprocess
import sys
from racecar_network import resolve, ssh_options

root = Path('D:/RacecarWork')
config = resolve()
hidden = subprocess.CREATE_NO_WINDOW
with (root/'verification/rviz-relay/recovery.log').open('w', encoding='utf-8') as log:
    reset_display = '--reset-display' in sys.argv[1:]
    if reset_display:
        listed = subprocess.run(['wsl', '--list', '--running', '--quiet'],
                                capture_output=True, check=True, timeout=10, creationflags=hidden).stdout
        names = listed.decode('utf-16-le' if b'\0' in listed else 'utf-8').lstrip('\ufeff')
        others = [line.strip() for line in names.splitlines()
                  if line.strip() and line.strip() != 'RacecarUbuntu2204']
        if others:
            raise RuntimeError('Other WSL distributions are running; display reset stopped: '+', '.join(others))
    subprocess.run(['wsl', '-d', 'RacecarUbuntu2204', '--', 'python3',
                    '/mnt/d/RacecarWork/tools/stop-local-rviz.py'],
                   stdout=log, stderr=log, check=True, timeout=20, creationflags=hidden)
    if reset_display:
        subprocess.run(['wsl', '--shutdown'], stdout=log, stderr=log,
                       check=True, timeout=30, creationflags=hidden)
    subprocess.run([str(Path(os.environ['WINDIR'])/'System32/OpenSSH/ssh.exe'),
                    *ssh_options(config), config['username']+'@'+config['host'],
                    '/home/bianbu/.local/lib/racecar-rviz-relay/rviz2 -d '
                    '/home/bianbu/racecar/install/racecar/share/racecar/rviz/racecar_cartographer.rviz'],
                   stdout=log, stderr=log, check=True, timeout=60, creationflags=hidden)

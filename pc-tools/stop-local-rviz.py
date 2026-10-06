"""Recovery helper: stop only this WSL distro's RViz after backing up its saved config."""
from pathlib import Path
import json
import os
import shutil
import signal
import subprocess
import time

assert os.environ.get('WSL_DISTRO_NAME') == 'RacecarUbuntu2204'
backup = Path('/mnt/d/RacecarWork/backups') / ('rviz-recovery-' + time.strftime('%Y%m%d-%H%M%S'))
backup.mkdir(parents=True, exist_ok=True)
for value in subprocess.run(['pgrep', '-x', 'rviz2'], capture_output=True, text=True).stdout.split():
    pid = int(value)
    proc = Path('/proc') / value
    try:
        birth = (proc/'stat').read_text().split()[21]
        args = (proc/'cmdline').read_bytes().decode().strip('\0').split('\0')
        (backup/(value+'-args.json')).write_text(json.dumps(args))
        for i, arg in enumerate(args[:-1]):
            if arg in ('-d', '--display-config') and Path(args[i+1]).is_file():
                shutil.copy2(args[i+1], backup/(value+'-display.rviz'))
        os.kill(pid, signal.SIGINT)
        deadline = time.monotonic()+4
        while proc.exists() and time.monotonic()<deadline:
            time.sleep(.2)
        if proc.exists() and (proc/'comm').read_text().strip()=='rviz2' and (proc/'stat').read_text().split()[21]==birth:
            os.kill(pid, signal.SIGKILL)
    except (ProcessLookupError, FileNotFoundError):
        pass
print('Stopped local RViz only. Saved config backup:', backup)

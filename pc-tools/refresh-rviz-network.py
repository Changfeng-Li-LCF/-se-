"""Refresh a verified relay-owned viewer when its config, network or domain changes."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from rviz_reuse import KINDS, can_reuse, managed_kind

EXECUTABLE = '/opt/racecar-rviz-original/lib/rviz2/rviz2'


def snapshot(pid):
    try:
        root = Path('/proc') / str(pid)
        if os.readlink(root / 'exe') != EXECUTABLE:
            return None
        env = dict(entry.split('=', 1) for entry in
                   (root / 'environ').read_text().split('\0') if '=' in entry)
        argv = (root / 'cmdline').read_text().strip('\0').split('\0')
        kind = managed_kind(env, argv, lambda path: Path(path).read_bytes())
        if kind is None:
            return None
        birth = (root / 'stat').read_text().rsplit(')', 1)[1].split()[19]
        return {'pid': pid, 'birth': birth, 'env': env, 'kind': kind}
    except (OSError, ValueError, IndexError):
        return None


def same_process(record):
    current = snapshot(record['pid'])
    return bool(current and current['birth'] == record['birth'])


def window_probe(pids):
    spec = importlib.util.spec_from_file_location('rviz_window', Path(__file__).with_name('probe-rviz-window.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.probe(pids)


def retire(record):
    pid = record['pid']
    if window_probe([pid])['dialogs']:
        raise RuntimeError('Managed RViz has an open dialog; close or cancel it before retrying. PID='+str(pid))
    if same_process(record):
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline and same_process(record):
        time.sleep(.1)
    if same_process(record):
        if window_probe([pid])['dialogs']:
            raise RuntimeError('RViz is waiting for Save/Discard confirmation; finish the dialog and retry.')
        if same_process(record):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and same_process(record):
            time.sleep(.1)
        if same_process(record):
            raise RuntimeError('Managed RViz has not released its instance lock')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('kind', choices=KINDS)
    parser.add_argument('domain', type=int)
    parser.add_argument('fingerprint')
    parser.add_argument('--inspect-only', action='store_true')
    args = parser.parse_args()
    if os.environ.get('WSL_DISTRO_NAME') != 'RacecarUbuntu2204':
        raise SystemExit('This helper only runs in the local RViz distro.')
    if not 0 <= args.domain <= 232 or len(args.fingerprint) != 64 or any(c not in '0123456789abcdef' for c in args.fingerprint):
        raise SystemExit('Invalid ROS domain or configuration fingerprint')
    selected = subprocess.check_output(
        ['python3', '/mnt/d/RacecarWork/tools/configure-fastdds.py', '--print-ip'], text=True).strip()
    records = [snapshot(int(p.name)) for p in Path('/proc').iterdir() if p.name.isdigit()]
    records = [r for r in records if r and r['kind'] == args.kind]
    kept, refreshed, replacements = [], [], []
    for record in records:
        if can_reuse(record, args.kind, args.domain, args.fingerprint, selected):
            kept.append(record['pid'])
        else:
            replacements.append(record['pid'])
            if not args.inspect_only:
                retire(record)
                refreshed.append(record['pid'])
    print(json.dumps({'ip': selected, 'pids': kept, 'refreshed': refreshed,
                      'replacement_pids': replacements, 'inspect_only': args.inspect_only}))


if __name__ == '__main__':
    main()

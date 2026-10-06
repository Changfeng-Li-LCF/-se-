#!/usr/bin/env python3
"""Route both packaged RViz executables through one launcher, with dpkg diversion.

Run as root in the specified car or PC WSL. Never starts RViz or a car node.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

p = argparse.ArgumentParser()
p.add_argument('mode', choices=['car', 'pc'])
args = p.parse_args()
if os.geteuid() != 0:
    raise SystemExit('Root is required for package executable diversion')
if args.mode == 'pc' and os.environ.get('WSL_DISTRO_NAME') != 'RacecarUbuntu2204':
    raise SystemExit('PC entrypoints must be installed only in RacecarUbuntu2204')
if args.mode == 'car' and not Path('/home/bianbu/.local/lib/racecar-rviz-relay/relay.py').is_file():
    raise SystemExit('Existing car relay is required')
paths = [Path('/opt/ros/humble/bin/rviz2'), Path('/opt/ros/humble/lib/rviz2/rviz2')]
backup = Path('/var/backups/racecar-rviz-entrypoints') / datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
backup.mkdir(parents=True)
manifest = {'mode': args.mode, 'backup': str(backup), 'entries': []}
marker = '# RACECAR UNIFIED RVIZ ENTRYPOINT v1'
if args.mode == 'pc':
    body = '''#!/usr/bin/env bash
# RACECAR UNIFIED RVIZ ENTRYPOINT v1
exec bash /mnt/d/RacecarWork/tools/run-rviz.sh "$@"
'''
else:
    body = '''#!/usr/bin/env bash
# RACECAR UNIFIED RVIZ ENTRYPOINT v1
if [[ "${RACECAR_RVIZ_ROUTE_CHECK:-}" == 1 ]]; then
  printf '%s\\n' 'RACECAR_ROUTE=car-to-pc-relay'
  exit 0
fi
if [[ "$(id -u)" == 0 ]]; then
  exec runuser -u bianbu -- /home/bianbu/.local/lib/racecar-rviz-relay/rviz2 "$@"
fi
if [[ "$(id -un)" != bianbu ]]; then
  echo 'Use the configured bianbu account for the PC RViz relay.' >&2
  exit 1
fi
exec /home/bianbu/.local/lib/racecar-rviz-relay/rviz2 "$@"
'''
subprocess.run(['bash', '-n'], input=body.encode(), check=True)
# Inspect and back up every entry before the first replacement.
for original in paths:
    diverted = Path('/opt/racecar-rviz-original') / original.relative_to('/opt/ros/humble')
    true_name = subprocess.check_output(['dpkg-divert', '--truename', str(original)], text=True).strip()
    if true_name not in (str(original), str(diverted)):
        raise SystemExit('Foreign diversion: ' + true_name)
    if not original.is_file():
        raise SystemExit('RViz executable missing: ' + str(original))
    saved = backup / original.relative_to('/')
    saved.parent.mkdir(parents=True)
    shutil.copy2(original, saved)
    existing = original.read_bytes()
    if true_name == str(diverted) and marker.encode() not in existing:
        raise SystemExit('Existing diversion entry is not our wrapper: ' + str(original))
    if true_name == str(original) and not existing.startswith(b'\x7fELF'):
        raise SystemExit('Unexpected original executable: ' + str(original))
    manifest['entries'].append({'path': str(original), 'diverted': str(diverted),
                                'before_sha256': hashlib.sha256(existing).hexdigest(),
                                'already_diverted': true_name == str(diverted)})
(backup/'manifest.json').write_text(json.dumps(manifest, indent=2))
for item in manifest['entries']:
    original, diverted = Path(item['path']), Path(item['diverted'])
    diverted.parent.mkdir(parents=True, exist_ok=True)
    if not item['already_diverted']:
        subprocess.run(['dpkg-divert','--local','--add','--rename','--divert',str(diverted),str(original)], check=True)
    if not diverted.read_bytes().startswith(b'\x7fELF'):
        raise SystemExit('Saved executable is not ELF: ' + str(diverted))
    temp = original.with_name(original.name+'.racecar-tmp')
    temp.write_text(body)
    temp.chmod(0o755)
    os.replace(temp, original)
    item['wrapper_sha256'] = hashlib.sha256(original.read_bytes()).hexdigest()
(backup/'manifest.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps(manifest, indent=2))

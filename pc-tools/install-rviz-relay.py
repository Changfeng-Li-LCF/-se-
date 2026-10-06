"""Retain RViz launch actions but route their executables to the PC relay."""
import ast
import datetime
import json
from pathlib import Path
import shutil
import subprocess

home = Path.home()
root = home / 'racecar'
relay_dir = home / '.local/lib/racecar-rviz-relay'
proxy = relay_dir / 'rviz2'
assert home.name == 'bianbu' and (relay_dir / 'relay.py').is_file()
changes = {}
changed_nodes = 0
seen = set()
for folder in (root / 'src', root / 'install'):
    # ROS also uses *_launch.py (e.g. the LS lidar viewers).
    for path in folder.rglob('*.py'):
        path = path.resolve()
        if path in seen or not path.is_relative_to(root):
            continue
        seen.add(path)
        text = path.read_text(encoding='utf-8')
        if 'rviz2' not in text:
            continue
        tree = ast.parse(text)
        edits = []
        data = text.encode('utf-8')
        lines = data.splitlines(keepends=True)
        offsets = [0]
        for line in lines:
            offsets.append(offsets[-1]+len(line))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not any(k.arg == 'package' and isinstance(k.value, ast.Constant) and k.value.value == 'rviz2' for k in node.keywords):
                continue
            executable = next((k.value for k in node.keywords if k.arg == 'executable'), None)
            assert isinstance(executable, ast.Constant)
            if executable.value == str(proxy):
                continue
            assert executable.value == 'rviz2', str(path)
            first = offsets[executable.lineno-1]+executable.col_offset
            last = offsets[executable.end_lineno-1]+executable.end_col_offset
            edits.append((first, last, repr(str(proxy)).encode()))
            # Replacing only this literal preserves existing launch conditions and arguments.
            executable.value = str(proxy)
        for first, last, replacement in sorted(edits, reverse=True):
            data = data[:first]+replacement+data[last:]
        if edits:
            parsed = ast.parse(data)
            assert ast.dump(tree, include_attributes=False) == ast.dump(parsed, include_attributes=False)
            compile(parsed, str(path), 'exec')
            changes[path] = data
            changed_nodes += len(edits)

changes[proxy] = b'''#!/usr/bin/env bash
if [[ "${RACECAR_RVIZ_ROUTE_CHECK:-}" == 1 ]]; then
  printf '%s\\n' 'RACECAR_ROUTE=car-to-pc-relay'
  exit 0
fi
exec python3 "$HOME/.local/lib/racecar-rviz-relay/relay.py" "$@"
'''
docker = root / 'rviz_docker.sh'
changes[docker] = b'''#!/usr/bin/env bash
# Forward this viewer to the PC instead of rendering in a car-side container.
config="$HOME/racecar/rviz_remote.rviz"
if [ -f "$config" ]; then
  exec "$HOME/.local/lib/racecar-rviz-relay/rviz2" -d "$config" "$@"
fi
exec "$HOME/.local/lib/racecar-rviz-relay/rviz2" "$@"
'''
rc = home / '.bashrc'
text = rc.read_text(encoding='utf-8')
if '# BEGIN RACECAR RVIZ RELAY' not in text:
    text += '''
# BEGIN RACECAR RVIZ RELAY
rviz2() {
    "$HOME/.local/lib/racecar-rviz-relay/rviz2" "$@"
}
ros2() {
    if [ "${1:-} ${2:-} ${3:-}" = 'run rviz2 rviz2' ]; then
        shift 3
        rviz2 "$@"
    else
        command ros2 "$@"
    fi
}
# END RACECAR RVIZ RELAY
'''
    changes[rc] = text.encode('utf-8')
for path, content in changes.items():
    if path.name in ('rviz2', '.bashrc') or path.suffix == '.sh':
        subprocess.run(['bash', '-n'], input=content, check=True)
backup = home / '.local/state/racecar-rviz-relay' / ('before-install-' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
backup.mkdir(parents=True, exist_ok=False)
manifest = {'backup': str(backup), 'rviz_nodes_routed': changed_nodes, 'files': []}
for path, content in changes.items():
    relative = path.relative_to(home)
    destination = backup / relative
    if path.exists():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    manifest['files'].append(str(relative))
proxy.chmod(0o755)
(backup / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
print(json.dumps(manifest, indent=2))

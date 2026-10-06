"""Make the inspected racecar workspace use PC-only RViz; back up every edit."""
import ast
import copy
import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

home = Path.home()
root = home / 'racecar'
if home.name != 'bianbu' or not (root / 'install/setup.bash').is_file():
    raise SystemExit('Run this installer as bianbu on the car with ~/racecar installed.')
helper = '_RacecarPCOnlyCondition'
condition = helper + "('false')"
changes = {}

def rviz_call(node):
    return isinstance(node, ast.Call) and any(
        kw.arg in ('package', 'executable') and isinstance(kw.value, ast.Constant)
        and kw.value.value == 'rviz2' for kw in node.keywords)

def strip_rviz_conditions(tree):
    tree = copy.deepcopy(tree)
    tree.body = [node for node in tree.body if not (
        isinstance(node, ast.ImportFrom) and node.module == 'launch.conditions'
        and any(alias.asname == helper for alias in node.names))]
    for node in ast.walk(tree):
        if rviz_call(node):
            node.keywords = [kw for kw in node.keywords if kw.arg != 'condition']
    return ast.dump(tree, include_attributes=False)

def patch_launch(text, filename):
    tree = ast.parse(text, filename=filename)
    data = text.encode('utf-8')
    lines = data.splitlines(keepends=True)
    offsets = [0]
    for line in lines:
        offsets.append(offsets[-1] + len(line))
    def pos(node, end=False):
        return offsets[(node.end_lineno if end else node.lineno)-1] + (node.end_col_offset if end else node.col_offset)
    edits = []
    for node in ast.walk(tree):
        if not rviz_call(node):
            continue
        old = next((kw.value for kw in node.keywords if kw.arg == 'condition'), None)
        if old is not None:
            if ast.get_source_segment(text, old) == condition:
                continue
            edits.append((pos(old), pos(old, True), condition.encode()))
        else:
            opening = data.index(b'(', pos(node.func, True)) + 1
            edits.append((opening, opening, ('condition=' + condition + ', ').encode()))
    if not edits:
        return text
    if not any(isinstance(node, ast.ImportFrom) and node.module == 'launch.conditions'
               and any(alias.asname == helper for alias in node.names) for node in tree.body):
        insert_at = 0
        for index, node in enumerate(tree.body):
            if (index == 0 and isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)
                    and isinstance(node.value.value, str)) or (isinstance(node, ast.ImportFrom) and node.module == '__future__'):
                insert_at = offsets[node.end_lineno]
            else:
                break
        edits.append((insert_at, insert_at, ('# RViz rendering is assigned to the Windows PC.\nfrom launch.conditions import IfCondition as ' + helper + '\n').encode()))
    for first, last, replacement in sorted(edits, reverse=True):
        data = data[:first] + replacement + data[last:]
    result = data.decode('utf-8')
    new_tree = ast.parse(result, filename=filename)
    assert strip_rviz_conditions(tree) == strip_rviz_conditions(new_tree), filename
    compile(new_tree, filename, 'exec')
    return result

seen = set()
for folder in (root / 'src', root / 'install'):
    for path in folder.rglob('*.launch.py'):
        actual = path.resolve()
        if not actual.is_relative_to(root.resolve()):
            continue
        if actual in seen:
            continue
        seen.add(actual)
        old = path.read_text(encoding='utf-8')
        if 'rviz2' not in old:
            continue
        new = patch_launch(old, str(path))
        if new != old:
            changes[actual] = new

source_block = '''# BEGIN RACECAR PC RVIZ ENV
RACECAR_WORKSPACE="$HOME/racecar"
source /opt/ros/humble/setup.bash || exit 1
source "$RACECAR_WORKSPACE/install/setup.bash" || exit 1
# END RACECAR PC RVIZ ENV
'''
for name in ('car.sh', 'gmapping.sh', 'nav.sh', 'nav_one.sh', 'explorer.sh', 'explorer_multi.sh'):
    path = root / name
    if not path.exists():
        continue
    old = path.read_text(encoding='utf-8')
    new = old
    if 'BEGIN RACECAR PC RVIZ ENV' not in new:
        at = new.index('\n') + 1 if new.startswith('#!') else 0
        new = new[:at] + source_block + new[at:]
    if name.startswith('explorer'):
        new = re.sub(r'(?m)^NO_RVIZ=false\s*$', 'NO_RVIZ=true', new)
        new = new.replace('正常启动（含 rviz2）', '正常启动（RViz 在电脑运行）')
    if new != old:
        subprocess.run(['bash', '-n'], input=new, text=True, check=True)
        changes[path] = new

docker = root / 'rviz_docker.sh'
if docker.exists():
    replacement = '''#!/usr/bin/env bash
# RViz rendering is permanently assigned to the Windows PC.
echo 'RViz runs on the Windows PC. Open the desktop shortcut: RViz（电脑运行）.'
echo 'The car continues publishing sensor and mapping data; no RViz container is started here.'
'''
    if docker.read_text(encoding='utf-8') != replacement:
        changes[docker] = replacement

rc = home / '.bashrc'
old = rc.read_text(encoding='utf-8')
if '# BEGIN RACECAR PC RVIZ DEFAULT' not in old:
    changes[rc] = old + '''
# BEGIN RACECAR PC RVIZ DEFAULT
# New SSH terminals find the installed workspace without a manual source command.
if [ -f "$HOME/racecar/install/setup.bash" ]; then
    source "$HOME/racecar/install/setup.bash"
fi
rviz2() {
    printf '%s\\n' 'RViz 已固定由电脑运行，请双击电脑桌面的“RViz（电脑运行）”。'
}
ros2() {
    if [ "${1:-} ${2:-} ${3:-}" = 'run rviz2 rviz2' ]; then
        rviz2
    else
        command ros2 "$@"
    fi
}
# END RACECAR PC RVIZ DEFAULT
'''
    subprocess.run(['bash', '-n'], input=changes[rc], text=True, check=True)

if '--apply' not in sys.argv:
    print(json.dumps({'planned_files': [str(p.relative_to(home)) for p in changes]}, indent=2))
    raise SystemExit(0)

backup = home / '.local/state/racecar-pc-rviz' / datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
backup.mkdir(parents=True, exist_ok=False)
for path, new in changes.items():
    if not path.resolve().is_relative_to(home.resolve()):
        raise SystemExit('Refusing path outside car user home: ' + str(path))
    target = backup / path.relative_to(home)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, target)
    path.write_text(new, encoding='utf-8')
manifest = {'backup': str(backup), 'changed_files': [str(path.relative_to(home)) for path in changes]}
(backup / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
print(json.dumps(manifest, indent=2))

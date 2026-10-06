"""Restore original car RViz behavior, retaining workspace environment loading."""
import ast
import copy
import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess

home = Path.home()
original = home / '.local/state/racecar-pc-rviz/20260924T182140Z'
manifest = json.loads((original / 'manifest.json').read_text())
changes = {}

def normalize(text):
    tree = ast.parse(text)
    tree.body = [n for n in tree.body if not (isinstance(n, ast.ImportFrom)
        and n.module == 'launch.conditions' and any(a.asname == '_RacecarPCOnlyCondition' for a in n.names))]
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and any(k.arg in ('package', 'executable')
                and isinstance(k.value, ast.Constant) and k.value.value == 'rviz2' for k in n.keywords):
            n.keywords = [k for k in n.keywords if k.arg != 'condition']
    return ast.dump(tree, include_attributes=False)

env_block = '''# BEGIN RACECAR PC RVIZ ENV
RACECAR_WORKSPACE="$HOME/racecar"
source /opt/ros/humble/setup.bash || exit 1
source "$RACECAR_WORKSPACE/install/setup.bash" || exit 1
# END RACECAR PC RVIZ ENV
'''
for relative in manifest['changed_files']:
    path = home / relative
    old_bytes = (original / relative).read_bytes()
    old = old_bytes.decode('utf-8').replace('\r\n', '\n')
    current = path.read_text(encoding='utf-8')
    if relative.endswith('.launch.py'):
        assert normalize(old) == normalize(current), 'Unrelated change detected: ' + relative
        changes[path] = old_bytes
    elif relative.endswith('rviz_docker.sh'):
        assert 'RViz rendering is permanently assigned' in current
        changes[path] = old_bytes
    elif relative.endswith(('explorer.sh', 'explorer_multi.sh')):
        at = old.index('\n') + 1 if old.startswith('#!') else 0
        restored = old[:at] + env_block + old[at:]
        expected = re.sub(r'(?m)^NO_RVIZ=false\s*$', 'NO_RVIZ=true', restored)
        expected = expected.replace('正常启动（含 rviz2）', '正常启动（RViz 在电脑运行）')
        assert current == expected, 'Unrelated change detected: ' + relative
        changes[path] = restored.encode('utf-8')
    elif relative == '.bashrc':
        marker = '# BEGIN RACECAR PC RVIZ DEFAULT'
        assert marker in current and '# END RACECAR PC RVIZ DEFAULT' in current
        block = '''# BEGIN RACECAR WORKSPACE ENV
if [ -f "$HOME/racecar/install/setup.bash" ]; then
    source "$HOME/racecar/install/setup.bash"
fi
# END RACECAR WORKSPACE ENV'''
        restored = re.sub(r'# BEGIN RACECAR PC RVIZ DEFAULT.*?# END RACECAR PC RVIZ DEFAULT', block, current, flags=re.S)
        changes[path] = restored.encode('utf-8')

for path, data in changes.items():
    if path.name.endswith('.sh') or path.name == '.bashrc':
        subprocess.run(['bash', '-n'], input=data, check=True)
    else:
        compile(data, str(path), 'exec')

backup = home / '.local/state/racecar-pc-rviz' / ('before-restore-' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
backup.mkdir(parents=True, exist_ok=False)
for path, data in changes.items():
    relative = path.relative_to(home)
    copy_to = backup / relative
    copy_to.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, copy_to)
    path.write_bytes(data)
for relative in manifest['changed_files']:
    if relative.endswith('.launch.py') or relative.endswith('rviz_docker.sh'):
        assert (home / relative).read_bytes() == (original / relative).read_bytes()
report = {'restored_files': [str(p.relative_to(home)) for p in changes],
          'rollback_backup': str(backup), 'workspace_environment_fixes_retained': True}
(backup / 'restore-result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report, indent=2))

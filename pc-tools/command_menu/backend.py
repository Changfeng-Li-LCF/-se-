"""Transport for the local menu. No connections or commands on import."""
import base64
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent
TOOLS = ROOT.parent
CONFIG = TOOLS / 'racecar-connection.json'
HIDDEN = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
PAYLOAD_NAMES = ('dispatch.py', 'remote.sh', 'references/s_curve.json', 'references/figure8.json')


def read_connection():
    return json.loads(CONFIG.read_text(encoding='utf-8-sig'))


def resolve_connection():
    spec = importlib.util.spec_from_file_location('menu_racecar_network', TOOLS / 'racecar_network.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.resolve()


def payload_files():
    # Normalize scripts to LF, regardless of the Windows editor's newline mode.
    return {n: (ROOT / 'payload' / n).read_bytes().replace(b'\r\n', b'\n') for n in PAYLOAD_NAMES}


def payload_version(files=None):
    files = payload_files() if files is None else files
    digest = hashlib.sha256()
    for name in sorted(files):
        digest.update(name.encode() + b'\0' + files[name] + b'\0')
    return digest.hexdigest()[:20]


def remote_command(action, version=None):
    from catalog import BY_KEY
    if action not in BY_KEY:
        raise ValueError('未知菜单操作')
    version = payload_version() if version is None else version
    if not re.fullmatch('[0-9a-f]{20}', version):
        raise ValueError('工具版本不合法')
    # Quoted $HOME expands only on the car, not on Windows or MobaXterm.
    return f'bash "$HOME/.local/share/racecar-command-menu/versions/{version}/remote.sh" {shlex.quote(action)}'


def windows_ssh():
    win = Path(os.environ.get('WINDIR', 'C:/Windows'))
    for folder in ('Sysnative', 'System32'):
        path = win / folder / 'OpenSSH/ssh.exe'
        if path.is_file():
            return str(path)
    return 'ssh'


def ssh_args(config):
    return [windows_ssh(), '-p', str(config.get('port', 22)), '-i', config['ssh_identity_file'],
            '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'HostKeyAlias=' + config['verified_host_alias'],
            '-o', 'UserKnownHostsFile="' + config['verified_known_hosts_file'] + '"',
            '-o', 'ConnectTimeout=6', '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
            config['username'] + '@' + config['host']]


# This installs only the menu's own versioned files. It never imports dispatch.py,
# starts ROS nodes, touches the car's code/configuration, or resets any stop.
INSTALLER = r'''
import base64,hashlib,json,os,pathlib,shutil,sys,tempfile
request=json.load(sys.stdin)
version=request['version']
assert len(version)==20 and all(c in '0123456789abcdef' for c in version)
files={n:base64.b64decode(v,validate=True) for n,v in request['files'].items()}
expected={'dispatch.py','remote.sh','references/s_curve.json','references/figure8.json'}
assert set(files)==expected
h=hashlib.sha256()
for name in sorted(files):h.update(name.encode()+b'\0'+files[name]+b'\0')
assert h.hexdigest()[:20]==version
root=pathlib.Path.home()/'.local/share/racecar-command-menu/versions'
root.mkdir(parents=True,exist_ok=True)
dest=root/version
if dest.exists():
 assert all((dest/n).read_bytes()==v for n,v in files.items()),'Existing menu version differs'
else:
 stage=pathlib.Path(tempfile.mkdtemp(prefix='.install-',dir=root))
 try:
  for name,data in files.items():
   p=stage/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data);p.chmod(0o644)
  try:stage.rename(dest)
  except OSError:
   assert dest.is_dir() and all((dest/n).read_bytes()==v for n,v in files.items())
 finally:
  if stage.exists():shutil.rmtree(stage)
print(json.dumps({'installed':str(dest),'version':version,'vehicle_commands_sent':False}))
'''


def install_payload(config):
    files = payload_files()
    version = payload_version(files)
    request = json.dumps({'version': version, 'files': {n: base64.b64encode(v).decode() for n, v in files.items()}})
    result = subprocess.run([*ssh_args(config), 'python3 -c ' + shlex.quote(INSTALLER)],
                            input=request.encode(), capture_output=True, timeout=35, creationflags=HIDDEN)
    if result.returncode:
        raise RuntimeError(result.stderr.decode('utf-8', errors='replace').strip() or '命令工具安装失败')
    reply = json.loads(result.stdout)
    if reply.get('version') != version:
        raise RuntimeError('命令工具版本核对失败')
    return version


def moba_path():
    settings = ROOT / 'settings.json'
    data = json.loads(settings.read_text(encoding='utf-8-sig')) if settings.exists() else {}
    candidates = [data.get('mobaxterm_path', ''), 'D:/RacecarTools/MobaXterm/MobaXterm.exe',
                  os.environ.get('ProgramFiles(x86)', '') + '/Mobatek/MobaXterm/MobaXterm.exe',
                  os.environ.get('ProgramFiles', '') + '/Mobatek/MobaXterm/MobaXterm.exe']
    for value in candidates:
        if value and Path(value).is_file():
            return Path(value)
    raise FileNotFoundError('未找到 MobaXterm，请在 settings.json 中填写 mobaxterm_path')


def moba_drive_path(path):
    path = str(path).replace('\\', '/')
    if not re.match(r'^[A-Za-z]:/', path):
        raise ValueError('MobaXterm 需要本机磁盘上的绝对路径：' + path)
    return '/drives/' + path[0].lower() + path[2:]


def terminal_script(config, action, version):
    from catalog import BY_KEY
    title = BY_KEY[action].title
    # All connection options are explicit; ignore the terminal's unrelated SSH
    # defaults (some MobaXterm installations generate an invalid ssh_config).
    args = ['ssh', '-F', '/dev/null', '-tt', '-p', str(config.get('port', 22)),
            '-i', moba_drive_path(config['ssh_identity_file']),
            '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'HostKeyAlias=' + config['verified_host_alias'],
            '-o', 'UserKnownHostsFile="' + moba_drive_path(config['verified_known_hosts_file']) + '"',
            '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
            config['username'] + '@' + config['host'], remote_command(action, version)]
    command = shlex.join(args)
    return ('#!/bin/bash\n'
            + 'printf "\\033]0;%s\\007" ' + shlex.quote('小车 · ' + title) + '\n'
            + "printf '%s\\n' " + shlex.quote(title + '  |  ' + config['username'] + '@' + config['host']) + '\n'
            + command + '\nresult=$?\n'
            + 'printf "\\n操作进程已结束，退出码：%s\\n" "$result"\n'
            + "read -r -p '按回车关闭此标签页…' _answer\nexit \"$result\"\n")


def launch_action(config, action, version):
    import uuid
    executable = moba_path()
    # Keep each launch immutable, so opening a second command cannot replace an
    # earlier tab's command while MobaXterm is still opening it.
    sessions = ROOT / 'terminal_sessions'
    sessions.mkdir(exist_ok=True)
    script = sessions / (action + '-' + uuid.uuid4().hex + '.sh')
    script.write_text(terminal_script(config, action, version), encoding='utf-8', newline='\n')
    return subprocess.Popen([str(executable), '-newtab', 'bash ' + shlex.quote(moba_drive_path(script))],
                            creationflags=HIDDEN)

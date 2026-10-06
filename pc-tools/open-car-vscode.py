"""Resolve the car IP and open its actual SFTP filesystem in VS Code."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from racecar_network import resolve, ssh_options

ROOT = Path(__file__).resolve().parent.parent
WORKSPACE = ROOT / 'Racecar-Car.code-workspace'


def prepare():
    print('正在查找小车并确认 SSH 连接……', flush=True)
    connection = resolve()
    remote_root = '/home/bianbu/racecar'
    check = subprocess.run(['ssh', *ssh_options(connection),
        connection['username']+'@'+connection['host'], 'test -d '+remote_root+'/src'],
        capture_output=True, timeout=10, creationflags=subprocess.CREATE_NO_WINDOW)
    if check.returncode:
        raise RuntimeError('已连接小车，但车端源码目录不可访问：'+remote_root)
    extensions = list((Path.home()/'.vscode/extensions').glob('kelvin.vscode-sshfs-*/package.json'))
    if not extensions:
        raise RuntimeError('VS Code 未安装 SSH FS 扩展，请安装 Kelvin.vscode-sshfs。')
    backup = None
    for filename in ('Racecar-Car.code-workspace','Racecar-Remote.code-workspace',
                     'Racecar-Live-Parameters.code-workspace'):
        path = ROOT / filename
        data = json.loads(path.read_text(encoding='utf-8-sig')) if path.exists() else {}
        settings = data.setdefault('settings',{})
        name = 'racecar-live' if 'Live-Parameters' in filename else 'racecar'
        configs = settings.setdefault('sshfs.configs',[])
        config = next((c for c in configs if c.get('name')==name),None)
        if config is None:
            config={'name':name};configs.append(config)
        config.update(host=connection['host'],port=connection.get('port',22),
            username=connection['username'],privateKeyPath=connection['ssh_identity_file'],
            root=remote_root,label='车端代码 '+connection['host'],
            keepaliveInterval=10000,keepaliveCountMax=3,
            terminalCommand='source /opt/ros/humble/setup.bash && source /home/bianbu/racecar/install/setup.bash && exec bash -i')
        folder = '/src/racecar/config' if name=='racecar-live' else '/'
        data['folders']=[{'uri':'ssh://'+name+folder,'name':'车端实时参数' if name=='racecar-live' else '车端源码（保存直接写入小车）'}]
        settings.update({'files.autoSave':'off','files.encoding':'utf8','files.eol':'\n'})
        settings['window.title']='车端源码 · '+connection['host']+' · ${activeEditorShort}${separator}${appName}'
        recommended=data.setdefault('extensions',{}).setdefault('recommendations',[])
        if 'kelvin.vscode-sshfs' not in recommended:recommended.append('kelvin.vscode-sshfs')
        content=json.dumps(data,ensure_ascii=False,indent=2)+'\n'
        if path.exists() and path.read_text(encoding='utf-8-sig')==content:continue
        if backup is None:
            backup=ROOT/'backups'/('vscode-car-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
            backup.mkdir(parents=True)
        if path.exists():shutil.copy2(path,backup/path.name)
        temporary=path.with_suffix('.tmp');temporary.write_text(content,encoding='utf-8');temporary.replace(path)
    print('已连接 '+connection['host']+'，即将打开车端 '+remote_root,flush=True)
    return WORKSPACE


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args()
    workspace=prepare()
    if args.prepare_only:return 0
    code=Path(os.environ['LOCALAPPDATA'])/'Programs/Microsoft VS Code/Code.exe'
    if not code.exists():raise RuntimeError('找不到 VS Code：'+str(code))
    environment=os.environ.copy();environment.pop('ELECTRON_RUN_AS_NODE',None)
    subprocess.Popen([str(code),'--new-window',str(workspace)],env=environment,
                     stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                     creationflags=subprocess.CREATE_NO_WINDOW)
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print('打开失败：'+str(exc),file=sys.stderr,flush=True)
        raise SystemExit(1)

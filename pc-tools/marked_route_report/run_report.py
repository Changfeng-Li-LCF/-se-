"""Open the latest completed marked-route report, including startup failures."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import webbrowser

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'motion_dashboard'))
from run_dashboard import find_car
import racecar_network as network
from model import clean
sys.path.insert(0,str(HERE))
from collect import collect, PATTERN
from build import build

OUTPUT=HERE.parents[1]/'reports/marked-route-dashboard'


def atomic(path,text):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+f'.{os.getpid()}.tmp')
    temp.write_text(text,encoding='utf-8');temp.replace(path)


def save_json(path,data):
    atomic(path,json.dumps(clean(data),ensure_ascii=False,allow_nan=False,separators=(',',':')))


def payload(data):
    return json.dumps(clean(data),ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c').replace('\u2028','\\u2028').replace('\u2029','\\u2029')


def fetch(requested=None):
    if requested and not re.fullmatch(PATTERN,requested):raise ValueError('运行编号格式不正确')
    cfg=find_car()
    print('已连接 '+cfg['host']+'，只读获取标定路线记录……',flush=True)
    source=(HERE/'collect.py').read_text(encoding='utf-8').replace('REQUESTED_RUN = None','REQUESTED_RUN = '+repr(requested),1)
    result=subprocess.run(['ssh',*network.ssh_options(cfg),cfg['username']+'@'+cfg['host'],'nice -n 10 python3 -'],
        input=source.encode('utf-8'),capture_output=True,timeout=180,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if result.returncode:raise RuntimeError(result.stderr.decode('utf-8',errors='replace').strip() or 'SSH 读取失败')
    print(f'已读取 {len(result.stdout)/1024:.0f} KB 精简数据。',flush=True)
    return json.loads(result.stdout)


def render(model,history,path):
    text=(HERE/'report.html').read_text(encoding='utf-8')
    library=(HERE.parent/'motion_dashboard/assets/plotly.min.js').read_text(encoding='utf-8')
    text=text.replace('/*__PLOTLY__*/',library).replace('/*__DATA__*/',payload(model)).replace('/*__HISTORY__*/',payload(history))
    atomic(path,text)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run');ap.add_argument('--local-run',type=Path)
    ap.add_argument('--offline',action='store_true');ap.add_argument('--no-open',action='store_true')
    ap.add_argument('--pause-error',action='store_true')
    args=ap.parse_args();start=time.perf_counter();raw=None;warning=None
    if args.local_run:raw=collect(args.local_run)
    elif not args.offline:
        print('正在查找小车并获取最近一次已结束的标定路线……',flush=True)
        try:raw=fetch(args.run)
        except Exception as exc:
            if args.run:raise
            warning='本次连接/读取失败，显示电脑缓存；这不一定是车端最新运行。原因：'+str(exc)
            print(warning,flush=True)
    data_dir=OUTPUT/'data';data_dir.mkdir(parents=True,exist_ok=True)
    if raw:
        model=build(raw)
        save_json(data_dir/(raw['run_id']+'.source.json'),raw)
        save_json(data_dir/(raw['run_id']+'.model.json'),model)
    cached=[]
    for p in sorted(data_dir.glob('*.model.json'),reverse=True):
        try:
            m=json.loads(p.read_text(encoding='utf-8'))
            if re.fullmatch(PATTERN,m['run_id']):cached.append(m)
        except (ValueError,KeyError,OSError):continue
    if not cached:raise RuntimeError('没有可显示的缓存，请在运行结束后连接小车再点击入口。')
    if not raw:model=cached[0]
    history=[{'run_id':m['run_id'],'outcome':m['outcome'],'file':m['run_id']+'.html'} for m in cached[:30]]
    if warning:model=dict(model,warnings=[warning,*model['warnings']])
    if args.offline:model=dict(model,warnings=['离线查看电脑缓存，未读取车端。',*model['warnings']])
    render(model,history,OUTPUT/(model['run_id']+'.html'))
    render(model,history,OUTPUT/'index.html')
    print('本次显示：'+model['run_id']+' · '+str(model['outcome']),flush=True)
    print('报告：'+str(OUTPUT/'index.html'),flush=True)
    print(f'读取和生成完成：{time.perf_counter()-start:.1f} 秒',flush=True)
    if not args.no_open:webbrowser.open((OUTPUT/'index.html').as_uri())
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as exc:
        print('报告生成失败：'+str(exc),file=sys.stderr,flush=True)
        if '--pause-error' in sys.argv:input('按回车关闭……')
        raise SystemExit(1)

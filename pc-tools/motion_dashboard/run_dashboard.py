#!/usr/bin/env python3
"""Fetch one completed run read-only, then build a self-contained local dashboard."""
import argparse
import copy
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import webbrowser

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
import racecar_network as network
from collect_run import collect_run
from model import build_model,clean

OUTPUT=HERE.parents[1]/'reports/motion-dashboard'


def find_car():
    """Use shared network identity without rewriting the main thread's config."""
    config=network.read_config()
    host=network.select_verified(list(dict.fromkeys([config['host'],*config.get('fallback_hosts',[])])),config)
    if not host and config.get('auto_discover'):
        names=[]
        for name in config.get('discovery_names',[]):
            try:names.extend(x[4][0] for x in socket.getaddrinfo(name,None,socket.AF_INET))
            except OSError:pass
        neighbors,subnet=network.subnet_candidates(network.lan_snapshot())
        host=network.select_verified(list(dict.fromkeys(names+neighbors)),config)
        if not host:host=network.select_verified(subnet,config)
    if not host:raise RuntimeError('未找到可连接的小车')
    return dict(config,host=host)


def fetch(requested=None):
    if requested and not re.fullmatch(r's-curve-\d{8}-\d{6}(?:-\d+)?',requested):
        raise ValueError('运行编号格式不正确')
    config=find_car()
    print('已连接小车 '+config['host']+'，读取最近一次已完成记录……',flush=True)
    source=(HERE/'collect_run.py').read_text(encoding='utf-8').replace('REQUESTED_RUN = None','REQUESTED_RUN = '+repr(requested),1)
    result=subprocess.run(['ssh',*network.ssh_options(config),
        config['username']+'@'+config['host'],'nice -n 10 python3 -'],
        input=source.encode('utf-8'),capture_output=True,timeout=120,
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if result.returncode:raise RuntimeError(result.stderr.decode('utf-8',errors='replace').strip())
    print(f'本次传输 {len(result.stdout)/1024:.0f} KB（没有下载整份原始日志）。',flush=True)
    return json.loads(result.stdout)


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(clean(value),ensure_ascii=False,allow_nan=False,separators=(',',':')),encoding='utf-8')
    temporary.replace(path)


def render(models,path):
    template=(HERE/'dashboard.html').read_text(encoding='utf-8')
    payload=json.dumps(clean(models),ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c').replace('\u2028','\\u2028').replace('\u2029','\\u2029')
    library=(HERE/'assets/plotly.min.js').read_text(encoding='utf-8')
    text=template.replace('/*__PLOTLY__*/',library).replace('/*__DATA__*/',payload)
    temporary=path.with_suffix('.tmp');temporary.write_text(text,encoding='utf-8');temporary.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',help='指定车端运行编号')
    parser.add_argument('--local-run',type=Path,help='分析已下载的完整运行目录')
    parser.add_argument('--offline',action='store_true',help='仅显示电脑已有缓存')
    parser.add_argument('--no-open',action='store_true')
    args=parser.parse_args()
    started=time.perf_counter();OUTPUT.mkdir(parents=True,exist_ok=True)
    raw=None;warning=None
    if args.local_run:raw=collect_run(args.local_run)
    elif not args.offline:
        try:raw=fetch(args.run)
        except Exception as exc:
            if args.run:raise
            warning='小车数据读取失败，当前显示电脑缓存：'+str(exc)
            print(warning,flush=True)
    if raw:
        run_id=raw['run_id']
        if not re.fullmatch(r's-curve-\d{8}-\d{6}(?:-\d+)?',run_id):raise ValueError('运行编号格式不正确')
        selected=build_model(raw)
        write_json(OUTPUT/'data'/f'{run_id}.source.json',raw)
        write_json(OUTPUT/'data'/f'{run_id}.model.json',selected)
    models=[]
    for path in sorted((OUTPUT/'data').glob('*.model.json'),reverse=True)[:20]:
        try:models.append(json.loads(path.read_text(encoding='utf-8')))
        except (OSError,ValueError):continue
    if not models:raise RuntimeError('没有可显示的完整记录。请连接小车或用 --local-run 指定已保存的运行目录。')
    if raw:models.sort(key=lambda x:x['run_id']!=raw['run_id'])
    if warning:
        models=copy.deepcopy(models)
        for item in models:item['warnings'].insert(0,warning)
    report=OUTPUT/(models[0]['run_id']+'.html')
    render([models[0]],report)
    index=OUTPUT/'index.html';render(models,index)
    print('当前显示：'+models[0]['run_id'],flush=True)
    print('交互图表：'+str(index),flush=True)
    print(f'读取与生成耗时：{time.perf_counter()-started:.2f} 秒',flush=True)
    if not args.no_open:webbrowser.open(index.as_uri())
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as exc:
        print('生成失败：'+str(exc),file=sys.stderr,flush=True)
        raise SystemExit(1)

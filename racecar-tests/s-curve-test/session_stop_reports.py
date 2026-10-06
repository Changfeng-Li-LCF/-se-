from pathlib import Path
import json,shutil
from datetime import datetime
def export_stop_reports(run,events,outcome,reason=None,root=None):
    run=Path(run);dst=run/'stop_reports';dst.mkdir(exist_ok=True)
    root=Path(root) if root is not None else Path.home()/'racecar-tests/stop-reports'
    rows=[]
    for item in events:
        e=item['event'];ident=e['id']
        if any(c not in '0123456789-_' for c in ident):continue
        day=datetime.fromtimestamp(e['stamp']).strftime('%Y%m%d')
        src=root/day/(ident+'.json');data={'event':e,'capture_status':'event_only_report_recorder_unavailable'}
        if src.exists():
            try:data=json.loads(src.read_text())
            except (OSError,ValueError):pass
        data['session_phase']=item['phase']
        (dst/(ident+'.json')).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
        md=root/day/(ident+'.md')
        if md.exists():shutil.copy2(md,dst/md.name)
        else:(dst/(ident+'.md')).write_text('# 停车事件\n\n'+json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
        rows.append({'id':ident,'phase':item['phase'],'reason':e['reason'],'kind':e['kind']})
    result={'outcome':outcome,'session_reason':reason,'stop_events':rows}
    (run/'stop_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    lines=['# 本次运行停车汇总','',f'运行结果：{outcome}',f'会话原因：{reason or "未记录异常"}','',
           '准备阶段的急停用于整理启动状态；清理阶段的急停是任务结束后的处理，不能覆盖行驶阶段的首次原因。','']
    for row in rows:lines.append(f"- {row['phase']} / {row['kind']}：{row['reason']}（[详细报告](stop_reports/{row['id']}.md)）")
    if not rows:lines.append('没有接收到驱动停车事件；请结合会话原因和 stack.log，不等同于确认实车没有停止。')
    (run/'stop_report.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    return result

"""Test executable routing and ONLY RViz launch actions; no sensors or motion nodes."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

mode = sys.argv[1]
assert mode in ('car', 'pc')
env = dict(os.environ, RACECAR_RVIZ_ROUTE_CHECK='1')
expected = 'RACECAR_ROUTE=' + ('car-to-pc-relay' if mode == 'car' else 'pc-local-fixed-rviz')
results = []
for title, command in [
    ('manual rviz2', ['rviz2','--help']),
    ('ros2 run', ['ros2','run','rviz2','rviz2','--help']),
    ('absolute bin', ['/opt/ros/humble/bin/rviz2','--help']),
    ('absolute lib', ['/opt/ros/humble/lib/rviz2/rviz2','--help']),
]:
    try:
        r = subprocess.run(command, env=env, capture_output=True, text=True, timeout=25)
        results.append({'entry': title, 'ok': r.returncode == 0 and expected in r.stdout,
                        'output': (r.stdout+r.stderr)[-1500:]})
    except Exception as e:
        results.append({'entry':title,'ok':False,'error':str(e)})

if mode == 'car':
    from launch import LaunchDescription, LaunchService
    from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
    from launch_ros.actions import Node
    root = Path.home()/'racecar'
    files = []
    for folder in (root/'src',root/'install'):
        for f in folder.rglob('*.py'):
            if f.is_symlink() or '/launch/' not in str(f):
                continue
            raw = f.read_text(errors='replace')
            if 'rviz2' not in raw:
                continue
            tree = ast.parse(raw)
            nodes = [n for n in ast.walk(tree) if isinstance(n,ast.Call) and any(
                k.arg=='package' and isinstance(k.value,ast.Constant) and k.value.value=='rviz2'
                for k in n.keywords)]
            if not nodes:
                continue
            paths = [next(k.value.value for k in n.keywords if k.arg=='executable') for n in nodes]
            for executable in paths:
                r = subprocess.run([executable,'--help'],env=env,capture_output=True,text=True,timeout=10)
                results.append({'entry':str(f.relative_to(root)), 'executable':executable,
                                'ok':r.returncode==0 and expected in r.stdout})
            if f.is_relative_to(root/'install'):
                files.append(f)
    # Keep original launch arguments/conditions, but run no other Node or IncludeLaunch.
    os.environ['RACECAR_RVIZ_ROUTE_CHECK'] = '1'
    for f in files:
        try:
            spec=importlib.util.spec_from_file_location('route_'+str(len(results)),f)
            module=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            description=module.generate_launch_description()
            declarations=[a for a in description.entities if isinstance(a,DeclareLaunchArgument)]
            viewers=[a for a in description.entities if isinstance(a,Node) and a.node_package=='rviz2']
            assert viewers, 'No direct RViz actions found'
            service=LaunchService()
            service.include_launch_description(LaunchDescription([
                *declarations, SetLaunchConfiguration('use_sim_time','false'),
                SetLaunchConfiguration('no_rviz','false'), SetLaunchConfiguration('use_rviz','true'),
                *viewers]))
            code=service.run()
            results.append({'entry':'RViz-only launch: '+f.name,'ok':code==0})
        except Exception as e:
            results.append({'entry':'RViz-only launch: '+f.name,'ok':False,'error':str(e)})
    r=subprocess.run(['bash',str(root/'rviz_docker.sh')],env=env,capture_output=True,text=True,timeout=10)
    results.append({'entry':'rviz_docker.sh','ok':r.returncode==0 and expected in r.stdout})

report={'mode':mode,'checks':len(results),'failed':sum(not r['ok'] for r in results),'results':results}
destination=Path.home()/('rviz-route-test-'+mode+'.json')
destination.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
raise SystemExit(1 if report['failed'] else 0)

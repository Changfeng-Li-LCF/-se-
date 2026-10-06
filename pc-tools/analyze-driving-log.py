import json
import re
from pathlib import Path
root = Path.home()/'.ros/log'
driver = root/'racecar_driver_node_31110_1790275383704.log'
values=[]
for line in driver.read_text(errors='replace').splitlines():
    m=re.search(r'线速度:\s*([-\d.]+)\s+角速度:\s*([-\d.]+)',line)
    if m: values.append(tuple(map(float,m.groups())))
result={'driver_samples':len(values)}
if values:
    result.update(motor_pwm_range=[min(v[0] for v in values),max(v[0] for v in values)],
                  servo_calculated_range=[min(v[1] for v in values),max(v[1] for v in values)],
                  negative_servo_samples=sum(v[1]<0 for v in values),
                  motor_neutral_samples=sum(v[0]==1500 for v in values))
events=[]
for name in ['controller_server_31242_1790275388901.log','bt_navigator_31245_1790275389004.log']:
    for line in (root/name).read_text(errors='replace').splitlines():
        if re.search(r'Failed|failed|Abort|abort|Cancel|cancel|Goal|goal|control loop|missed|progress',line):
            events.append(line)
result['navigation_events']=events[-18:]
bad_frames=[]
launch=root/'2026-09-25-02-43-02-757044-bianbu-31080/launch.log'
for line in launch.open(errors='replace'):
    m=re.search(r'TX: ((?:[0-9A-F]{2} ){6}[0-9A-F]{2})',line)
    if m:
        data=bytes.fromhex(m.group(1)); servo=data[3]+256*data[4]
        if servo>2500 and len(bad_frames)<5:
            bad_frames.append({'frame':m.group(1),'motor':data[1]+256*data[2],'servo':servo})
result['out_of_range_transmitted_frames']=bad_frames
print(json.dumps(result,ensure_ascii=False,indent=2))

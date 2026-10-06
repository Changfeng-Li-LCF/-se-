"""Select a local IPv4 interface for Fast DDS in this WSL environment."""
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import sys

interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show']))
local_addresses = {info['local'] for interface in interfaces for info in interface.get('addr_info', [])}
selected = os.environ.get('RACECAR_ROS_IP')
if not selected:
    connection = json.loads(Path('/mnt/d/RacecarWork/tools/racecar-connection.json').read_text(encoding='utf-8-sig'))
    route = subprocess.run(['ip', '-j', '-4', 'route', 'get', connection['host']], capture_output=True, text=True)
    if route.returncode == 0:
        routes = json.loads(route.stdout)
        if routes:
            selected = routes[0].get('prefsrc') or routes[0].get('src')
if not selected:
    candidates = [info['local'] for interface in interfaces
                  if interface['ifname'] not in ('lo', 'loopback0')
                  for info in interface.get('addr_info', []) if info.get('scope') == 'global']
    selected = candidates[0] if candidates else '127.0.0.1'
ipaddress.IPv4Address(selected)
if selected not in local_addresses:
    raise SystemExit('RACECAR_ROS_IP must be an IPv4 address assigned to this computer.')
if sys.argv[1:] == ['--print-ip']:
    print(selected)
    raise SystemExit(0)

target = Path('/mnt/d/RacecarWork/tools/fastdds-runtime.xml')
xml = f'''<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor>
    <transport_id>racecar_udp</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>{selected}</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="racecar_network" is_default_profile="true"><rtps>
    <useBuiltinTransports>false</useBuiltinTransports>
    <userTransports><transport_id>racecar_udp</transport_id></userTransports>
  </rtps></participant>
</profiles>
'''
temporary = target.with_suffix(f'.{os.getpid()}.tmp')
temporary.write_text(xml, encoding='utf-8')
temporary.replace(target)
print(f'ROS2 Fast DDS: UDP interface {selected}')

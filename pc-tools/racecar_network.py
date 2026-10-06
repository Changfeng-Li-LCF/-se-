"""Discover the configured car on the current LAN; authenticate its pinned SSH key."""
from concurrent.futures import ThreadPoolExecutor
import ipaddress
import json
import os
from pathlib import Path
import socket
import subprocess

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / 'racecar-connection.json'
HIDDEN = getattr(subprocess, 'CREATE_NO_WINDOW', 0)


def read_config():
    return json.loads(CONFIG.read_text(encoding='utf-8-sig'))


def ssh_options(config):
    return ['-p', str(config.get('port', 22)), '-i', config['ssh_identity_file'],
            '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=3',
            '-o', 'HostKeyAlias='+config['verified_host_alias'],
            '-o', 'UserKnownHostsFile="'+config['verified_known_hosts_file']+'"']


def lan_snapshot():
    script = r'''
$entries = @(Get-NetAdapter -Physical | Where-Object Status -eq 'Up' | ForEach-Object {
  Get-NetIPAddress -InterfaceIndex $_.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object AddressState -eq 'Preferred' | Select-Object IPAddress,PrefixLength
})
$neighbors = @(Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Where-Object { $_.State -in @('Reachable','Stale','Delay','Probe','Permanent') } |
  Select-Object -ExpandProperty IPAddress)
@{interfaces=$entries;neighbors=$neighbors} | ConvertTo-Json -Compress
'''
    result = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', script],
                            capture_output=True, text=True, timeout=12, creationflags=HIDDEN)
    if result.returncode:
        raise RuntimeError('Cannot discover local network: '+result.stderr.strip())
    return json.loads(result.stdout)


def subnet_candidates(snapshot, max_hosts=254):
    """Bound discovery to active physical private IPv4 LANs, at most one /24 each."""
    local = {i['IPAddress'] for i in snapshot['interfaces']}
    networks = []
    private = [ipaddress.ip_network(n) for n in ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16')]
    for item in snapshot['interfaces']:
        address = ipaddress.ip_address(item['IPAddress'])
        if not any(address in n for n in private):
            continue
        network = ipaddress.ip_network(f"{address}/{max(24,int(item['PrefixLength']))}", strict=False)
        networks.append(network)
    neighbors = [h for h in snapshot['neighbors'] if h not in local and
                 any(ipaddress.ip_address(h) in n for n in networks)]
    addresses = [str(h) for n in networks for h in list(n.hosts())[:max_hosts] if str(h) not in local]
    return list(dict.fromkeys(neighbors)), list(dict.fromkeys(addresses))


def reachable(host, port):
    try:
        with socket.create_connection((host, port), timeout=.35):
            return host
    except OSError:
        return None


def verified(host, config):
    result = subprocess.run(['ssh', *ssh_options(config), config['username']+'@'+host,
                             'printf RACECAR_IDENTITY_OK'], capture_output=True,
                            timeout=6, creationflags=HIDDEN)
    return result.returncode == 0 and result.stdout == b'RACECAR_IDENTITY_OK'


def select_verified(candidates, config):
    with ThreadPoolExecutor(max_workers=32) as pool:
        opened = list(pool.map(lambda h: reachable(h, config.get('port',22)), candidates))
    for host in filter(None, opened):
        try:
            if verified(host, config):
                return host
        except subprocess.TimeoutExpired:
            continue
    return None


def remember(host, original):
    # Preserve parameter/configuration edits made while discovery was running.
    current = read_config()
    if current['host'] != original['host'] and current['host'] != host:
        raise RuntimeError('Connection configuration changed during discovery; retry')
    current['fallback_hosts'] = list(dict.fromkeys([host,current['host'],*current.get('fallback_hosts',[])]))[:16]
    current['host'] = host
    temporary = CONFIG.with_suffix(f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(current,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temporary.replace(CONFIG)
    return current


def resolve():
    config = read_config()
    configured = list(dict.fromkeys([config['host'],*config.get('fallback_hosts',[])]))
    host = select_verified(configured, config)
    if host:
        return remember(host, config)
    if not config.get('auto_discover',False):
        raise RuntimeError('Configured car addresses are unavailable')
    names = []
    for name in config.get('discovery_names',[]):
        try:
            names.extend(i[4][0] for i in socket.getaddrinfo(name,None,socket.AF_INET))
        except OSError:
            pass
    snapshot = lan_snapshot()
    neighbors, subnet = subnet_candidates(snapshot, min(254,int(config.get('discovery_max_subnet_hosts',254))))
    for candidates in (list(dict.fromkeys(names+neighbors)),subnet):
        host = select_verified(candidates,config)
        if host:
            return remember(host,config)
    raise RuntimeError('No car with the saved SSH host key found on the current LAN')


if __name__ == '__main__':
    print(json.dumps(resolve(),ensure_ascii=False))

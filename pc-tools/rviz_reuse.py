"""Pure request identity and process ownership rules for the PC RViz relay."""
import base64
import hashlib
import json
import re

FILE_FLAGS = ('-d', '--display-config', '--params-file', '-s', '--splash-screen')
KINDS = ('default', 's_curve', 'live_slam')
MANAGED = 'pc-relay-v2'
VALID_ID = re.compile(r'^[a-f0-9]{32}$')
LEGACY_CONFIG = re.compile(r'^/mnt/d/RacecarWork/verification/rviz-relay/[a-f0-9]{32}/[0-9]+\.rviz$')


def request_spec(request, default_config=None):
    if not VALID_ID.fullmatch(request.get('id', '')):
        raise ValueError('Invalid request ID')
    args = request.get('args', [])
    if not isinstance(args, list) or len(args) > 100 or not all(isinstance(a, str) and len(a) < 4096 for a in args):
        raise ValueError('Invalid arguments')
    domain = int(request.get('domain', 0))
    if not 0 <= domain <= 232:
        raise ValueError('Invalid ROS domain')
    canonical = list(args)
    files = {}
    kind = 'default'
    payload = request.get('files', {})
    if not isinstance(payload, dict):
        raise ValueError('Invalid files')
    for key, item in payload.items():
        index = int(key)
        if index <= 0 or index >= len(args) or args[index-1] not in FILE_FLAGS:
            raise ValueError('Invalid transferred file position')
        content = base64.b64decode(item['data'], validate=True)
        if len(content) > 2_000_000:
            raise ValueError('File too large')
        files[index] = content
        canonical[index] = {'sha256': hashlib.sha256(content).hexdigest()}
        if args[index-1] in ('-d', '--display-config'):
            name = item.get('name', '')
            if name == 'live_slam_run.rviz' or b'/live_slam_actual_path' in content:
                kind = 'live_slam'
            elif name == 's_curve_live.rviz':
                kind = 's_curve'
    # Never pass an untransferred car-local config path to the Windows viewer.
    for index, arg in enumerate(args[:-1]):
        if arg in FILE_FLAGS and index+1 not in files:
            raise ValueError('Missing transferred RViz file')
    identity = {'args': canonical, 'domain': domain, 'kind': kind}
    if not any(arg in ('-d', '--display-config') for arg in args):
        if default_config is None:
            raise ValueError('Default RViz configuration bytes are required')
        identity['default_config'] = hashlib.sha256(default_config).hexdigest()
    fingerprint = hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return {'id': request['id'], 'args': list(args), 'files': files,
            'domain': domain, 'kind': kind, 'fingerprint': fingerprint}


def managed_kind(env, argv, read_config):
    """A bare rviz2 process is never ours, including processes without a marker."""
    kind = env.get('RACECAR_RVIZ_INSTANCE')
    if kind not in KINDS:
        return None
    if env.get('RACECAR_RVIZ_MANAGED') == MANAGED:
        return kind
    # Old relay versions supplied an instance marker plus this unique config path.
    configs = [argv[i+1] for i, value in enumerate(argv[:-1])
               if value in ('-d', '--display-config') and LEGACY_CONFIG.fullmatch(argv[i+1])]
    if not configs:
        return None
    if kind == 'default':
        try:
            if b'/live_slam_actual_path' in read_config(configs[-1]):
                return 'live_slam'
        except OSError:
            pass
    return kind


def can_reuse(record, kind, domain, fingerprint, ip):
    env = record['env']
    return (record['kind'] == kind and env.get('RACECAR_RVIZ_MANAGED') == MANAGED
            and env.get('RACECAR_RVIZ_FINGERPRINT') == fingerprint
            and env.get('RACECAR_RVIZ_BIND_IP') == ip
            and env.get('ROS_DOMAIN_ID', '0') == str(domain))

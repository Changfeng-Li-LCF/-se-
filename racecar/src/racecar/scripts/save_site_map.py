#!/usr/bin/env python3
"""Save the current full /map to a new version; never stop SLAM or overwrite old maps."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', nargs='?', default='site')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.name):
        parser.error('name must contain only letters, digits, underscore or hyphen')
    maps = Path.home() / 'maps'
    maps.mkdir(exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    directory = maps / (args.name + '-' + stamp)
    # The temporary directory is retained on failure for diagnosis.
    stage = Path(tempfile.mkdtemp(prefix='.saving-', dir=maps))
    prefix = stage / args.name
    result = subprocess.run([
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
        '--ros-args', '-p', 'save_map_timeout:=15.0',
        '-p', 'map_subscribe_transient_local:=false',
    ])
    if result.returncode:
        raise RuntimeError('Map was not saved. Existing maps unchanged. Files: ' + str(stage))
    metadata = prefix.with_suffix('.yaml')
    data = yaml.safe_load(metadata.read_text())
    image = Path(data['image'])
    if not image.is_absolute():
        image = stage / image
    if not image.is_file() or image.stat().st_size == 0:
        raise RuntimeError('Saved map image is missing or empty: ' + str(image))
    # Ensure YAML remains valid when the version directory is renamed.
    if image.parent.resolve() != stage.resolve():
        raise RuntimeError('Unexpected map image path: ' + str(image))
    data['image'] = image.name
    metadata.write_text(yaml.safe_dump(data, sort_keys=False))
    (stage / 'save_info.json').write_text(json.dumps({
        'saved_at': datetime.now().isoformat(), 'topic': '/map',
        'kind': 'occupancy_grid_snapshot',
        'note': 'Captures all mapped cells, including any existing mapping errors. Not a SLAM pose graph.',
    }, indent=2))
    stage.rename(directory)
    # Only the convenience pointer changes; old versions are preserved.
    pointer = maps / 'site_latest.yaml'
    temporary = maps / ('.site_latest-' + stamp)
    temporary.symlink_to(directory / metadata.name)
    if pointer.exists() and not pointer.is_symlink():
        temporary.unlink()
        print('Saved map:', directory / metadata.name)
        print('Existing regular site_latest.yaml preserved. Pass the saved YAML explicitly to nav.')
        return
    os.replace(temporary, pointer)
    print('Saved full /map:', directory / metadata.name)
    print('Navigation: bash ~/racecar/src/racecar/scripts/site_map.sh nav')


if __name__ == '__main__':
    main()

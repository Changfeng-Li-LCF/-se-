"""Check custom planner metadata and dependencies before starting any nodes."""
from pathlib import Path
import ctypes
import xml.etree.ElementTree as ET
import yaml


def validate_planner_installation(workspace):
    config = Path(workspace) / 'install/racecar/share/racecar/config/nav_carto.yaml'
    parameters = yaml.safe_load(config.read_text())['planner_server']['ros__parameters']
    selected = [parameters[name]['plugin'] for name in parameters['planner_plugins']]
    plugin = 'racecar_smac_planner/SmacPlannerHybrid'
    if plugin not in selected:
        return []
    from ament_index_python.packages import get_package_prefix
    from ament_index_python.resources import get_resource
    package = 'racecar_smac_planner'
    prefix = Path(get_package_prefix(package))
    manifest = prefix / 'share' / package / 'package.xml'
    name = ET.parse(manifest).getroot().findtext('name')
    if name != package:
        raise RuntimeError(f'Planner package manifest mismatch: {manifest}: name={name!r}, expected {package!r}; no stack started')
    resource, resource_prefix = get_resource('nav2_core__pluginlib__plugin', package)
    if Path(resource_prefix) != prefix:
        raise RuntimeError('Planner package and plugin resource resolve to different install prefixes')
    descriptions = [prefix / line for line in resource.splitlines() if line.strip()]
    libraries = []
    for description in descriptions:
        root = ET.parse(description).getroot()
        for library in ([root] if root.tag == 'library' else root.findall('library')):
            if any(entry.get('name') == plugin and entry.get('base_class_type') == 'nav2_core::GlobalPlanner'
                   for entry in library.findall('class')):
                libraries.append(prefix / 'lib' / ('lib' + library.attrib['path'] + '.so'))
    if len(libraries) != 1 or not libraries[0].is_file():
        raise RuntimeError(f'Planner plugin library missing or ambiguous: {libraries}; no stack started')
    ctypes.CDLL(str(libraries[0]))
    return [manifest, *descriptions, *libraries]

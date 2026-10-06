"""Execute ONLY the RViz action from the real explorer launch description."""
import importlib.util
from pathlib import Path
from launch import LaunchDescription, LaunchService
from launch.actions import SetLaunchConfiguration
from launch_ros.actions import Node

path = Path.home() / 'racecar/install/racecar/share/racecar/launch/Run_explorer.launch.py'
spec = importlib.util.spec_from_file_location('car_explorer_launch_check', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
description = module.generate_launch_description()
rviz_actions = [action for action in description.entities
                if isinstance(action, Node) and action.node_package == 'rviz2']
assert len(rviz_actions) == 1, 'Expected exactly one RViz action'
service = LaunchService()
service.include_launch_description(LaunchDescription([
    SetLaunchConfiguration('use_sim_time', 'false'),
    SetLaunchConfiguration('no_rviz', 'false'),
    *rviz_actions,
]))
raise SystemExit(service.run())

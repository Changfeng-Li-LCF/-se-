"""Shared launch-time planner geometry; runtime adaptation uses the same config."""
import math
from pathlib import Path
import yaml

def paths(share):
    driver=Path.home()/'racecar/src/racecar/config/driver_calibration.yaml'
    if not driver.is_file():driver=Path(share)/'config/driver_calibration.yaml'
    return str(driver),str(Path(share)/'config/dynamic_turning_radius.yaml')

def boundary_guard_enabled(share):
    driver = Path(share)/'config/driver_calibration.yaml'
    value = yaml.safe_load(driver.read_text())['racecar_driver']['ros__parameters']['map_boundary_guard_enabled']
    if not isinstance(value, bool):
        raise ValueError('map_boundary_guard_enabled must be a boolean')
    return value


def startup_radius(share):
    """Map search radius lower bound, independent of measured/cruise speed."""
    _,policy=paths(share)
    p=yaml.safe_load(Path(policy).read_text())['speed_dependent_turning_radius']['ros__parameters']
    radius=float(p['minimum_turning_radius_m'])
    if not math.isfinite(radius) or radius<=0:raise ValueError('Invalid minimum planning radius')
    return radius

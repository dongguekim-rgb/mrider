"""Read official vehicle overlays for the experimental planar sensor simulator."""
import importlib.util
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def measured_parameters(vehicle, config):
    desc=Path(get_package_share_directory('mitt_description'))
    spec=importlib.util.spec_from_file_location('team_vehicle_config',desc/'config/vehicle_config.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    number=module.parse_vehicle(vehicle)
    if number==0:return {},None
    over=module.load_overlay(str(desc),number)
    if 'min_turning_radius' not in over:raise ValueError('measured turning radius required for team vehicle mode')
    dims=module._estimates(str(desc));dims.update(over)
    points=yaml.safe_load(module.footprint(dims))
    # The manual lock measurement constrains the virtual model; this is not
    # a calibrated steering actuator angle or a firmware parameter.
    limits={'wheelbase':float(dims['wheelbase']),
            'front':max(p[0] for p in points),'rear':-min(p[0] for p in points),
            'half_width':max(abs(p[1]) for p in points),
            'max_steering':min(math.radians(dims['steer_limit_deg']),math.atan(dims['wheelbase']/dims['min_turning_radius']))}
    nav=yaml.safe_load(Path(config).read_text())
    for name in ('global_costmap','local_costmap'):
        params=nav[name][name]['ros__parameters']
        params['footprint']=module.footprint(dims)
        # The official radius is a minimum, not permission to shrink the
        # experiment's wider obstacle cost field when selecting a vehicle.
        params['inflation_layer']['inflation_radius']=max(
            params['inflation_layer']['inflation_radius'],module.inflation_radius(dims))
    planner=nav['planner_server']['ros__parameters']['GridBased']
    planner['minimum_turning_radius']=max(planner['minimum_turning_radius'],module.planner_radius(dims['min_turning_radius']))
    return limits,nav

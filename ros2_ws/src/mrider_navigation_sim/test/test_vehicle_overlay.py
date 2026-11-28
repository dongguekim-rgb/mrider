"""Measured geometry must not shrink a configured planning margin."""
import math
from pathlib import Path
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from mrider_navigation_sim.vehicle_overlay import measured_parameters


@pytest.mark.parametrize('requested_radius', [.01, 2.])
def test_vehicle_preserves_or_expands_inflation(tmp_path, requested_radius):
    source = Path(get_package_share_directory('mrider_navigation_sim')) / 'config/nav2_sensor_only.yaml'
    config = yaml.safe_load(source.read_text())
    for name in ('local_costmap', 'global_costmap'):
        config[name][name]['ros__parameters']['inflation_layer']['inflation_radius'] = requested_radius
    path = tmp_path / 'navigation.yaml'
    path.write_text(yaml.safe_dump(config))
    limits, actual = measured_parameters('1', str(path))
    for name in ('local_costmap', 'global_costmap'):
        parameters = actual[name][name]['ros__parameters']
        radius = parameters['inflation_layer']['inflation_radius']
        points = yaml.safe_load(parameters['footprint'])
        assert radius >= requested_radius
        assert radius >= max(math.hypot(*point) for point in points)
        assert max(point[0] for point in points) == limits['front']
    assert path.read_text() == yaml.safe_dump(config)

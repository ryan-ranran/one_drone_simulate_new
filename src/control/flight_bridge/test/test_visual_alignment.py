import importlib.util
import math
from pathlib import Path

import pytest

module_path = Path(__file__).parents[1] / 'src' / 'visual_alignment.py'
spec = importlib.util.spec_from_file_location('visual_alignment', module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
alignment_ready = module.alignment_ready
alignment_velocity = module.alignment_velocity


def test_velocity_is_gain_limited_and_points_toward_tag():
    assert alignment_velocity(.1, -.2, .6, .15) == pytest.approx((.06, -.12))


def test_velocity_magnitude_is_capped():
    x, y = alignment_velocity(2., 2., 1., .15)
    assert math.hypot(x, y) == pytest.approx(.15)
    assert x == pytest.approx(y)


def test_alignment_requires_position_and_speed_tolerance():
    assert alignment_ready(.03, .04, .02, .06, .05)
    assert not alignment_ready(.05, .05, .02, .06, .05)
    assert not alignment_ready(.01, .0, .1, .06, .05)

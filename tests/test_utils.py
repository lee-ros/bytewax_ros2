import pytest

from bytewax_ros.utilities import value_from_obj


@pytest.mark.parametrize(
    "obj, attribute, value",
    [({"a": {"b": 5}}, "obj['a']['b']", 5), ([1, 2, 3], "obj[2]", 3), (1, "obj", 1)],
)
def test_value_from_obj(obj, attribute, value):
    assert value_from_obj(obj, attribute) == value

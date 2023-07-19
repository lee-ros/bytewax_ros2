import pytest

from bytewax_ros import execution


@pytest.mark.parametrize(
    "obj, attribute, value",
    [({"a": {"b": 5}}, "obj['a']['b']", 5), ([1, 2, 3], "obj[2]", 3), (1, "obj", 1)],
)
def test_value_from_obj(obj, attribute, value):
    assert execution.eval_value_from_obj(obj, attribute) == value

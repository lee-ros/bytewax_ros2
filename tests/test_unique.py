from bytewax_ros.unique import Unique


def test_unique():
    unique = Unique()
    samples = [1] * 3 + [2] + [3] * 2 + [1] * 1_000
    excepted_result = [1, 2, 3, 1]

    result = []
    for sample in samples:
        if s := unique(sample):
            result.append(s)

    assert all(x == y for x, y in zip(result, excepted_result))

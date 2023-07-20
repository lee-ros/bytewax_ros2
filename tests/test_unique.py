from bytewax_ros.unique import UniqueFilter


def test_unique():
    unique_filter = UniqueFilter()
    samples = [1] * 3 + [2] + [3] * 2 + [1] * 1_000
    excepted_result = [1, 2, 3, 1]

    result = [sample for sample in samples if unique_filter(sample)]
    assert all(x == y for x, y in zip(result, excepted_result))

import math
import random

import pytest

from pai_task_env.sampler import ObjectSampler, SamplingError


@pytest.fixture
def sampler():
    return ObjectSampler(x=(0.10, 0.20), y=(-0.10, 0.10), r=(0.09, 0.22),
                         place_xy=(0.12, -0.12), min_dist_from_place=0.06)


def test_samples_are_in_range(sampler):
    rng = random.Random(0)
    for _ in range(500):
        x, y = sampler.sample(rng)
        assert 0.10 <= x <= 0.20
        assert -0.10 <= y <= 0.10
        assert 0.09 <= math.hypot(x, y) <= 0.22
        assert math.hypot(x - 0.12, y + 0.12) >= 0.06


def test_seed_is_reproducible(sampler):
    a = [sampler.sample(random.Random(42)) for _ in range(3)]
    b = [sampler.sample(random.Random(42)) for _ in range(3)]
    assert a == b


def test_impossible_constraints_raise():
    s = ObjectSampler(x=(0.10, 0.11), y=(0.0, 0.01), r=(0.5, 0.6),
                      place_xy=(0.0, 0.0), min_dist_from_place=0.0, max_attempts=50)
    with pytest.raises(SamplingError):
        s.sample(random.Random(0))


def test_from_dict():
    s = ObjectSampler.from_dict({
        'x': [0.1, 0.2], 'y': [-0.1, 0.1], 'r': [0.0, 1.0],
        'place_xy': [0.0, -0.15], 'min_dist_from_place': 0.05,
    })
    assert s.place_xy == (0.0, -0.15)


def test_sample_positions_is_deterministic_and_valid(sampler):
    from pai_task_env.sampler import sample_positions
    a = sample_positions(sampler, n=30, seed=7)
    b = sample_positions(sampler, n=30, seed=7)
    assert a == b and len(a) == 30
    assert all(sampler.is_valid(x, y) for x, y in a)
    assert sample_positions(sampler, n=30, seed=8) != a

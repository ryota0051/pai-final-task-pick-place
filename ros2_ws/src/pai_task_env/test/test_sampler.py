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


# ---- 置き場所のランダム化（物体と置き場所の両方を毎回変える） ----
from pai_task_env.sampler import PlaceSampler, sample_scene  # noqa: E402


@pytest.fixture
def place_sampler():
    return PlaceSampler(x=(0.04, 0.16), y=(-0.14, 0.14), r=(0.09, 0.165))


def test_place_samples_are_in_range(place_sampler):
    rng = random.Random(0)
    for _ in range(500):
        x, y = place_sampler.sample(rng)
        assert 0.04 <= x <= 0.16 and -0.14 <= y <= 0.14
        assert 0.09 <= math.hypot(x, y) <= 0.165


def test_place_sampler_from_dict():
    s = PlaceSampler.from_dict({'x': [0.04, 0.16], 'y': [-0.14, 0.14], 'r': [0.09, 0.165]})
    assert s.x == (0.04, 0.16) and s.r == (0.09, 0.165)


def test_object_keeps_distance_from_given_place(sampler):
    rng = random.Random(1)
    place = (0.15, 0.0)
    for _ in range(300):
        x, y = sampler.sample(rng, place_xy=place)
        assert math.hypot(x - place[0], y - place[1]) >= 0.06
        assert sampler.is_valid(x, y, place_xy=place)


def test_sample_scene_with_fixed_place_uses_configured_place(sampler):
    obj, place = sample_scene(sampler, None, random.Random(0))
    assert place == (0.12, -0.12)
    assert sampler.is_valid(*obj)


def test_sample_scene_with_random_place(sampler, place_sampler):
    rng = random.Random(3)
    places = set()
    for _ in range(100):
        obj, place = sample_scene(sampler, place_sampler, rng)
        assert sampler.is_valid(*obj, place_xy=place)
        places.add(place)
    assert len(places) == 100


def test_sample_positions_with_place_returns_four_values(sampler, place_sampler):
    from pai_task_env.sampler import sample_positions
    a = sample_positions(sampler, n=30, seed=7, place_sampler=place_sampler)
    assert a == sample_positions(sampler, n=30, seed=7, place_sampler=place_sampler)
    assert all(len(p) == 4 for p in a)
    assert all(sampler.is_valid(ox, oy, place_xy=(px, py)) for ox, oy, px, py in a)

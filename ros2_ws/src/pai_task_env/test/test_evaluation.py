import csv

import pytest

from pai_task_env.evaluation import TrialResult, summarize, wilson_interval, write_csv


def test_wilson_interval_known_values():
    lo, hi = wilson_interval(8, 10)
    assert lo == pytest.approx(0.4902, abs=1e-3)
    assert hi == pytest.approx(0.9433, abs=1e-3)


def test_wilson_interval_edges():
    assert wilson_interval(0, 0) == (0.0, 0.0)
    lo, hi = wilson_interval(0, 5)
    assert lo == 0.0 and 0.0 < hi < 1.0
    lo, hi = wilson_interval(5, 5)
    assert hi == pytest.approx(1.0) and lo < 1.0


def _r(i, success, t=10.0, lifted=True):
    return TrialResult(trial=i, success=success, lifted=lifted, duration=t, object_x=0.1, object_y=0.0)


def test_summarize():
    res = [_r(0, True, 8.0), _r(1, False, 30.0, lifted=False), _r(2, True, 12.0), _r(3, False, 30.0)]
    s = summarize(res)
    assert s['trials'] == 4
    assert s['successes'] == 2
    assert s['success_rate'] == pytest.approx(0.5)
    assert s['lift_rate'] == pytest.approx(0.75)
    assert s['mean_success_time'] == pytest.approx(10.0)
    assert s['ci95_low'] < 0.5 < s['ci95_high']


def test_summarize_no_success():
    s = summarize([_r(0, False)])
    assert s['mean_success_time'] is None


def test_write_csv(tmp_path):
    path = tmp_path / 'out' / 'r.csv'
    write_csv(path, [_r(0, True), _r(1, False)])
    rows = list(csv.DictReader(open(path)))
    assert len(rows) == 2
    assert rows[0]['success'] == 'True'
    assert set(rows[0]) == {'trial', 'success', 'lifted', 'duration', 'object_x', 'object_y'}

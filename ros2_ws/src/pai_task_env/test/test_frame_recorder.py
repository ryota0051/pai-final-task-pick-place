from pai_task_env.frame_recorder import FrameRateLimiter


def test_keeps_frames_at_target_rate():
    lim = FrameRateLimiter(fps=10.0)
    kept = [t for t in [0.00, 0.05, 0.10, 0.15, 0.20, 0.31] if lim.accept(t)]
    assert kept == [0.00, 0.10, 0.20, 0.31]


def test_does_not_drift_when_input_is_jittery():
    lim = FrameRateLimiter(fps=10.0)
    ts = [i * 0.05 + (0.004 if i % 2 else -0.004) for i in range(1, 41)]
    kept = [t for t in ts if lim.accept(t)]
    assert len(kept) == 20

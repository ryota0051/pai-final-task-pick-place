"""提出動画の素材を録画する（コンテナ内で実行。シミュレーションのスタックは起動済みであること）.

python3 ~/final_task/scripts/video/record_clips.py [clip 名 ...]   # 省略時は docs/report/video.yaml の全 clip
Gazebo GUI の 3D ビューを画面録画し（capture 設定）、方策 ON/OFF・成功の時刻を event_logger で記録する。
出力: <media_dir>/<clip>.mp4 と <clip>.mp4.events.csv（動画内の秒）
"""
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / 'scripts' / 'sim'


def run(cmd, **kw):
    print('$', ' '.join(map(str, cmd)), flush=True)
    return subprocess.run(cmd, check=True, **kw)


def to_video_time(abs_csv: Path, out_csv: Path, t0: float):
    """event_logger の壁時計の時刻を、録画開始からの秒に直す."""
    rows = list(csv.DictReader(open(abs_csv)))
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time', 'event'])
        for r in rows:
            w.writerow([round(float(r['time']) - t0, 2), r['event']])


def stop(proc, sig=signal.SIGINT):
    os.killpg(proc.pid, sig)  # ros2 run などの子プロセスまで止める
    proc.wait(timeout=30)


def record(name, spec, cap, media_dir):
    out = media_dir / f'{name}.mp4'
    abs_csv = media_dir / f'{name}.events_abs.csv'
    x, y, w, h = cap['region']
    logger = subprocess.Popen(['ros2', 'run', 'pai_task_env', 'event_logger', '--ros-args',
                               '-p', f'output:={abs_csv}'], start_new_session=True)
    time.sleep(2)
    t0 = time.time()
    grab = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'x11grab', '-framerate', str(cap['framerate']),
                             '-video_size', f'{w}x{h}', '-i', f"{cap['display']}+{x},{y}",
                             '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '18', str(out)],
                            stdin=subprocess.PIPE, start_new_session=True)
    try:
        if spec['kind'] == 'eval':
            pos = ','.join(str(v) for v in spec['positions'])
            run(['bash', str(SIM / 'eval_positions.sh'), f'video_{name}', spec['infer_config'], pos,
                 *spec.get('args', [])])
        elif spec['kind'] == 'scripted':
            run(['ros2', 'run', 'pai_task_env', 'scripted_demo', '--ros-args', '--params-file',
                 str(ROOT / 'ros2_ws/src/pai_task_env/config/scripted_demo.yaml'),
                 '-p', f"episodes:={spec['episodes']}", '-p', f"seed:={spec['seed']}", '-p', 'record:=false',
                 *spec.get('args', [])])
        else:
            raise ValueError(f"unknown kind: {spec['kind']}")
        time.sleep(2)
    finally:
        grab.communicate(b'q', timeout=30)   # ffmpeg は q で正常終了（mp4 を閉じる）
        stop(logger)
    to_video_time(abs_csv, Path(f'{out}.events.csv'), t0)
    abs_csv.unlink()
    print(f'recorded {out}', flush=True)


def main():
    cfg = yaml.safe_load((ROOT / 'docs' / 'report' / 'video.yaml').read_text(encoding='utf-8'))
    cap = cfg['capture']
    media_dir = Path(os.path.expanduser(cfg['media_dir']))
    media_dir.mkdir(parents=True, exist_ok=True)
    names = sys.argv[1:] or list(cfg['clips'])
    os.environ['DISPLAY'] = cap['display']
    run(['wmctrl', '-r', 'Gazebo', '-b', 'add,maximized_vert,maximized_horz'])
    run(['xdotool', 'mousemove', '1700', '800'])   # カーソルを録画範囲の外へ
    run(['bash', str(SIM / 'gui_view.sh'), *map(str, cap['view'])])
    time.sleep(2)
    for name in names:
        record(name, cfg['clips'][name], cap, media_dir)


if __name__ == '__main__':
    main()

"""docs/report/video.yaml のタイムラインに従って提出動画を編集する（コンテナ内で実行）.

python3 ~/final_task/scripts/video/compose.py
素材: record_clips.py が録画した <media_dir>/<clip>.mp4 と .events.csv
字幕・カードは Pillow で画像にして ffmpeg で重ねる（ナレーションなし）。
"""
import csv
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
BG = (16, 22, 34)
ACCENT = (42, 111, 219)


class Composer:
    def __init__(self, cfg, work: Path):
        self.cfg = cfg
        self.W, self.H = cfg['size']
        self.fps = cfg['fps']
        self.media = Path(os.path.expanduser(cfg['media_dir']))
        self.img_dir = Path(os.path.expanduser(cfg['report_img_dir']))
        self.work = work
        self.n = 0

    # ---- 描画 ----
    def font(self, size, bold=True):
        return ImageFont.truetype(self.cfg['font'] if bold else self.cfg['font_regular'], size)

    def wrap(self, draw, text, font, width):
        lines, cur = [], ''
        for ch in text:
            if draw.textlength(cur + ch, font=font) > width:
                lines.append(cur)
                cur = ch
            else:
                cur += ch
        return lines + [cur] if cur else lines

    def caption_layer(self, text, speed=1.0):
        """下部の字幕帯（半透明）と、倍速時は右上に倍速表示を描いた透過 PNG."""
        img = Image.new('RGBA', (self.W, self.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        if text:
            f = self.font(30)
            lines = self.wrap(d, text, f, self.W - 120)
            h = 26 + 44 * len(lines)
            d.rectangle([0, self.H - h, self.W, self.H], fill=(0, 0, 0, 170))
            for i, line in enumerate(lines):
                d.text((self.W // 2, self.H - h + 13 + 44 * i), line, font=f, fill='white', anchor='ma')
        if speed != 1:
            f = self.font(28)
            label = f'{speed:g}倍速'
            d.rounded_rectangle([self.W - 150, 18, self.W - 18, 70], radius=10, fill=(0, 0, 0, 170))
            d.text((self.W - 84, 44), label, font=f, fill='white', anchor='mm')
        return self.save(img)

    def card(self, title, subtitle=None, lines=()):
        img = Image.new('RGB', (self.W, self.H), BG)
        d = ImageDraw.Draw(img)
        y = self.H // 2 - 40 * (len(lines) + 1) - (20 if subtitle else 0)
        d.text((self.W // 2, y), title, font=self.font(48), fill='white', anchor='mm')
        d.line([self.W // 2 - 200, y + 42, self.W // 2 + 200, y + 42], fill=ACCENT, width=4)
        y += 95
        if subtitle:
            d.text((self.W // 2, y), subtitle, font=self.font(28, bold=False), fill=(200, 210, 225), anchor='mm')
            y += 60
        for line in lines:
            d.text((self.W // 2, y), line, font=self.font(32, bold=False), fill='white', anchor='mm')
            y += 62
        return self.save(img)

    def image_frame(self, name):
        img = Image.new('RGB', (self.W, self.H), BG)
        src = Image.open(self.img_dir / name).convert('RGB')
        box_w, box_h = self.W - 80, self.H - 170
        scale = min(box_w / src.width, box_h / src.height)
        src = src.resize((int(src.width * scale), int(src.height * scale)), Image.LANCZOS)
        img.paste(src, ((self.W - src.width) // 2, 30 + (box_h - src.height) // 2))
        return self.save(img)

    def label_layer(self, labels):
        img = Image.new('RGBA', (self.W, self.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        f = self.font(26)
        for i, text in enumerate(labels):
            cx = self.W // 4 + i * self.W // 2
            d.rounded_rectangle([cx - 250, 92, cx + 250, 138], radius=8, fill=(0, 0, 0, 170))
            d.text((cx, 115), text, font=f, fill='white', anchor='mm')
        return self.save(img)

    def save(self, img):
        self.n += 1
        p = self.work / f'layer_{self.n:03d}.png'
        img.save(p)
        return p

    # ---- 素材の切り出し ----
    def trial_windows(self, clip, trials):
        """events.csv から試行ごとの [方策 ON の 0.5 秒前, 成功の 1 秒後（なければ OFF）] を返す."""
        rows = list(csv.DictReader(open(self.media / f'{clip}.mp4.events.csv')))
        ons = [i for i, r in enumerate(rows) if r['event'] == 'policy_on']
        out = []
        for k in trials:
            i = ons[k]
            start = max(0.0, float(rows[i]['time']) - 0.5)
            end = None
            for r in rows[i + 1:]:
                if r['event'] == 'success':
                    end = float(r['time']) + 1.0
                    break
                if r['event'] == 'policy_off':
                    end = float(r['time'])
                    break
            out.append((start, end))
        return out

    def ffmpeg(self, args):
        subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', *args], check=True)

    def encode_opts(self):
        return ['-r', str(self.fps), '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'medium', '-crf', '20', '-an']

    def segment_out(self):
        self.n += 1
        return self.work / f'seg_{self.n:03d}.mp4'

    def still(self, frame, duration, overlay=None):
        out = self.segment_out()
        inputs = ['-loop', '1', '-t', str(duration), '-i', str(frame)]
        if overlay:
            inputs += ['-loop', '1', '-t', str(duration), '-i', str(overlay)]
            filt = ['-filter_complex', '[0:v][1:v]overlay=0:0']
        else:
            filt = []
        self.ffmpeg([*inputs, *filt, *self.encode_opts(), str(out)])
        return out

    def cut(self, clip, start, end, speed, max_seconds, overlay):
        out = self.segment_out()
        trim = f'trim=start={start}' + (f':end={end}' if end is not None else '')
        limit = f',trim=end={max_seconds}' if max_seconds else ''
        filt = (f'[0:v]{trim},setpts=(PTS-STARTPTS)/{speed}{limit},fps={self.fps},'
                f'scale={self.W}:{self.H}[v];[v][1:v]overlay=0:0')
        self.ffmpeg(['-i', str(self.media / f'{clip}.mp4'), '-i', str(overlay), '-filter_complex', filt,
                     *self.encode_opts(), str(out)])
        return out

    def compare(self, left, right, window_l, window_r, speed, max_seconds, overlay, crop=None):
        out = self.segment_out()
        hw, hh = self.W // 2, self.H // 2

        def part(idx, w):
            s, e = w
            trim = f'trim=start={s}' + (f':end={e}' if e is not None else '')
            # crop: 素材のうち使う範囲 [x, y, 幅, 高さ]（素材サイズに対する割合）。ロボット周辺を拡大する
            cr = f'crop=iw*{crop[2]}:ih*{crop[3]}:iw*{crop[0]}:ih*{crop[1]},' if crop else ''
            return f'[{idx}:v]{trim},setpts=(PTS-STARTPTS)/{speed},fps={self.fps},{cr}scale={hw}:{hh}[p{idx}]'

        dur = f',trim=end={max_seconds}' if max_seconds else ''
        filt = (f'{part(0, window_l)};{part(1, window_r)};'
                f'[p0][p1]hstack=inputs=2,tpad=stop_mode=clone:stop_duration=30{dur},'
                f'pad={self.W}:{self.H}:0:{(self.H - hh) // 2}:color=0x101622[v];[v][2:v]overlay=0:0')
        self.ffmpeg(['-i', str(self.media / f'{left}.mp4'), '-i', str(self.media / f'{right}.mp4'),
                     '-i', str(overlay), '-filter_complex', filt, *self.encode_opts(), str(out)])
        return out

    # ---- タイムライン ----
    def build(self):
        segs = []
        for item in self.cfg['timeline']:
            t = item['type']
            if t == 'card':
                segs.append(self.still(self.card(item['title'], item.get('subtitle'), item.get('lines', [])),
                                       item['duration']))
            elif t == 'image':
                segs.append(self.still(self.image_frame(item['image']), item['duration'],
                                       self.caption_layer(item['caption'])))
            elif t == 'clip':
                speed = item.get('speed', 1)
                overlay = self.caption_layer(item['caption'], speed)
                if 'trials' in item:
                    windows = self.trial_windows(item['clip'], item['trials'])
                else:
                    windows = [(item.get('start', 0.0), item.get('end'))]
                for s, e in windows:
                    segs.append(self.cut(item['clip'], s, e, speed, item.get('max_seconds'), overlay))
            elif t == 'compare':
                speed = item.get('speed', 1)
                overlay = self.combine(self.caption_layer(item['caption'], speed),
                                       self.label_layer([item['left_label'], item['right_label']]))
                k = item['trials'][0]
                segs.append(self.compare(item['left'], item['right'],
                                         self.trial_windows(item['left'], [k])[0],
                                         self.trial_windows(item['right'], [k])[0],
                                         speed, item.get('max_seconds'), overlay, item.get('crop')))
            else:
                raise ValueError(f'unknown timeline type: {t}')
        lst = self.work / 'list.txt'
        lst.write_text(''.join(f"file '{s}'\n" for s in segs))
        out = Path(os.path.expanduser(self.cfg['output']))
        out.parent.mkdir(parents=True, exist_ok=True)
        self.ffmpeg(['-f', 'concat', '-safe', '0', '-i', str(lst), '-c', 'copy', '-movflags', '+faststart', str(out)])
        return out

    def combine(self, a, b):
        img = Image.open(a).convert('RGBA')
        img.alpha_composite(Image.open(b).convert('RGBA'))
        return self.save(img)


def main():
    cfg = yaml.safe_load((ROOT / 'docs' / 'report' / 'video.yaml').read_text(encoding='utf-8'))
    work = Path(tempfile.mkdtemp(prefix='video_'))
    try:
        out = Composer(cfg, work).build()
    finally:
        shutil.rmtree(work, ignore_errors=True)
    dur = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', str(out)],
                         capture_output=True, text=True).stdout.strip()
    print(f'saved {out} ({float(dur):.1f} s)')


if __name__ == '__main__':
    main()

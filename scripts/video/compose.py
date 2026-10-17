"""docs/report/video.yaml のタイムラインに従って提出動画を編集する（コンテナ内で実行）.

python3 ~/final_task/scripts/video/compose.py
素材: record_clips.py が録画した <media_dir>/<clip>.mp4 と .events.csv
字幕・カードは Pillow で画像にして ffmpeg で重ねる（ナレーションなし）。
比較の場面には、上部にモデル表（video.yaml の models）を重ね、その場面で使うモデル（timeline の models）の列を強調する。
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
STRIP_H = 78           # 上部のモデル表の高さ [px]
STRIP_RIGHT = 170      # 右上の倍速表示の分だけ空ける幅 [px]


def validate(cfg):
    """timeline の models が video.yaml の models にあるかを確かめる."""
    ids = {m['id'] for m in cfg.get('models', [])}
    for item in cfg['timeline']:
        for mid in item.get('models', []):
            if mid not in ids:
                raise ValueError(f'unknown model {mid!r} in timeline item {item}')


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

    def image_frame(self, name, top=30):
        """画像を中央に置いた画面. top: 画像を置く範囲の上端（上部にモデル表を重ねるときは下げる）."""
        img = Image.new('RGB', (self.W, self.H), BG)
        src = Image.open(self.img_dir / name).convert('RGB')
        box_w, box_h = self.W - 80, self.H - 140 - top
        scale = min(box_w / src.width, box_h / src.height)
        src = src.resize((int(src.width * scale), int(src.height * scale)), Image.LANCZOS)
        img.paste(src, ((self.W - src.width) // 2, top + (box_h - src.height) // 2))
        return self.save(img)

    def fit_font(self, draw, text, size, width, bold=True):
        """幅に収まるまで文字を小さくしたフォント."""
        while size > 9 and draw.textlength(text, font=self.font(size, bold)) > width:
            size -= 1
        return self.font(size, bold)

    def strip_column_centers(self):
        n = len(self.cfg['models'])
        w = (self.W - STRIP_RIGHT - 16) / n
        return [16 + w * (i + 0.5) for i in range(n)]

    def model_strip_layer(self, active):
        """上部のモデル表（記号・呼び名・データ）. active のモデルの列だけ青地・太字で強調した透過 PNG."""
        img = Image.new('RGBA', (self.W, self.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        models = self.cfg['models']
        w = (self.W - STRIP_RIGHT - 16) / len(models)
        d.rectangle([0, 0, self.W - STRIP_RIGHT + 4, STRIP_H + 4], fill=(0, 0, 0, 150))
        for cx, m in zip(self.strip_column_centers(), models):
            on = m['id'] in active
            box = [cx - w / 2 + 3, 6, cx + w / 2 - 3, STRIP_H]
            if on:
                d.rounded_rectangle(box, radius=6, fill=ACCENT + (255,))
            color = 'white' if on else (150, 160, 178)
            d.text((cx, 10), m['id'], font=self.font(19 if on else 17, bold=on), fill=color, anchor='ma')
            for y, key in ((34, 'name'), (54, 'data')):
                d.text((cx, y), m[key], font=self.fit_font(d, m[key], 14, w - 10, bold=on), fill=color, anchor='ma')
        return self.save(img)

    def models_card(self):
        """モデル一覧の画面（記号・呼び名・入力・学習データ）."""
        img = Image.new('RGB', (self.W, self.H), BG)
        d = ImageDraw.Draw(img)
        d.text((self.W // 2, 60), self.cfg.get('models_title', 'モデル一覧'), font=self.font(40), fill='white', anchor='mm')
        d.line([self.W // 2 - 200, 96, self.W // 2 + 200, 96], fill=ACCENT, width=4)
        cols = [('記号', 'id', 110), ('呼び名', 'name', 300), ('方策の入力', 'input', 380), ('学習データ', 'data', 330)]
        x0 = (self.W - sum(c[2] for c in cols)) // 2
        rows = [{k: h for h, k, _ in cols}] + self.cfg['models']
        row_h = min(64, (self.H - 220) // len(rows))
        for r, m in enumerate(rows):
            y = 130 + r * row_h
            d.rectangle([x0, y, x0 + sum(c[2] for c in cols), y + row_h], fill=(30, 40, 60) if r == 0 else BG,
                        outline=(70, 82, 104))
            x = x0
            for _, key, cw in cols:
                text = m[key]
                f = self.fit_font(d, text, 26, cw - 20, bold=(r == 0 or key == 'id'))
                d.text((x + cw / 2, y + row_h / 2), text, font=f, fill='white', anchor='mm')
                x += cw
        note = self.cfg.get('models_note')
        if note:
            d.text((self.W // 2, 130 + len(rows) * row_h + 40), note, font=self.font(22, bold=False),
                   fill=(200, 210, 225), anchor='mm')
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
    def trial_windows(self, clip, trials, tail=1.0):
        """events.csv から試行ごとの [方策 ON の 0.5 秒前, 成功の tail 秒後（なければ OFF）] を返す."""
        rows = list(csv.DictReader(open(self.media / f'{clip}.mp4.events.csv')))
        ons = [i for i, r in enumerate(rows) if r['event'] == 'policy_on']
        out = []
        for k in trials:
            i = ons[k]
            start = max(0.0, float(rows[i]['time']) - 0.5)
            end = None
            for r in rows[i + 1:]:
                if r['event'] == 'success':
                    end = float(r['time']) + tail
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

    def compare(self, left, right, window_l, window_r, speed, max_seconds, overlay, crop=None, panel_height=None):
        out = self.segment_out()
        hw, hh = self.W // 2, panel_height or self.H // 2   # 左右それぞれの映像の大きさ

        def part(idx, w):
            s, e = w
            trim = f'trim=start={s}' + (f':end={e}' if e is not None else '')
            # crop: 素材のうち使う範囲 [x, y, 幅, 高さ]（素材サイズに対する割合）。ロボット周辺を拡大する
            cr = f'crop=iw*{crop[2]}:ih*{crop[3]}:iw*{crop[0]}:ih*{crop[1]},' if crop else ''
            # 縦横比を保って縮小し、余白で {hw}x{hh} にそろえる（俯瞰カメラの 4:3 の映像も歪めない）
            return (f'[{idx}:v]{trim},setpts=(PTS-STARTPTS)/{speed},fps={self.fps},{cr}'
                    f'scale={hw}:{hh}:force_original_aspect_ratio=decrease,'
                    f'pad={hw}:{hh}:(ow-iw)/2:(oh-ih)/2:color=0x101622[p{idx}]')

        dur = f',trim=end={max_seconds}' if max_seconds else ''
        filt = (f'{part(0, window_l)};{part(1, window_r)};'
                f'[p0][p1]hstack=inputs=2,tpad=stop_mode=clone:stop_duration=30{dur},'
                f'pad={self.W}:{self.H}:0:{(self.H - hh) // 2}:color=0x101622[v];[v][2:v]overlay=0:0')
        self.ffmpeg(['-i', str(self.media / f'{left}.mp4'), '-i', str(self.media / f'{right}.mp4'),
                     '-i', str(overlay), '-filter_complex', filt, *self.encode_opts(), str(out)])
        return out

    # ---- タイムライン ----
    def with_models(self, overlay, item):
        """item に models があれば、上部のモデル表を overlay に重ねる."""
        if not item.get('models'):
            return overlay
        strip = self.model_strip_layer(item['models'])
        return self.combine(overlay, strip) if overlay else strip

    def build(self):
        validate(self.cfg)
        segs = []
        for item in self.cfg['timeline']:
            t = item['type']
            if t == 'models':
                segs.append(self.still(self.models_card(), item['duration']))
            elif t == 'card':
                segs.append(self.still(self.card(item['title'], item.get('subtitle'), item.get('lines', [])),
                                       item['duration']))
            elif t == 'image':
                top = STRIP_H + 20 if item.get('models') else 30
                segs.append(self.still(self.image_frame(item['image'], top), item['duration'],
                                       self.with_models(self.caption_layer(item['caption']), item)))
            elif t == 'clip':
                speed = item.get('speed', 1)
                overlay = self.with_models(self.caption_layer(item['caption'], speed), item)
                if 'trials' in item:
                    windows = self.trial_windows(item['clip'], item['trials'])
                else:
                    windows = [(item.get('start', 0.0), item.get('end'))]
                for s, e in windows:
                    segs.append(self.cut(item['clip'], s, e, speed, item.get('max_seconds'), overlay))
            elif t == 'compare':
                speed = item.get('speed', 1)
                overlay = self.with_models(self.combine(self.caption_layer(item['caption'], speed),
                                                        self.label_layer([item['left_label'], item['right_label']])), item)
                k = item['trials'][0]
                segs.append(self.compare(item['left'], item['right'],
                                         self.trial_windows(item['left'], [k], item.get('tail', 1.0))[0],
                                         self.trial_windows(item['right'], [k], item.get('tail', 1.0))[0],
                                         speed, item.get('max_seconds'), overlay, item.get('crop'),
                                         item.get('panel_height')))
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

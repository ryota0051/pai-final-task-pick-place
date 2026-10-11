"""レポート（docs/report/report.md）を HTML 経由で PDF にする.

uv run --group report python scripts/report/build_report.py
設定: docs/report/build.yaml（出力ファイル名・使うブラウザ）
"""
import subprocess
from pathlib import Path

import markdown
import yaml

ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT / 'docs' / 'report'


def find_browser(candidates):
    for c in candidates:
        if Path(c).exists():
            return c
    raise FileNotFoundError(f'Chrome / Edge が見つかりません: {candidates}')


def main():
    cfg = yaml.safe_load((REPORT / 'build.yaml').read_text(encoding='utf-8'))
    body = markdown.markdown((REPORT / cfg['source']).read_text(encoding='utf-8'),
                             extensions=['tables', 'md_in_html'])
    css = (REPORT / cfg['style']).read_text(encoding='utf-8')
    html_path = REPORT / cfg['html']
    html_path.parent.mkdir(parents=True, exist_ok=True)
    # 画像は report.md からの相対パス。HTML を build/ に置くので base で docs/report を指す
    html_path.write_text(
        f'<!doctype html><html lang="ja"><head><meta charset="utf-8"><title>{cfg["title"]}</title>'
        f'<base href="{REPORT.as_uri()}/"><style>{css}</style></head><body>{body}</body></html>',
        encoding='utf-8')
    pdf_path = REPORT / cfg['pdf']
    browser = find_browser(cfg['chrome_candidates'])
    subprocess.run([browser, '--headless=new', '--disable-gpu', '--no-pdf-header-footer',
                    f'--print-to-pdf={pdf_path}', html_path.as_uri()],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'saved {pdf_path}')


if __name__ == '__main__':
    main()

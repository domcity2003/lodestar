"""Print the one-page report (out/report/index.html) to a single tall PDF page, with every image.

Uses headless Microsoft Edge (Chromium) so the PDF looks exactly like the web page, light theme.
Pass 1 measures the page height at a 1200 px window; pass 2 prints with the page sized to fit.

Run:  uv run python scripts/report_pdf.py
Writes: out/report/Lodestar_report.pdf
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REP = ROOT / "out" / "report"
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
WIDTH = 1200


def page(height=None):
    body = (REP / "index.html").read_text(encoding="utf-8").replace('loading="lazy"', 'loading="eager"')
    size = f"@page {{ size: {WIDTH}px {height}px; margin: 0; }}" if height else "@page { margin: 0; }"
    extra = f"""<style>{size}
html, body {{ -webkit-print-color-adjust: exact; print-color-adjust: exact; }}
.fill {{ transition: none !important; }}
</style>
<script>window.addEventListener('load', () => {{ document.body.dataset.h = Math.ceil(document.documentElement.scrollHeight); }});</script>"""
    return f'<!doctype html><html lang="en" data-theme="light"><head><meta charset="utf-8">' \
           f'<meta name="viewport" content="width=device-width, initial-scale=1">{body}{extra}</html>'


def edge(args):
    with tempfile.TemporaryDirectory() as prof:
        return subprocess.run([EDGE, "--headless=new", "--disable-gpu", f"--user-data-dir={prof}",
                               f"--window-size={WIDTH},1000", "--virtual-time-budget=8000",
                               "--run-all-compositor-stages-before-draw", *args],
                              capture_output=True, text=True, encoding="utf-8", timeout=180).stdout


if __name__ == "__main__":
    tmp = REP / "_print.html"
    try:
        tmp.write_text(page(), encoding="utf-8")
        dom = edge(["--dump-dom", tmp.as_uri()])
        h = int(re.search(r'data-h="(\d+)"', dom).group(1))
        tmp.write_text(page(h + 8), encoding="utf-8")
        out = REP / "Lodestar_report.pdf"
        edge(["--no-pdf-header-footer", f"--print-to-pdf={out}", tmp.as_uri()])
        print(f"page height {h} px -> {out} ({out.stat().st_size / 1e6:.1f} MB)")
    finally:
        tmp.unlink(missing_ok=True)

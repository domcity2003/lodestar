"""Print README.md to a paged PDF (US Letter), with every figure, using headless Chrome or Edge.

The README uses a small part of Markdown (headings, paragraphs, lists, tables, images, one code block), so it is
converted here without extra packages. Repository links become GitHub links so they work from the PDF.

Run:  uv run python scripts/readme_pdf.py
Writes: docs/Lodestar_README.pdf (gitignored: a local copy for sharing, not a second README in the repository)
"""
from pathlib import Path
import html
import re

from report_pdf import edge

ROOT = Path(__file__).resolve().parents[1]
REPO_URL = "https://github.com/domcity2003/lodestar/blob/main/"
OUT = ROOT / "docs" / "Lodestar_README.pdf"

CSS = """
@page { size: Letter; margin: 15mm 16mm 16mm 16mm;
        @bottom-right { content: counter(page) " / " counter(pages); font: 8pt 'Segoe UI', Arial, sans-serif;
                        color: #6b6b6b; } }
html { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
body { font: 10pt/1.45 'Segoe UI', Arial, sans-serif; color: #1f2328; margin: 0; }
h1 { font-size: 22pt; margin: 0 0 6pt; padding-bottom: 4pt; border-bottom: 1px solid #d1d9e0; }
h2 { font-size: 15pt; margin: 16pt 0 6pt; padding-bottom: 3pt; border-bottom: 1px solid #d1d9e0;
     break-after: avoid; }
h3 { font-size: 12pt; margin: 12pt 0 4pt; break-after: avoid; }
p { margin: 0 0 7pt; }
p.fig { margin: 8pt 0 3pt; text-align: center; break-inside: avoid; break-after: avoid; }
p.fig img { max-width: 100%; max-height: 225mm; }
p.caption { font-size: 8.5pt; color: #59636e; margin-bottom: 10pt; }
ul { margin: 0 0 8pt; padding-left: 16pt; }
li { margin-bottom: 3pt; }
table { border-collapse: collapse; margin: 4pt 0 9pt; font-size: 9pt; break-inside: avoid; }
th, td { border: 1px solid #d1d9e0; padding: 3pt 7pt; text-align: left; vertical-align: top; }
th { background: #f6f8fa; }
td.num, th.num { text-align: right; }
code { font: 8.5pt Consolas, 'Courier New', monospace; background: #eff2f5; padding: 0 2pt; border-radius: 3px; }
pre { background: #f6f8fa; border-radius: 4px; padding: 7pt 9pt; break-inside: avoid; white-space: pre-wrap;
      font-size: 7.6pt; line-height: 1.35; }
pre code { background: none; padding: 0; font-size: 7.6pt; }
section.cover { page: cover; }
@page cover { size: Letter landscape; }
section.cover p.fig img { max-height: 146mm; }
a { color: #0969da; text-decoration: none; }
"""


def link(href):
    """Repository-relative links point at GitHub; web links and anchors stay as they are."""
    return href if re.match(r"[a-z]+:|#", href) else REPO_URL + href


def inline(text):
    codes = []

    def keep(m):
        codes.append(f"<code>{html.escape(m.group(1))}</code>")
        return f"\x00{len(codes) - 1}\x00"

    t = re.sub(r"`([^`]+)`", keep, text)
    t = html.escape(t, quote=False)
    t = re.sub(r"!\[([^\]]*)\]\(([^)]+)\)",
               lambda m: f'<img src="{(ROOT / m.group(2)).as_uri()}" alt="{html.escape(m.group(1))}">', t)
    t = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", lambda m: f'<a href="{link(m.group(2))}">{m.group(1)}</a>', t)
    t = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"\*([^*]+)\*", r"<em>\1</em>", t)
    return re.sub(r"\x00(\d+)\x00", lambda m: codes[int(m.group(1))], t)


def table(rows):
    cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
    head, body = cells[0], cells[2:]
    num = [all(re.fullmatch(r"[*\d.,−+\- ]+", r[i]) for r in body) for i in range(len(head))]
    th = "".join(f'<th class="{"num" if n else ""}">{inline(c)}</th>' for c, n in zip(head, num))
    trs = "".join("<tr>" + "".join(f'<td class="{"num" if n else ""}">{inline(c)}</td>' for c, n in zip(r, num))
                  + "</tr>" for r in body)
    return f"<table><thead><tr>{th}</tr></thead><tbody>{trs}</tbody></table>"


def convert(md):
    out, lines, i = [], md.splitlines(), 0
    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
        elif line.startswith("```"):
            j = lines.index("```", i + 1) if "```" in lines[i + 1:] else len(lines)
            out.append(f"<pre><code>{html.escape(chr(10).join(lines[i + 1:j]))}</code></pre>")
            i = j + 1
        elif m := re.match(r"(#{1,3}) (.*)", line):
            n = len(m.group(1))
            out.append(f"<h{n}>{inline(m.group(2))}</h{n}>")
            i += 1
        elif line.startswith("|"):
            j = i
            while j < len(lines) and lines[j].startswith("|"):
                j += 1
            out.append(table(lines[i:j]))
            i = j
        elif line.startswith("- "):
            items = []
            while i < len(lines) and (lines[i].startswith("- ") or lines[i].startswith("  ")):
                if lines[i].startswith("- "):
                    items.append(lines[i][2:].strip())
                else:
                    items[-1] += " " + lines[i].strip()
                i += 1
            out.append("<ul>" + "".join(f"<li>{inline(t)}</li>" for t in items) + "</ul>")
        else:
            para = []
            while i < len(lines) and lines[i].strip() and not re.match(r"#|\||- |```", lines[i]):
                para.append(lines[i].strip())
                i += 1
            text = " ".join(para)
            if re.fullmatch(r"\[?!\[.*", text):                     # an image line, maybe wrapped in a link
                cls = "fig"
            elif re.fullmatch(r"\*[^*].*[^*]\*", text):              # a whole-paragraph italic line: a caption
                cls = "caption"
            else:
                cls = ""
            out.append(f'<p class="{cls}">{inline(text)}</p>')
    return out


if __name__ == "__main__":
    md = (ROOT / "README.md").read_text(encoding="utf-8").replace("Click for the vector PDF.",
                                                                    "The drawing links to the vector PDF.")
    blocks = convert(md)
    cut = next(i for i, b in enumerate(blocks) if 'class="caption"' in b) + 1   # title, intro, GA and its caption
    body = '<section class="cover">' + "\n".join(blocks[:cut]) + "</section>\n" + "\n".join(blocks[cut:])
    page = (f'<!doctype html><html lang="en"><head><meta charset="utf-8"><title>Lodestar</title>'
            f"<style>{CSS}</style></head><body>{body}</body></html>")
    tmp = ROOT / "_readme_print.html"
    try:
        tmp.write_text(page, encoding="utf-8")
        edge(["--no-pdf-header-footer", f"--print-to-pdf={OUT}", tmp.as_uri()])
        print(f"{OUT} ({OUT.stat().st_size / 1e6:.1f} MB)")
    finally:
        tmp.unlink(missing_ok=True)

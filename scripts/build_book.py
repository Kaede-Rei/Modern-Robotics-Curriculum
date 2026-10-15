#!/usr/bin/env python3
"""Build the single-file, file://-compatible offline curriculum reader."""
from __future__ import annotations
import argparse
import base64
import hashlib
import html as html_lib
import json
import mimetypes
from pathlib import Path
import re
import subprocess
import sys
from bs4 import BeautifulSoup
import yaml

ROOT = Path(__file__).resolve().parent.parent
CATALOG = ROOT / 'book/catalog.yml'
TEMPLATE = ROOT / 'reader/template.html'
OUTPUT = ROOT / 'dist/index.html'
LABS = json.loads((ROOT / 'reader/learning-labs.json').read_text(encoding='utf-8'))
LAB_BY_ID = {lab['id'].upper(): lab for lab in LABS}


def learning_lab_html(lab: dict) -> str:
    controls = []
    for name, title, unit, low, high, value, step in lab['controls']:
        name, title, unit = map(html_lib.escape, (name, title, unit))
        controls.append(
            '<label class="lab-control"><span><span>' + title + '</span><output data-value="' + name + '">' + str(value) +
            (' ' + unit if unit else '') + '</output></span><input type="range" name="' + name + '" data-unit="' + unit +
            '" min="' + str(low) + '" max="' + str(high) + '" value="' + str(value) + '" step="' + str(step) + '"></label>'
        )
    return ('<section class="learning-lab" aria-label="' + html_lib.escape(lab['title']) + '" data-learning-lab="' + html_lib.escape(lab['id']) + '">' +
            '<span class="lab-eyebrow">INTERACTIVE CONCEPT</span><h3>' + html_lib.escape(lab['title']) + '</h3>' +
            '<p class="lab-intro">' + html_lib.escape(lab['description']) + '</p>' +
            '<canvas width="650" height="262" role="img" aria-label="' + html_lib.escape(lab['title']) + '动态图"></canvas>' +
            '<div class="lab-controls">' + ''.join(controls) + '</div>' +
            '<p class="lab-summary" data-lab-summary aria-live="polite"></p>' +
            '<p class="lab-caveat">教学示意使用简化模型，仅用于观察参数与公式关系，不作为真机性能或安全依据</p></section>')


def convert_markdown(path: Path, *, chapter: str) -> dict:
    result = subprocess.run(
        ['pandoc', '--from=markdown+tex_math_dollars+raw_html+fenced_divs',
         '--to=html5', '--mathml', '--wrap=none', str(path)],
        capture_output=True, text=True, encoding='utf-8', check=True,
    )
    if result.stderr.strip():
        raise ValueError(f'Pandoc reported a conversion warning in {path}: {result.stderr.strip()[:900]}')
    html = result.stdout
    if chapter == 'M03':
        token = '<!-- INTERACTIVE:ARM2R -->'
        if token not in html:
            raise ValueError('M03 is missing the arm2r demo marker')
        html = html.replace(token, (ROOT / 'reader/arm2r-demo.html').read_text(encoding='utf-8'))
    for lab in LABS:
        marker = '<!-- INTERACTIVE:' + lab['id'].upper() + ' -->'
        if lab['chapter'] == chapter:
            if html.count(marker) != 1:
                raise ValueError(f'Expected one lab marker {marker} in {chapter}')
            html = html.replace(marker, learning_lab_html(lab))
        elif marker in html:
            raise ValueError(f'Unexpected lab marker {marker} in {chapter}')
    soup = BeautifulSoup(html, 'html.parser')
    # Never leave dependencies on files adjacent to the generated index.html.
    for img in soup.find_all('img'):
        src = img.get('src', '')
        if src.startswith('data:'):
            continue
        if src.startswith('http:') or src.startswith('https:'):
            raise ValueError(f'Network image prohibited in offline textbook: {src}')
        image = (path.parent / src).resolve()
        if not image.is_file() or not image.is_relative_to(ROOT):
            raise FileNotFoundError(f'Missing/out-of-tree image {src} in {path}')
        mime = mimetypes.guess_type(image.name)[0] or 'application/octet-stream'
        img['src'] = f'data:{mime};base64,' + base64.b64encode(image.read_bytes()).decode('ascii')
    for tag in soup.find_all(['script', 'iframe']):
        if tag.get('src') or tag.name == 'iframe':
            raise ValueError(f'External scripts/iframes not allowed in {path}')
    # Large display MathML equations and wide tables must scroll INSIDE the article on mobile.
    for math in soup.find_all('math', attrs={'display': 'block'}):
        parent = math.parent
        if parent is not None and parent.name == 'p' and len(parent.find_all(recursive=False)) == 1:
            parent['class'] = list(parent.get('class', [])) + ['math-scroll']
    for table in list(soup.find_all('table')):
        wrapper = soup.new_tag('div', attrs={'class': 'table-scroll'})
        table.wrap(wrapper)
    text = re.sub(r'\s+', ' ', soup.get_text(' ', strip=True))
    return {'html': str(soup), 'text': text}


def validate_catalog(cat: dict) -> tuple[list[dict], dict]:
    parts = cat.get('parts', [])
    assert len(parts) == 6, f'expected six parts, got {len(parts)}'
    all_chapters = [c for p in parts for c in p['chapters']]
    assert len(all_chapters) == 49, f'expected 49 chapters, got {len(all_chapters)}'
    ids = [c['id'] for c in all_chapters]
    assert len(set(ids)) == 49, 'duplicate chapter IDs'
    mapping = {c['id']: c for c in all_chapters}
    for c in all_chapters:
        assert c['status'] in cat['status_values']
        for pre in c['prerequisites']:
            assert pre in mapping and pre != c['id'], f'bad prerequisite {pre}'
    graph = {c['id']: c['prerequisites'] for c in all_chapters}
    visited, visiting = set(), set()
    def dfs(node):
        if node in visited:return
        if node in visiting:raise ValueError('Prerequisite cycle at ' + node)
        visiting.add(node)
        for pre in graph[node]:dfs(pre)
        visiting.remove(node);visited.add(node)
    for node in ids:dfs(node)
    return all_chapters, mapping


def build():
    cat = yaml.safe_load(CATALOG.read_text(encoding='utf-8'))
    chapters, _ = validate_catalog(cat)
    pages = {'notation': convert_markdown(ROOT / 'book/notation.md', chapter='notation')}
    for c in chapters:
        p = (ROOT / c['path']).resolve()
        if not p.is_relative_to(ROOT):
            raise ValueError('Chapter outside of repository root: ' + c['path'])
        if c['status'] == 'planned':
            if p.is_file():
                raise ValueError(f'Planned chapter has published content: {c["id"]}')
            continue
        if not p.is_file():
            raise FileNotFoundError(f'Published chapter missing: {p}')
        pages[c['id']] = convert_markdown(p, chapter=c['id'])
    payload = {
        'project': cat['project'], 'title': cat['title'],
        'parts': cat['parts'], 'pages': pages,
    }
    raw_json = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
    # Escaping < prevents attacker-supplied prose from breaking out of JSON script tag.
    raw_json = raw_json.replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    template = TEMPLATE.read_text(encoding='utf-8')
    style = (ROOT / 'reader/style.css').read_text(encoding='utf-8') + '\n' + (ROOT / 'reader/learning-labs.css').read_text(encoding='utf-8')
    script = (ROOT / 'reader/learning-labs.js').read_text(encoding='utf-8') + '\n' + (ROOT / 'reader/reader.js').read_text(encoding='utf-8')
    for marker in ('/*INLINE_STYLE*/', '/*INLINE_SCRIPT*/', '/*INLINE_DATA*/'):
        if template.count(marker) != 1:
            raise ValueError('Template marker missing or repeated: ' + marker)
    html = template.replace('/*INLINE_STYLE*/', style).replace('/*INLINE_SCRIPT*/', script).replace('/*INLINE_DATA*/', raw_json)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(html, encoding='utf-8')
    n = len(pages)-1
    print(f'BUILT {OUTPUT.relative_to(ROOT)} | {len(chapters)} catalog chapters | {n} published | {OUTPUT.stat().st_size:,} bytes | sha256={hashlib.sha256(OUTPUT.read_bytes()).hexdigest()}')
    return 0


if __name__ == '__main__':
    try:sys.exit(build())
    except (AssertionError, OSError, ValueError, subprocess.CalledProcessError) as e:
        print('BUILD FAILED:', str(e), file=sys.stderr)
        sys.exit(1)

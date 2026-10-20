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
from image_assets import read_png

ROOT = Path(__file__).resolve().parent.parent
CATALOG = ROOT / 'book/catalog.yml'
TEMPLATE = ROOT / 'reader/template.html'
OUTPUT = ROOT / 'dist/index.html'
DETAIL_CATALOG = ROOT / 'book/explanations/catalog.yml'
LABS = json.loads((ROOT / 'reader/learning-labs.json').read_text(encoding='utf-8'))
WEB_PNG_ASSETS = json.loads((ROOT / 'assets/figures/web-png-sources.json').read_text(encoding='utf-8'))['assets']
WEB_PNG_BY_URL = {item['url']: item for item in WEB_PNG_ASSETS.values()}
WEB_PNG_BY_PATH = {(ROOT / item['local']).resolve(): item for item in WEB_PNG_ASSETS.values()}
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
    if chapter == 'explain:F02-core2-rank-nullspace':
        demos = {
            'RANK_MOTORS': 'rank-demo-motors.html',
            'RANK_SINGULARITY': 'rank-demo-singularity.html',
            'RANK_REDUNDANCY': 'rank-demo-redundancy.html',
        }
        for name, fragment in demos.items():
            marker = '<!-- INTERACTIVE:' + name + ' -->'
            if html.count(marker) != 1:
                raise ValueError('Missing or repeated F02 lesson marker: ' + marker)
            html = html.replace(marker, (ROOT / 'reader' / fragment).read_text(encoding='utf-8'))
    if chapter == 'explain:F02-core3-inner-product-projection':
        for name, fragment in {'PROJECTION_WORK': 'projection-demo-work.html',
                               'PROJECTION_FIT': 'projection-demo-fit.html'}.items():
            marker = '<!-- INTERACTIVE:' + name + ' -->'
            if html.count(marker) != 1:
                raise ValueError('Missing or repeated projection lesson marker: ' + marker)
            html = html.replace(marker, (ROOT / 'reader' / fragment).read_text(encoding='utf-8'))
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
        if src.startswith(('http:', 'https:', '//')):
            raise ValueError(f'Remote image is not allowed in offline source: {src}; download and use a local path')
        image = (path.parent / src).resolve()
        if not image.is_file() or not image.is_relative_to(ROOT):
            raise FileNotFoundError(f'Missing/out-of-tree image {src} in {path}')
        mime = mimetypes.guess_type(image.name)[0] or 'application/octet-stream'
        contents = read_png(image, WEB_PNG_BY_PATH.get(image)) if mime == 'image/png' else image.read_bytes()
        img['src'] = f'data:{mime};base64,' + base64.b64encode(contents).decode('ascii')
        img['data-image-source'] = 'embedded'
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
    # Supplemental explanations are independent pages, not additional curriculum chapters
    detail_specs = yaml.safe_load(DETAIL_CATALOG.read_text(encoding='utf-8'))['explanations']
    known_ids = {item['id'] for item in detail_specs}
    if len(known_ids) != len(detail_specs):
        raise ValueError('Duplicate supplemental explanation ID')
    chapter_by_id = {c['id']: c for c in chapters}
    for item in detail_specs:
        slug = item['id']
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9-]*', slug):
            raise ValueError('Invalid supplemental explanation ID: ' + slug)
        chapter = item['chapter']
        if chapter not in chapter_by_id:
            raise ValueError('Unknown source chapter: ' + chapter)
        origin = pages.get(chapter)
        if not origin:
            raise ValueError('Source chapter not published: ' + chapter)
        source = BeautifulSoup(origin['html'], 'html.parser')
        if not source.find(id=item['anchor']):
            raise ValueError('Source section anchor does not exist: ' + str(item['anchor']))
        destination = '#/explain/' + slug
        if not any(a.get('href') == destination for a in source.find_all('a')):
            raise ValueError('Source chapter missing detail link: ' + destination)
        detail_file = (ROOT / item['path']).resolve()
        if not detail_file.is_relative_to(ROOT) or not detail_file.is_file():
            raise FileNotFoundError('Missing supplemental explanation: ' + item['path'])
        pages['explain:' + slug] = convert_markdown(detail_file, chapter='explain:' + slug)
    payload = {
        'project': cat['project'], 'title': cat['title'],
        'parts': cat['parts'], 'pages': pages, 'explanations': detail_specs,
    }
    raw_json = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
    # Escaping < prevents attacker-supplied prose from breaking out of JSON script tag.
    raw_json = raw_json.replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    template = TEMPLATE.read_text(encoding='utf-8')
    style = '\n'.join((ROOT / 'reader' / name).read_text(encoding='utf-8') for name in ('style.css', 'learning-labs.css', 'rank-lesson.css', 'projection-lesson.css'))
    script = '\n'.join((ROOT / 'reader' / name).read_text(encoding='utf-8') for name in ('learning-labs.js', 'rank-lesson.js', 'projection-lesson.js', 'reader.js'))
    for marker in ('/*INLINE_STYLE*/', '/*INLINE_SCRIPT*/', '/*INLINE_DATA*/'):
        if template.count(marker) != 1:
            raise ValueError('Template marker missing or repeated: ' + marker)
    html = template.replace('/*INLINE_STYLE*/', style).replace('/*INLINE_SCRIPT*/', script).replace('/*INLINE_DATA*/', raw_json)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(html, encoding='utf-8')
    n = len(chapters) - sum(c['status']=='planned' for c in chapters)
    print(f'BUILT {OUTPUT.relative_to(ROOT)} | {len(chapters)} catalog chapters | {n} published | {OUTPUT.stat().st_size:,} bytes | sha256={hashlib.sha256(OUTPUT.read_bytes()).hexdigest()}')
    return 0


if __name__ == '__main__':
    try:sys.exit(build())
    except (AssertionError, OSError, ValueError, subprocess.CalledProcessError) as e:
        print('BUILD FAILED:', str(e), file=sys.stderr)
        sys.exit(1)

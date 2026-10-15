#!/usr/bin/env python3
"""Verify the curriculum, its embedded resources, and the offline reader"""
from __future__ import annotations
import hashlib
from html.parser import HTMLParser
import json
import re
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
cat = yaml.safe_load((ROOT / 'book/catalog.yml').read_text(encoding='utf-8'))
parts = cat['parts']
assert len(parts) == 6
chapters = [c for part in parts for c in part['chapters']]
assert len(chapters) == 49
assert len({c['id'] for c in chapters}) == 49
assert all(c['status'] != 'planned' for c in chapters)
for chapter in chapters:
    file = (ROOT / chapter['path']).resolve()
    assert file.is_relative_to(ROOT) and file.is_file() and file.stat().st_size > 800
    assert chapter['status'] in ('review', 'verified')

bib = (ROOT / 'references/references.bib').read_text(encoding='utf-8')
bib_ids = re.findall(r'@\w+\{([^,\s]+),', bib)
assert len(bib_ids) == len(set(bib_ids)), 'duplicate bibliography entry'

lab_specs = json.loads((ROOT / 'reader/learning-labs.json').read_text(encoding='utf-8'))
assert len(lab_specs) == 12 and len({x['id'] for x in lab_specs}) == 12
source_files = [ROOT / 'README.md'] + list((ROOT / 'book').rglob('*.md'))
source_files += list((ROOT / 'reader').glob('*.html')) + list((ROOT / 'reader').glob('*.js'))
for file in source_files:
    source = file.read_text(encoding='utf-8')
    assert '。' not in source, f'Chinese full stop in {file}'
    assert not re.search(r'\bP[1-7]\s+(?:实验|工程|阶段|的)', source), f'phase metadata in {file}'

html_path = ROOT / 'dist/index.html'
html = html_path.read_text(encoding='utf-8')
assert '。' not in html
assert 'generatedAt' not in html
for pattern in (r'<script\b[^>]*\bsrc=', r'<link\b[^>]*\bhref=', r'<iframe\b', r'<object\b', r'<embed\b'):
    assert not re.search(pattern, html, re.I), f'External resource syntax {pattern}'
match = re.search(r'<script id="book-payload" type="application/json">(.*?)</script>', html, re.S)
assert match, 'Missing embedded book data'
payload = json.loads(match.group(1))
assert len(payload['pages']) == 50

class ArticleScanner(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.h1 = 0
        self.math = 0
        self.images = 0
        self.labs = []
    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'h1': self.h1 += 1
        if tag == 'math': self.math += 1
        if tag == 'img':
            self.images += 1
            assert (attrs.get('src') or '').startswith('data:image/'), attrs.get('src')
        if tag == 'a':
            u = attrs.get('href', '')
            assert u.startswith(('https://','http://','#','mailto:')), u
        if 'data-learning-lab' in attrs:
            self.labs.append(attrs['data-learning-lab'])

math_total = images_total = 0
all_labs = {}
for chapter in chapters:
    page = payload['pages'].get(chapter['id'])
    assert page and len(page['text']) > 120, chapter['id']
    scan = ArticleScanner()
    scan.feed(page['html'])
    assert scan.h1 >= 1, chapter['id']
    math_total += page['html'].count('<math')
    images_total += scan.images
    for lab in scan.labs:
        all_labs[lab] = chapter['id']
assert all_labs == {x['id']:x['chapter'] for x in lab_specs}, all_labs
math_total += payload['pages']['notation']['html'].count('<math')
assert images_total >= 31, images_total
report = {
    'chapters':len(chapters), 'parts':len(parts),
    'interactive_labs':len(all_labs), 'bibliography_entries':len(bib_ids),
    'mathml_elements':math_total,'embedded_images':images_total,
    'html_bytes':html_path.stat().st_size,
    'html_sha256':hashlib.sha256(html_path.read_bytes()).hexdigest()
}
print('PASS CURRICULUM INTEGRITY',json.dumps(report,ensure_ascii=False))

#!/usr/bin/env python3
"""Verify the curriculum, its embedded resources, and the offline reader"""
from __future__ import annotations
import hashlib
import base64
from bs4 import BeautifulSoup
from image_assets import read_png, validate_png
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

manifest = json.loads((ROOT / 'assets/figures/web-png-sources.json').read_text(encoding='utf-8'))
assets = manifest['assets']
for key, asset in assets.items():
    image = (ROOT / asset['local']).resolve()
    assert image.is_relative_to(ROOT), key
    assert all(asset.get(field) for field in ('sha256', 'bytes', 'width', 'height', 'author', 'license', 'license_url', 'download_url')), key
    assert '参见' not in asset['license'], key
    read_png(image, asset)
for file in source_files:
    source = file.read_text(encoding='utf-8')
    for img in BeautifulSoup(source, 'html.parser').find_all('img'):
        assert not img.get('src', '').startswith(('http:', 'https:', '//')), f'Online image in {file}'
    assert not re.search(r'!\[[^\]]*\]\(https?://', source), f'Online Markdown image in {file}'
html_path = ROOT / 'dist/index.html'
html = html_path.read_text(encoding='utf-8')
assert '。' not in html
assert 'generatedAt' not in html
for pattern in (r'<script\b[^>]*\bsrc=', r'<link\b[^>]*\bhref=', r'<iframe\b', r'<object\b', r'<embed\b'):
    assert not re.search(pattern, html, re.I), f'External resource syntax {pattern}'
match = re.search(r'<script id="book-payload" type="application/json">(.*?)</script>', html, re.S)
assert match, 'Missing embedded book data'
payload = json.loads(match.group(1))
details = yaml.safe_load((ROOT / 'book/explanations/catalog.yml').read_text(encoding='utf-8'))['explanations']
assert len(payload['pages']) == 50 + len(details)
assert payload['explanations'] == details
assert len({item['id'] for item in details}) == len(details)
for item in details:
    assert item['chapter'] in {c['id'] for c in chapters}
    assert payload['pages'].get('explain:' + item['id'])
    doc = payload['pages'][item['chapter']]['html']
    assert '#/explain/' + item['id'] in doc
    assert item['anchor'] in doc
projection_page = BeautifulSoup(payload['pages']['explain:F02-core3-inner-product-projection']['html'], 'html.parser')
assert {lab['data-projection-lab'] for lab in projection_page.select('[data-projection-lab]')} == {'work', 'fit'}
assert '<!-- INTERACTIVE:' not in str(projection_page), 'Unexpanded lesson marker'
assert '部分教学详细讲解' in html
assert 'data-target="explanations"' in html

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
            src = attrs.get('src') or ''
            assert src.startswith('data:image/'), f'Non-embedded image: {src[:120]}'
            assert attrs.get('alt'), 'Image missing alternative text'
            if src.startswith('data:image/png;base64,'):
                validate_png(base64.b64decode(src.split(',', 1)[1], validate=True))
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
assert images_total >= 49, images_total
for page_id, page in payload['pages'].items():
    scan = ArticleScanner()
    scan.feed(page['html'])
    if page_id != 'notation':
        assert scan.images > 0, f'Unillustrated page: {page_id}'
# CSS can reference inlined images or internal SVG fragments, never network resources
assert not re.search(r'url\([\s\"\']*(?:https?:|//)', html, re.I)
assert 'data-image-source=\"remote\"' not in html
assert not re.search(r'\b(?:srcset|poster)\s*=', html, re.I)
report = {
    'chapters':len(chapters), 'parts':len(parts),
    'interactive_labs':len(all_labs), 'bibliography_entries':len(bib_ids),
    'mathml_elements':sum(page['html'].count('<math') for page in payload['pages'].values()),
    'chapter_images':images_total,
    'embedded_images':sum(len(BeautifulSoup(page['html'], 'html.parser').find_all('img')) for page in payload['pages'].values()),
    'offline_png_assets':len(assets),
    'remote_png_references':sum(page['html'].count('data-image-source="remote"') for page in payload['pages'].values()),
    'html_bytes':html_path.stat().st_size,
    'html_sha256':hashlib.sha256(html_path.read_bytes()).hexdigest()
}
print('PASS CURRICULUM INTEGRITY',json.dumps(report,ensure_ascii=False))

#!/usr/bin/env python3
"""Check the published curriculum and offline reader"""
from __future__ import annotations
import hashlib
import json
import re
from pathlib import Path
from bs4 import BeautifulSoup
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

# Reader-visible prose must not contain Chinese full stops or internal phase labels
source_files = [ROOT / 'README.md'] + list((ROOT / 'book').rglob('*.md'))
source_files += [ROOT / 'reader/reader.js', ROOT / 'reader/template.html', ROOT / 'reader/arm2r-demo.html']
for file in source_files:
    source = file.read_text(encoding='utf-8')
    assert chr(0x3002) not in source, f'Chinese full stop in {file}'
    assert not re.search(r'\bP[1-7]\s+(?:实验|工程|阶段|的)', source), f'phase metadata in {file}'

html = (ROOT / 'dist/index.html').read_text(encoding='utf-8')
assert chr(0x3002) not in html
assert 'generatedAt' not in html
soup = BeautifulSoup(html, 'html.parser')
assert not soup.select('script[src],link[href],iframe,object,embed,video[src],audio[src]')
assert not soup.find_all('img', src=lambda s: bool(s) and not s.startswith('data:'))
payload = json.loads(soup.find(id='book-payload').string)
assert len(payload['pages']) == 50
for chapter in chapters:
    page = payload['pages'].get(chapter['id'])
    assert page is not None and len(page['text']) > 120
    doc = BeautifulSoup(page['html'], 'html.parser')
    assert doc.find('h1') is not None
    for img in doc.find_all('img'):
        assert img.get('src', '').startswith('data:image/')
    for a in doc.find_all('a'):
        u = a.get('href', '')
        assert u.startswith(('https://','http://','#','mailto:')),(chapter['id'], u)

report = {
    'chapters': len(chapters),
    'parts': len(parts),
    'bibliography_entries': len(bib_ids),
    'mathml_elements': sum(len(BeautifulSoup(p['html'],'html.parser').find_all('math')) for p in payload['pages'].values()),
    'embedded_images': sum(len(BeautifulSoup(p['html'],'html.parser').find_all('img')) for p in payload['pages'].values()),
    'html_bytes': (ROOT / 'dist/index.html').stat().st_size,
    'html_sha256': hashlib.sha256((ROOT / 'dist/index.html').read_bytes()).hexdigest(),
}
print('PASS CURRICULUM INTEGRITY', json.dumps(report, ensure_ascii=False))

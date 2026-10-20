#!/usr/bin/env python3
"""Optional browser regression: copy the HTML alone, block network, visit every page.

Requires playwright and its Chromium browser (see README).
"""
import json
import argparse
import shutil
import tempfile
from pathlib import Path
from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--browser-channel', help='Optional installed browser channel, e.g. msedge')
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix='mrc-offline-') as directory:
    single = Path(directory) / 'index.html'
    shutil.copyfile(ROOT / 'dist/index.html', single)
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(channel=args.browser_channel)
        context = browser.new_context(offline=True, viewport={'width': 1280, 'height': 900})
        page = context.new_page()
        errors, requests = [], []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.on('request', lambda request: requests.append(request.url) if request.url.startswith(('http:', 'https:')) else None)
        page.goto(single.as_uri())
        titles = page.evaluate("""Object.fromEntries(Object.entries(JSON.parse(document.getElementById('book-payload').textContent).pages).map(([id,p]) => [id, new DOMParser().parseFromString(p.html, 'text/html').querySelector('h1').textContent]))""")
        expected_images = page.evaluate("""Object.fromEntries(Object.entries(JSON.parse(document.getElementById('book-payload').textContent).pages).map(([id,p]) => [id, new DOMParser().parseFromString(p.html, 'text/html').querySelectorAll('img').length]))""")
        ids = list(titles)
        image_count = lab_count = 0
        for page_id in ids:
            route = page_id.replace('explain:', 'explain/')
            page.evaluate('(route) => { location.hash = "#/" + route; }', route)
            expect(page.locator('#page-content h1')).to_have_text(titles[page_id])
            loaded = page.evaluate('''async () => {
                const imgs = [...document.querySelectorAll('#page-content img')];
                await Promise.all(imgs.map(i => { i.loading = 'eager'; return i.decode(); }));
                return imgs.every(i => i.naturalWidth > 0 && i.src.startsWith('data:image/')) ? imgs.length : -1;
            }''')
            assert loaded == expected_images[page_id], (page_id, loaded, expected_images[page_id])
            image_count += loaded
            for lab in page.locator('[data-learning-lab]').all():
                before = lab.locator('canvas').evaluate('e => e.toDataURL()')
                slider = lab.locator('input[type=range]').first
                slider.evaluate("e => { e.value = e.value === e.max ? e.min : e.max; e.dispatchEvent(new Event('input', {bubbles:true})); }")
                assert lab.locator('canvas').evaluate('e => e.toDataURL()') != before, page_id
                name = slider.get_attribute('name')
                assert slider.input_value() in lab.locator(f'[data-value="{name}"]').inner_text(), page_id
                lab_count += 1
        assert lab_count == 12, lab_count
        page.goto(single.as_uri() + '#/M03')
        page.locator('[data-q2]').evaluate("e => {e.value=0; e.dispatchEvent(new Event('input', {bubbles:true}));}")
        assert abs(float(page.locator('[data-det-out]').inner_text())) < 1e-8
        page.goto(single.as_uri() + '#/explain/F02-core2-rank-nullspace')
        page.locator('[data-motor-preset=add]').click()
        assert page.locator('[data-motor-total]').inner_text() == '+4'
        page.locator('[data-sing-preset="45"]').click()
        assert page.locator('[data-sing-rank]').inner_text() == '2'
        before = page.locator('[data-red-angle]').inner_text()
        page.locator('[data-red-input]').evaluate("e => { e.value=e.max; e.dispatchEvent(new Event('input', {bubbles:true})); }")
        assert page.locator('[data-red-angle]').inner_text() != before
        page.goto(single.as_uri() + '#/F02')
        page.locator('a[href="#/explain/F02-core3-inner-product-projection"]').click()
        expect(page.locator('#page-content h1')).to_have_text(titles['explain:F02-core3-inner-product-projection'])
        for angle, expected in ((0, '20.000'), (90, '0.000'), (180, '-20.000')):
            page.locator(f'[data-work-angle="{angle}"]').click()
            assert f'W = {expected} J' in page.locator('[data-projection-out="work"]').inner_text()
        page.locator('[data-fit-best]').click()
        assert page.locator('[name=t]').input_value() == '0.8'
        assert 'f = 0.900' in page.locator('[data-projection-out="fit"]').inner_text()
        assert 'aᵀr = 0.000' in page.locator('[data-projection-out="normal"]').inner_text()
        page.locator('[name=t]').evaluate("e => {e.value=1.8; e.dispatchEvent(new Event('input', {bubbles:true}));}")
        assert 'f = 3.400' in page.locator('[data-projection-out="fit"]').inner_text()
        for bx, by, expected in ((2, 1, '1'), (-2, -1, '-1'), (0, 0, '0')):
            for name, value in (('bx', bx), ('by', by)):
                page.locator(f'[name={name}]').evaluate("(e,v) => {e.value=v; e.dispatchEvent(new Event('input', {bubbles:true}));}", value)
            page.locator('[data-fit-best]').click()
            assert page.locator('[name=t]').input_value() == expected
            assert '残差距离 = 0.000' in page.locator('[data-projection-out="fit"]').inner_text()
        page.locator('[data-fit-reset]').click()
        assert page.locator('[name=bx]').input_value() == '1'
        assert page.locator('[name=t]').input_value() == '0'
        back = page.locator('.detail-back a').first
        back.click()
        assert '核心-3.' in page.url or '%E6%A0%B8%E5%BF%83-3.' in page.url
        for width in (1280, 390):
            page.set_viewport_size({'width': width, 'height': 900})
            page.goto(single.as_uri() + '#/explain/F02-core3-inner-product-projection')
            page.wait_for_selector('[data-projection-lab]')
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 1'), width
        assert not errors, errors
        assert not requests, requests
        browser.close()
        print('PASS OFFLINE BROWSER', json.dumps({'pages':len(ids), 'decoded_images':image_count, 'interactive_labs':lab_count, 'extra_demos':6, 'network_requests':len(requests)}))

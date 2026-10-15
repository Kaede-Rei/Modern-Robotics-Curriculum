#!/usr/bin/env python3
"""Fetch genuine, credited internet PNGs for fully offline curriculum builds.

Downloaded assets are versioned with the textbook and referenced by local paths.
No AI-generated images, conversions, or screenshot rendering take place here.
"""
from __future__ import annotations

import argparse
import os
import tempfile
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'assets/figures/web-png-sources.json'
from image_assets import MAX_BYTES, read_png, validate_png


def download(url: str, target: Path, *, timeout: int, expected_sha256: str | None = None) -> int:
    req = urllib.request.Request(
        url,
        headers={
            'User-Agent': 'Modern-Robotics-Curriculum/1.0 (educational illustration attribution; Python urllib)',
            'Accept': 'image/png,image/*;q=0.8',
        },
    )
    with urllib.request.urlopen(req, timeout=timeout) as response:
        data = bytearray()
        while True:
            chunk = response.read(128 * 1024)
            if not chunk:
                break
            data.extend(chunk)
            if len(data) > MAX_BYTES:
                raise ValueError('PNG too large, use a Wikimedia thumbnail instead')
    validate_png(bytes(data), expected_sha256=expected_sha256)
    target.parent.mkdir(parents=True, exist_ok=True)
    # A failed or interrupted download must never replace a working asset.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, suffix='.part', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, target)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return len(data)


def main() -> int:
    parser = argparse.ArgumentParser(description='Cache online PNG illustrations for offline course HTML')
    parser.add_argument('--only', nargs='*', metavar='KEY', help='fetch only selected asset keys')
    parser.add_argument('--timeout', type=int, default=25, help='timeout for each image (seconds)')
    parser.add_argument('--check', action='store_true', help='verify all local PNGs and hashes without network access')
    args = parser.parse_args()
    assets = json.loads(MANIFEST.read_text(encoding='utf-8'))['assets']
    unknown = set(args.only or []) - set(assets)
    if unknown:
        parser.error(f'Unknown image keys: {", ".join(sorted(unknown))}')
    selected = args.only if args.only else sorted(assets)
    success = failed = 0
    for key in selected:
        asset = assets[key]
        dest = (ROOT / asset['local']).resolve()
        if not dest.is_relative_to(ROOT):
            print(f'BLOCKED outside repository: {key}', file=sys.stderr)
            failed += 1
            continue
        try:
            if dest.is_file():
                read_png(dest, asset)
                print(f'CACHED {key}: {dest.relative_to(ROOT)}')
            elif args.check:
                raise FileNotFoundError(f'Missing local PNG: {dest.relative_to(ROOT)}')
            else:
                size = download(asset.get('download_url', asset['url']), dest, timeout=args.timeout,
                                expected_sha256=asset.get('sha256'))
                print(f'OK {key}: {size:,} bytes from {asset["page"]}')
                time.sleep(1)
            success += 1
        except (OSError, ValueError, urllib.error.URLError) as exc:
            print(f'FAILED {key}: {exc}\n  source: {asset["page"]}', file=sys.stderr)
            failed += 1
            # Respect source throttling; a later run resumes from verified files.
            if isinstance(exc, urllib.error.HTTPError) and exc.code == 429:
                print('Source rate limit reached; retry later', file=sys.stderr)
                return 1
    print(f'DONE: {success} available, {failed} failed, out of {len(selected)} requested')
    if success:
        print('Run: python scripts/build_book.py && python scripts/check_book.py')
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())

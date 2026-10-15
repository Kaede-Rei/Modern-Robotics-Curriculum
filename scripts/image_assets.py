"""Shared PNG decoding and integrity checks for offline assets."""
from __future__ import annotations
import hashlib
import io
from pathlib import Path
from PIL import Image

PNG_HEADER = b'\x89PNG\r\n\x1a\n'
MAX_BYTES = 18 * 1024 * 1024


def validate_png(data: bytes, *, expected_sha256: str | None = None) -> dict:
    if not data.startswith(PNG_HEADER) or len(data) > MAX_BYTES:
        raise ValueError('Invalid PNG signature or file exceeds 18 MiB')
    try:
        with Image.open(io.BytesIO(data)) as image:
            if image.format != 'PNG':
                raise ValueError('Expected PNG')
            image.verify()
        with Image.open(io.BytesIO(data)) as image:
            image.load()
            width, height = image.size
    except (OSError, SyntaxError) as exc:
        raise ValueError(f'Damaged PNG: {exc}') from exc
    digest = hashlib.sha256(data).hexdigest()
    if expected_sha256 and digest != expected_sha256:
        raise ValueError('PNG SHA-256 differs from the source manifest')
    return {'sha256': digest, 'bytes': len(data), 'width': width, 'height': height}


def read_png(path: Path, asset: dict | None = None) -> bytes:
    data = path.read_bytes()
    metadata = validate_png(data, expected_sha256=(asset or {}).get('sha256'))
    for name in ('bytes', 'width', 'height'):
        if asset and name in asset and asset[name] != metadata[name]:
            raise ValueError(f'PNG {name} differs from manifest: {path}')
    return data

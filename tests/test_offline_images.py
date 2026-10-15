"""Regressions for a genuinely self-contained offline book."""
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import build_book
import fetch_web_png
from image_assets import validate_png, read_png


class OfflineImages(unittest.TestCase):
    def png(self):
        out = io.BytesIO()
        Image.new('RGB', (3, 2), 'white').save(out, format='PNG')
        return out.getvalue()

    def convert(self, markup):
        result = type('Result', (), {'stdout': markup, 'stderr': ''})()
        with patch.object(build_book.subprocess, 'run', return_value=result):
            return build_book.convert_markdown(build_book.ROOT / 'book/test.md', chapter='test')

    def test_remote_image_is_rejected_even_when_previously_allowlisted(self):
        url = next(iter(build_book.WEB_PNG_BY_URL))
        with self.assertRaisesRegex(ValueError, '[Oo]nline|[Rr]emote'):
            self.convert(f'<img src="{url}" alt="test">')

    def test_missing_local_image_fails(self):
        with self.assertRaises(FileNotFoundError):
            self.convert('<img src="missing.png" alt="test">')

    def test_truncated_png_is_not_embedded(self):
        with tempfile.NamedTemporaryFile(dir=build_book.ROOT, suffix='.png') as f:
            f.write(self.png()[:33]); f.flush()
            with self.assertRaises((ValueError, OSError, SyntaxError)):
                self.convert(f'<img src="../{Path(f.name).name}" alt="test">')

    def test_valid_local_png_is_embedded(self):
        with tempfile.NamedTemporaryFile(dir=build_book.ROOT, suffix='.png') as f:
            f.write(self.png()); f.flush()
            page = self.convert(f'<img src="../{Path(f.name).name}" alt="test">')
            self.assertIn('src="data:image/png;base64,', page['html'])

    def test_checksum_mismatch_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'SHA-256'):
            validate_png(self.png(), expected_sha256='0' * 64)

    def test_manifest_dimensions_are_checked(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'image.png'
            path.write_bytes(self.png())
            with self.assertRaisesRegex(ValueError, 'width'):
                read_png(path, {'width': 99})

    def test_failed_download_leaves_existing_file_intact(self):
        with tempfile.TemporaryDirectory() as d:
            dest = Path(d) / 'image.png'
            dest.write_bytes(self.png())
            response = io.BytesIO(self.png()[:33])
            with patch.object(fetch_web_png.urllib.request, 'urlopen', return_value=response):
                with self.assertRaises((ValueError, OSError, SyntaxError)):
                    fetch_web_png.download('https://example.org/x.png', dest, timeout=1)
            self.assertEqual(dest.read_bytes(), self.png())


if __name__ == '__main__':
    unittest.main()

"""Offline Linux tests; never execute a vehicle operation."""
import ast
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('menu_dispatch', ROOT / 'payload/dispatch.py')
dispatch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dispatch)


class PayloadTests(unittest.TestCase):
    def test_reference_replacement_does_not_run_or_rewrite_runner(self):
        source = "from pathlib import Path\nroot=Path('/original')\na=(root/'reference_local.json').read_text()\n"
        tree = dispatch.reference_tree(source, Path('/new/reference.json'))
        self.assertIn('/new/reference.json', ast.unparse(tree))
        self.assertIn('reference_local.json', source)
        for wrong in ['', source + source]:
            with self.assertRaises(RuntimeError):
                dispatch.reference_tree(wrong, Path('/other'))

    def test_both_routes_have_valid_ordered_points(self):
        for name in ('s_curve', 'figure8'):
            dispatch.validate_reference(json.loads((ROOT/'payload/references'/f'{name}.json').read_text()))

    def test_sync_backs_up_and_leaves_source_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            dispatch.WORK = Path(tmp)
            src = dispatch.WORK/'src/racecar/config';dst = dispatch.WORK/'install/racecar/share/racecar/config'
            src.mkdir(parents=True);dst.mkdir(parents=True)
            (src/'example.yaml').write_text('value: 2\n');(dst/'example.yaml').write_text('value: 1\n')
            dispatch.sync_config()
            self.assertEqual((src/'example.yaml').read_text(),'value: 2\n')
            self.assertEqual((dst/'example.yaml').read_text(),'value: 2\n')
            saved = list((dispatch.WORK/'config_backups').glob('*/example.yaml'))
            self.assertEqual(len(saved),1)
            self.assertEqual(saved[0].read_text(),'value: 1\n')
            dispatch.sync_config()
            self.assertEqual(len(list((dispatch.WORK/'config_backups').glob('*/example.yaml'))),1)


if __name__ == '__main__':
    unittest.main()

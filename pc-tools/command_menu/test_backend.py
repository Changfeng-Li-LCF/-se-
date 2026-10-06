import base64
import io
import json
from pathlib import Path
import shlex
import tempfile
import unittest
from unittest.mock import patch

import backend
from catalog import ACTIONS


class TransportTests(unittest.TestCase):
    def config(self):
        return dict(host='192.0.2.15', username='robot', port=2222,
                    ssh_identity_file="C:/Users/名字 with space/key'quoted",
                    verified_host_alias='racecar-verified',
                    verified_known_hosts_file='D:/Tools with space/known_hosts')

    def test_terminal_quotes_paths_and_one_remote_command(self):
        version = 'a' * 20
        for action in ACTIONS:
            text = backend.terminal_script(self.config(), action.key, version)
            command = next(line for line in text.splitlines() if line.startswith('ssh '))
            argv = shlex.split(command)
            self.assertIn("/drives/c/Users/名字 with space/key'quoted", argv)
            self.assertIn('UserKnownHostsFile="/drives/d/Tools with space/known_hosts"', argv)
            self.assertIn('StrictHostKeyChecking=yes', argv)
            self.assertEqual(argv[-2], 'robot@192.0.2.15')
            self.assertEqual(argv[-1], backend.remote_command(action.key, version))
            self.assertNotIn('\r', text)

    def test_unknown_actions_cannot_inject_commands(self):
        for name in ['keyboard; rm -rf /', '$(touch /tmp/test)', '', '../keyboard']:
            with self.assertRaises(ValueError):
                backend.remote_command(name)

    def test_installer_is_idempotent_and_does_not_execute_payload(self):
        # Executing this fake payload would fail; installation must only write it.
        files = {name: b'raise RuntimeError("must not execute")\n' for name in backend.PAYLOAD_NAMES}
        version = backend.payload_version(files)
        request = json.dumps({'version': version, 'files': {n: base64.b64encode(v).decode() for n, v in files.items()}})
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for _ in range(2):
                with patch('pathlib.Path.home', return_value=root), patch('sys.stdin', io.StringIO(request)), patch('sys.stdout', io.StringIO()):
                    exec(backend.INSTALLER, {})
            dest = root / '.local/share/racecar-command-menu/versions' / version
            self.assertEqual({str(p.relative_to(dest)).replace('\\','/') for p in dest.rglob('*') if p.is_file()}, set(files))
            for name, content in files.items():
                self.assertEqual((dest / name).read_bytes(), content)
            (dest / 'dispatch.py').write_bytes(b'concurrent edit')
            with patch('pathlib.Path.home', return_value=root), patch('sys.stdin', io.StringIO(request)), self.assertRaises(AssertionError):
                exec(backend.INSTALLER, {})
            self.assertEqual((dest / 'dispatch.py').read_bytes(), b'concurrent edit')

    def test_installer_rejects_outside_paths(self):
        files = {name: b'noop' for name in backend.PAYLOAD_NAMES}
        files['../outside'] = b'bad'
        request = json.dumps({'version': backend.payload_version(files), 'files': {n: base64.b64encode(v).decode() for n, v in files.items()}})
        with patch('sys.stdin', io.StringIO(request)), self.assertRaises(AssertionError):
            exec(backend.INSTALLER, {})


if __name__ == '__main__':
    unittest.main()

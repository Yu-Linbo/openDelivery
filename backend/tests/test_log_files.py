import logging
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import diagnostic_logging
import log_files


class LogFilesTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for directory in log_files.roots(self.root).values():
            directory.mkdir(parents=True)

    def write(self, relative, content):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
        return path

    def test_recording_symlink_resolves_to_archived_log(self):
        recorded = self.write('log_bag/robot1/backup/logs/a_terminal_log.txt', 'recorded')
        (self.root / 'log_bag/robot1/current_terminal_log.txt').symlink_to(recorded)
        result = log_files.read_log(self.root, 'log_bag/robot1/current_terminal_log.txt')
        self.assertEqual(result['path'], 'log_bag/robot1/backup/logs/a_terminal_log.txt')
        self.assertEqual(result['text'], 'recorded')

    def test_reject_traversal_non_log_and_symlink_escape(self):
        private = self.write('backend/private.txt', 'private')
        (self.root / 'log_bag/robot1/backup/logs').mkdir(parents=True)
        (self.root / 'log_bag/robot1/backup/logs/escape.log').symlink_to(private)
        for path in ['backend/private.txt', 'log_bag/robot1/backup/logs/../private.txt', 'log_bag/robot1/backup/logs/escape.log', 'log_bag/robot1/backup/logs/config.json']:
            with self.subTest(path=path), self.assertRaises(ValueError):
                log_files.read_log(self.root, path)

    def test_utf8_tail_only_keeps_complete_lines(self):
        self.write('log_bag/robot1/backup/logs/large.log', '旧行\n' * 20 + '最后一行\n')
        with patch.object(log_files, 'MAX_READ_BYTES', 20):
            result = log_files.read_log(self.root, 'log_bag/robot1/backup/logs/large.log')
        self.assertTrue(result['truncated'])
        self.assertEqual(result['text'], '最后一行\n')
        self.assertNotIn('\ufffd', result['text'])

    def test_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            log_files.read_log(self.root, 'log_bag/robot1/backup/logs/missing.log')

    def test_formatter_severity_and_time(self):
        record = logging.LogRecord('opendelivery.server', logging.WARNING, '', 0, 'failed %s', ('robot1',), None)
        record.created = 12.5
        self.assertEqual(diagnostic_logging.RosFormatter().format(record), '[WARN] [12.500000000] [opendelivery.server]: failed robot1')


if __name__ == '__main__':
    unittest.main()

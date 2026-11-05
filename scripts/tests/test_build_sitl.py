"""Verify that building the clock fix cannot silently overwrite upstream edits."""

from pathlib import Path
import tempfile
import unittest

from scripts.build_sitl import SOURCE, patched_source


class SITLBuildTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / SOURCE
        self.source.parent.mkdir(parents=True)
        self.original = b"int clock = 1;\n"
        self.source.write_bytes(self.original)
        self.patch = self.root / "clock.patch"
        self.patch.write_text(f"--- a/{SOURCE}\n+++ b/{SOURCE}\n@@ -1 +1 @@\n-int clock = 1;\n+int clock = 2;\n")

    def test_restores_source_after_success_and_failure(self):
        for fail in (False, True):
            with self.subTest(fail=fail):
                timestamp = self.source.stat().st_mtime_ns
                try:
                    with patched_source(self.root, self.patch):
                        self.assertEqual(self.source.read_bytes(), b"int clock = 2;\n")
                        if fail:
                            raise ValueError("compiler failed")
                except ValueError:
                    self.assertTrue(fail)
                self.assertEqual(self.source.read_bytes(), self.original)
                self.assertEqual(self.source.stat().st_mtime_ns, timestamp)
                self.assertFalse((self.root / ".swarm-sitl-build.lock").exists())

    def test_preserves_concurrent_source_edit_and_backup(self):
        with self.assertRaisesRegex(RuntimeError, "Source changed"):
            with patched_source(self.root, self.patch):
                self.source.write_bytes(b"user edit\n")
        self.assertEqual(self.source.read_bytes(), b"user edit\n")
        self.assertEqual((self.root / ".swarm-sitl-source-backup" / SOURCE).read_bytes(), self.original)

    def test_existing_lock_leaves_source_untouched(self):
        (self.root / ".swarm-sitl-build.lock").touch()
        with self.assertRaises(FileExistsError):
            with patched_source(self.root, self.patch):
                self.fail("build lock ignored")
        self.assertEqual(self.source.read_bytes(), self.original)


if __name__ == "__main__":
    unittest.main()

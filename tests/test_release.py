import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import release  # noqa: E402


class ReleaseTests(unittest.TestCase):
    def write_metadata(self, root: Path, version: str, changelog: str) -> None:
        (root / "VERSION").write_text(version + "\n", encoding="utf-8")
        (root / "CHANGELOG.md").write_text(changelog, encoding="utf-8")

    def test_version_from_tag_requires_strict_semver(self):
        self.assertEqual(release.version_from_tag("v1.4.0"), "1.4.0")
        for invalid in ("1.4.0", "v1.4", "v01.4.0", "v1.4.0-dev"):
            with self.assertRaises(release.ReleaseError):
                release.version_from_tag(invalid)

    def test_validate_metadata_matches_version_and_changelog(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_metadata(root, "2.0.0", "# Changelog\n\n## 2.0.0 - 2026-10-01\n\n- New.\n")
            version, section = release.validate_metadata("v2.0.0", root)
            self.assertEqual(version, "2.0.0")
            self.assertEqual(section, "- New.")

    def test_validate_metadata_rejects_version_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_metadata(root, "2.0.1", "# Changelog\n\n## 2.0.0 - 2026-10-01\n\n- New.\n")
            with self.assertRaises(release.ReleaseError):
                release.validate_metadata("v2.0.0", root)

    def test_changelog_rejects_invalid_release_date(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_metadata(root, "2.0.0", "# Changelog\n\n## 2.0.0 - 2026-99-99\n\n- New.\n")
            with self.assertRaises(release.ReleaseError):
                release.validate_metadata("v2.0.0", root)

    def test_changelog_section_stops_at_next_release(self):
        changelog = (
            "# Changelog\n\n"
            "## 2.0.0 - 2026-10-01\n\n### Added\n\n- New.\n\n"
            "## 1.9.0 - 2026-09-01\n\n- Old.\n"
        )
        section = release.changelog_section(changelog, "2.0.0")
        self.assertIn("### Added", section)
        self.assertIn("- New.", section)
        self.assertNotIn("Old", section)

    def test_render_notes_records_validation_limits(self):
        notes = release.render_notes("### Added\n\n- New.", "abc123")
        self.assertIn("Release commit: `abc123`", notes)
        self.assertIn("make container-check", notes)
        self.assertIn("No firmware binary is generated or attached automatically", notes)
        self.assertIn("not a flashable firmware artifact", notes)

    def test_annotated_tag_must_point_to_checked_out_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(["git", "init", "-q", "-b", "master", root], check=True)
            subprocess.run(["git", "-C", root, "config", "user.name", "Test"], check=True)
            subprocess.run(
                ["git", "-C", root, "config", "user.email", "test@example.com"], check=True
            )
            self.write_metadata(root, "2.0.0", "# Changelog\n\n## 2.0.0 - 2026-10-01\n\n- New.\n")
            subprocess.run(["git", "-C", root, "add", "."], check=True)
            subprocess.run(["git", "-C", root, "commit", "-qm", "release"], check=True)
            subprocess.run(["git", "-C", root, "tag", "-a", "v2.0.0", "-m", "v2.0.0"], check=True)
            release.check_release("v2.0.0", True, root, "master")

            subprocess.run(["git", "-C", root, "tag", "lightweight"], check=True)
            with self.assertRaises(release.ReleaseError):
                release.require_annotated_tag("lightweight", root)

            subprocess.run(["git", "-C", root, "checkout", "-qb", "feature"], check=True)
            (root / "later.txt").write_text("later\n", encoding="utf-8")
            subprocess.run(["git", "-C", root, "add", "."], check=True)
            subprocess.run(["git", "-C", root, "commit", "-qm", "later"], check=True)
            with self.assertRaises(release.ReleaseError):
                release.require_commit_on_branch("master", root)


if __name__ == "__main__":
    unittest.main()

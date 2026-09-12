from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class RepositoryMetadataTests(unittest.TestCase):
    def test_version_is_development_semver(self):
        version = (ROOT / "VERSION").read_text().strip()
        self.assertRegex(version, r"^\d+\.\d+\.\d+-dev$")
        self.assertIn(version, (ROOT / "CHANGELOG.md").read_text())

    def test_standalone_licenses_exist(self):
        mit = (ROOT / "LICENSES" / "MIT.txt").read_text()
        cern = (ROOT / "LICENSES" / "CERN-OHL-1.2.txt").read_text()
        self.assertIn("MIT License", mit)
        self.assertIn("Imperial College Space Society", mit)
        self.assertIn("CERN Open Hardware Licence v1.2", cern)

    def test_iar_project_source_references_exist(self):
        projects = [
            ROOT / "firmware" / "HC12Tracker.ewp",
            ROOT / "test_firmware" / "HC12CW" / "HC12Tracker.ewp",
            ROOT / "test_firmware" / "HC12MOD" / "HC12Tracker.ewp",
        ]
        missing = []
        for project in projects:
            tree = ET.parse(project)
            for node in tree.iter("name"):
                text = node.text or ""
                if not text.startswith("$PROJ_DIR$"):
                    continue
                relative = text[len("$PROJ_DIR$\\"):].replace("\\", "/")
                path = project.parent / relative
                if not path.exists():
                    missing.append(f"{project.relative_to(ROOT)}: {relative}")
        self.assertEqual(missing, [])


if __name__ == "__main__":
    unittest.main()

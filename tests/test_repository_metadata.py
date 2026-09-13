from pathlib import Path
import json
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class RepositoryMetadataTests(unittest.TestCase):
    def test_version_is_semver(self):
        version = (ROOT / "VERSION").read_text().strip()
        self.assertRegex(version, r"^\d+\.\d+\.\d+(?:-dev)?$")
        self.assertIn(version, (ROOT / "CHANGELOG.md").read_text())

    def test_root_makefile_exposes_developer_entry_points(self):
        makefile = (ROOT / "Makefile").read_text()
        for target in ("help:", "demo:", "simulate:", "test:", "stm8:", "decode:", "check:", "clean:", "container-build:", "container-check:"):
            self.assertIn(target, makefile)

    def test_reproducible_development_environment(self):
        config = json.loads((ROOT / ".devcontainer" / "devcontainer.json").read_text())
        dockerfile = (ROOT / ".devcontainer" / "Dockerfile").read_text()
        workflow = (ROOT / ".github" / "workflows" / "host-tests.yml").read_text()
        self.assertEqual(config["build"]["dockerfile"], "Dockerfile")
        self.assertEqual(config["remoteUser"], "developer")
        for expected in ("python:3.12-slim-bookworm", "build-essential", "sdcc=${SDCC_VERSION}"):
            self.assertIn(expected, dockerfile)
        self.assertIn("make container-check", workflow)

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

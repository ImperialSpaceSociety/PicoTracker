#!/usr/bin/env python3
"""Validate PicoTracker release metadata and render release notes."""

import argparse
import re
import subprocess
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TAG_RE = re.compile(r"^v(?P<version>(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\.(?:0|[1-9]\d*))$")


class ReleaseError(ValueError):
    """Release metadata is inconsistent or incomplete."""


def version_from_tag(tag: str) -> str:
    match = TAG_RE.fullmatch(tag)
    if not match:
        raise ReleaseError(f"release tag must use vX.Y.Z format: {tag}")
    return match.group("version")


def read_version(root: Path = ROOT) -> str:
    return (root / "VERSION").read_text(encoding="utf-8").strip()


def read_changelog(root: Path = ROOT) -> str:
    return (root / "CHANGELOG.md").read_text(encoding="utf-8")


def changelog_section(changelog: str, version: str) -> str:
    heading = re.compile(
        rf"^## {re.escape(version)} - (?P<date>\d{{4}}-\d{{2}}-\d{{2}})[ \t]*$",
        re.MULTILINE,
    )
    matches = list(heading.finditer(changelog))
    if len(matches) != 1:
        raise ReleaseError(
            f"CHANGELOG.md must contain exactly one `## {version} - YYYY-MM-DD` heading"
        )
    try:
        date.fromisoformat(matches[0].group("date"))
    except ValueError as exc:
        raise ReleaseError(f"CHANGELOG.md has an invalid release date for {version}") from exc

    start = matches[0].end()
    next_heading = re.search(r"^##\s+", changelog[start:], re.MULTILINE)
    end = start + next_heading.start() if next_heading else len(changelog)
    section = changelog[start:end].strip()
    if not section:
        raise ReleaseError(f"CHANGELOG.md section for {version} is empty")
    return section


def validate_metadata(tag: str, root: Path = ROOT) -> tuple[str, str]:
    version = version_from_tag(tag)
    recorded_version = read_version(root)
    if recorded_version != version:
        raise ReleaseError(
            f"tag {tag} does not match VERSION ({recorded_version}); expected {version}"
        )
    return version, changelog_section(read_changelog(root), version)


def git_output(*args: str, root: Path = ROOT) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise ReleaseError(f"git {' '.join(args)} failed: {detail}")
    return result.stdout.strip()


def require_annotated_tag(tag: str, root: Path = ROOT) -> str:
    object_type = git_output("cat-file", "-t", tag, root=root)
    if object_type != "tag":
        raise ReleaseError(f"{tag} must be an annotated tag, found Git object type {object_type}")
    return git_output("rev-parse", f"{tag}^{{}}", root=root)


def current_commit(root: Path = ROOT) -> str:
    return git_output("rev-parse", "HEAD", root=root)


def require_commit_on_branch(reference: str, root: Path = ROOT) -> None:
    result = subprocess.run(
        ["git", "merge-base", "--is-ancestor", "HEAD", reference],
        cwd=root,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        raise ReleaseError(f"checked-out release commit is not contained in {reference}")


def render_notes(section: str, commit_sha: str) -> str:
    return (
        f"Release commit: `{commit_sha}`\n\n"
        f"{section}\n\n"
        "### Release validation\n\n"
        "- Automated publication gate: the full reproducible `make container-check` suite.\n"
        "- GitHub provides source-code archives for this tag automatically.\n"
        "- No firmware binary is generated or attached automatically. Only a native-IAR and "
        "hardware-validated firmware artifact should be added separately.\n"
        "- The SDCC structural image used by CI is not a flashable firmware artifact.\n"
    )


def check_release(
    tag: str, require_tag: bool, root: Path = ROOT, require_branch: str | None = None
) -> tuple[str, str]:
    version, section = validate_metadata(tag, root)
    if require_tag:
        tagged_commit = require_annotated_tag(tag, root)
        head_commit = current_commit(root)
        if tagged_commit != head_commit:
            raise ReleaseError(
                f"{tag} points to {tagged_commit}, but the checked-out commit is {head_commit}"
            )
    if require_branch:
        require_commit_on_branch(require_branch, root)
    return version, section


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    check = subparsers.add_parser("check", help="validate tag, VERSION, and changelog metadata")
    check.add_argument("--tag", required=True)
    check.add_argument(
        "--require-annotated",
        action="store_true",
        help="require an annotated Git tag pointing at the checked-out commit",
    )
    check.add_argument(
        "--require-branch",
        help="require the checked-out release commit to be contained in this Git ref",
    )

    notes = subparsers.add_parser("notes", help="render release notes from CHANGELOG.md")
    notes.add_argument("--tag", required=True)
    notes.add_argument("--sha", help="release commit SHA; defaults to HEAD")
    notes.add_argument("--output", type=Path, help="write notes to a file instead of stdout")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == "check":
            version, _ = check_release(
                args.tag, args.require_annotated, require_branch=args.require_branch
            )
            print(f"release metadata OK: {args.tag} / VERSION {version}")
            return 0

        version, section = validate_metadata(args.tag)
        commit_sha = args.sha or current_commit()
        notes = render_notes(section, commit_sha)
        if args.output:
            args.output.write_text(notes, encoding="utf-8")
            print(f"wrote release notes for {args.tag} to {args.output}")
        else:
            print(notes, end="")
        return 0
    except ReleaseError as exc:
        raise SystemExit(f"release error: {exc}") from exc


if __name__ == "__main__":
    raise SystemExit(main())

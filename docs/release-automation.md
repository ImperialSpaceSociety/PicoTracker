# Release automation

PicoTracker releases are published from annotated semantic-version tags. The tag is the final publication trigger; target and hardware validation remain maintainer decisions made before the tag is pushed.

## Before tagging

Prepare the release on `master`:

1. Set `VERSION` to the final `X.Y.Z` value with no development suffix.
2. Add a matching `## X.Y.Z - YYYY-MM-DD` section to `CHANGELOG.md`.
3. Complete the source, target-build, and hardware gates in [`release-checklist.md`](release-checklist.md).
4. Confirm the normal `master` CI run is green.
5. Preview the automated metadata and notes:

```sh
make release-check RELEASE_TAG=vX.Y.Z
make release-notes RELEASE_TAG=vX.Y.Z
```

These commands do not create a tag or a GitHub Release.

## Publish

Create and push an annotated tag from the intended release commit:

```sh
git checkout master
git pull --ff-only
git tag -a vX.Y.Z -m "PicoTracker vX.Y.Z"
git push origin vX.Y.Z
```

Pushing the tag starts `.github/workflows/release.yml`. The workflow:

1. checks out the complete Git history and tags;
2. verifies strict `vX.Y.Z` syntax;
3. verifies that `VERSION` exactly matches the tag;
4. verifies that `CHANGELOG.md` contains exactly one matching release section;
5. requires the pushed tag to be annotated and to point to the checked-out commit;
6. verifies that the release commit is contained in `origin/master`;
7. runs the full reproducible `make container-check` suite;
8. renders the matching changelog section into release notes; and
9. creates the GitHub Release and marks it as the latest release.

If any validation or test step fails, the GitHub Release is not created.

## Firmware artifacts

The automated workflow deliberately does **not** publish the SDCC structural image. That image uses IAR compatibility shims and is not flashable firmware.

If a native-IAR binary has been validated on the target hardware, attach it to the GitHub Release separately and publish its checksum. A source-only release is acceptable when no validated binary is available, but the release record must not imply that CI produced a flashable artifact.

## Release notes

`tools/release.py` extracts only the matching version section from `CHANGELOG.md` and appends a standard validation note covering the reproducible CI gate and firmware-artifact limitation. This keeps the changelog as the release-note source of truth instead of maintaining a second copy by hand.

The existing `v1.4.0` release predates this automation and remains unchanged.

# Release checklist

Use this checklist before creating a PicoTracker release tag. The current version is recorded in [`../VERSION`](../VERSION).

**v1.4.0 record:** repository/CI gates passed and the maintainer reported successful target/hardware validation. Native IAR build metadata and a flashable binary artifact were not archived, so this release is source-only.

## Source and regression gates

- [ ] `master` is clean and contains the intended release changes.
- [ ] `VERSION` is changed from the development suffix to the release version.
- [ ] `CHANGELOG.md` contains release notes for that exact version.
- [ ] GitHub Actions host tests pass on the release commit.
- [ ] `make check` passes locally with an STM8-capable SDCC installation.
- [ ] All IAR project source references resolve to tracked files.
- [ ] No release-blocking TODO, known infinite wait, or unresolved protocol error remains.

## Target build gates

Only the maintained `firmware/HC12Tracker.ewp` project is a release target. The projects under `test_firmware/` are historical diagnostics and are not release-qualified.

- [x] All production IAR project source references resolve to tracked files.
- [x] Independent SDCC STM8 structural compile/link passes within the STM8S003F3 flash/RAM window.
- [x] Maintainer reported successful target build and operation for `v1.4.0`.
- [ ] Native IAR version/build-size output and flashable binary artifact archived. *(Not available for v1.4.0; non-blocking by maintainer release decision.)*

## Hardware gates

- [x] Maintainer reported successful target/hardware validation for `v1.4.0`.
- [x] GPS acquisition, telemetry, radio transmission, sleep/wake and power cycling accepted by maintainer for release.
- [x] [`hardware-validation.md`](hardware-validation.md) records the validation basis and the absence of archived quantitative measurements.

## Release publication

- [ ] Review [`../LICENSE.md`](../LICENSE.md) and the standalone license files.
- [ ] Update README verification status if target validation is complete.
- [ ] Create an annotated `vX.Y.Z` tag only after all required gates pass.
- [ ] Publish release notes with build metadata, hardware configuration, known limitations, and artifact checksums.
- [ ] Attach the validated firmware artifact to the GitHub release.

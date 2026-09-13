# Release checklist

Use this checklist before creating a PicoTracker release tag. The current version is recorded in [`../VERSION`](../VERSION).

**v1.4.0 record:** repository/CI gates passed and the maintainer reported successful target/hardware validation. Native IAR build metadata and a flashable binary artifact were not archived, so this release is source-only.

## Source and regression gates

- [ ] `master` is clean and contains the intended release changes.
- [ ] `VERSION` is changed from the development suffix to the release version.
- [ ] `CHANGELOG.md` contains release notes for that exact version.
- [ ] `make release-check RELEASE_TAG=vX.Y.Z` passes.
- [ ] `make release-notes RELEASE_TAG=vX.Y.Z` has been reviewed.
- [ ] GitHub Actions host tests pass on the release commit.
- [ ] `make check` passes locally with an STM8-capable SDCC installation.
- [ ] All IAR project source references resolve to tracked files.
- [ ] No release-blocking TODO, known infinite wait, or unresolved protocol error remains.

## Target build gates

Only the maintained `firmware/HC12Tracker.ewp` project is a release target. The projects under `test_firmware/` are historical diagnostics and are not release-qualified.

- [ ] All production IAR project source references resolve to tracked files.
- [ ] Independent SDCC STM8 structural compile/link passes within the STM8S003F3 flash/RAM window.
- [ ] Native IAR target build and operation have been validated for this release where the toolchain/hardware is available.
- [ ] Native IAR version/build-size output and any flashable binary intended for publication are archived.
- [ ] If a native binary is unavailable, the maintainer has explicitly approved a source-only release and recorded that limitation.

## Hardware gates

- [ ] GPS acquisition, telemetry, radio transmission, sleep/wake, and power cycling have been accepted on the intended hardware, where applicable.
- [ ] [`hardware-validation.md`](hardware-validation.md) records the validation basis, hardware configuration, and any missing measurements or limitations.
- [ ] Any source-only or hardware-not-revalidated exception is explicitly approved by the maintainer before tagging.

## Release publication

- [ ] Review [`../LICENSE.md`](../LICENSE.md) and the standalone license files.
- [ ] Update README verification status if target validation is complete.
- [ ] Create and push an annotated `vX.Y.Z` tag only after all required gates pass.
- [ ] Confirm the automated `Release` workflow completes successfully.
- [ ] Confirm the generated GitHub Release contains the intended changelog section and release commit.
- [ ] If a native-IAR/hardware-validated firmware artifact is available, attach it separately with its checksum.
- [ ] If no validated firmware artifact is available, retain the release as source-only and do not attach the SDCC structural image.

See [`release-automation.md`](release-automation.md) for the tag-driven publication process.
